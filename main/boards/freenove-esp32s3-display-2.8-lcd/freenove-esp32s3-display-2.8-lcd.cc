// File: src/boards/freenove_esp32s3_display.cc

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <wifi_station.h>
#include "application.h"
#include "audio_codecs/no_audio_codec.h"
#include "audio_codecs/es8311_audio_codec.h"
#include "button.h"
#include "display/lcd_display.h"
#include "led/single_led.h"
#include "system_reset.h"
#include "wifi_board.h"
#include "mcp_server.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_lvgl_port.h>

#include "esp_lcd_ili9341.h"
#include <hal/adc_types.h>
#include <driver/i2c.h>


#define TAG "FreenoveESP32S3Display"


LV_FONT_DECLARE(font_awesome_16_4);
LV_FONT_DECLARE(font_viet_16);
LV_FONT_DECLARE(font_viet_20);
LV_FONT_DECLARE(font_viet_24);

class FreenoveESP32S3Display : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay *display_ = nullptr;
    i2c_master_bus_handle_t codec_i2c_bus_;
    i2c_master_dev_handle_t touch_dev_handle_ = nullptr;
    bool touch_initialized_ = false;

    int last_battery_level_ = 0;

    bool last_charging_state_ = false;
    bool stt_only_active_ = false;
    bool current_ps_mode_ = false; // Tracks current PowerSave state to prevent log spam

    /* ================= ACTION LOGIC ================= */

    void ActionToggleChat() {
        auto &app = Application::GetInstance();
        ESP_LOGI(TAG, "Chat Action: Toggle chat state (Current device state: %d)", app.GetDeviceState());
        if (app.GetDeviceState() == kDeviceStateStarting &&
            !WifiStation::GetInstance().IsConnected()) {
            ESP_LOGW(TAG, "Device starting without WiFi - resetting configuration");
            ResetWifiConfiguration();
            return;
        }
        app.ToggleChatState();
    }

    void ActionToggleSttMode() {
        auto &app = Application::GetInstance();
        stt_only_active_ = !stt_only_active_;
        ESP_LOGI(TAG, "State Change: STT Only Mode switched to %s", stt_only_active_ ? "ENABLED" : "DISABLED");
        app.SetSttOnlyMode(stt_only_active_);
    }

    void ActionToggleAudioTesting() {
        auto &app = Application::GetInstance();
        bool entering = app.GetDeviceState() != kDeviceStateAudioTesting;
        ESP_LOGI(TAG, "Mode Change: %s audio testing mode", entering ? "Entering" : "Exiting");
        if (entering) app.EnterAudioTestingMode();
        else app.ExitAudioTestingMode();
    }

    /* ================= TOUCH HELPERS ================= */

    bool ReadTouchRegister(uint8_t reg, uint8_t *data, size_t len) {
        if (!touch_initialized_ && reg != FT6336_FIRMWARE_ID) return false;
        esp_err_t ret = i2c_master_transmit_receive(touch_dev_handle_, &reg, 1, data, len, -1);
        if (ret != ESP_OK) {
            ESP_LOGV(TAG, "Touch I2C error at reg 0x%02X", reg);
        }
        return ret == ESP_OK;
    }

    bool WriteTouchRegister(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {reg, value};
        esp_err_t ret = i2c_master_transmit(
            touch_dev_handle_, buf, 2, -1);
        return ret == ESP_OK;
    }

    bool ReadTouchPoint(uint16_t &x, uint16_t &y) {
        uint8_t count = 0;
        if (!ReadTouchRegister(FT6336_TD_STATUS, &count, 1)) return false;
        count &= 0x0F;
        if (count == 0 || count > 2) return false;

        uint8_t data[4];
        if (!ReadTouchRegister(FT6336_TOUCH1_XH, data, 4)) return false;

        x = ((data[0] & 0x0F) << 8) | data[1];
        y = ((data[2] & 0x0F) << 8) | data[3];

        x = std::min((uint16_t)319, x);
        y = std::min((uint16_t)239, y);
        return true;
    }

    void InitializeTouch() {
        ESP_LOGI(TAG, "Initializing FT6336 Touch on I2C address 0x%02X", FT6336_ADDR);
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = FT6336_ADDR,
            .scl_speed_hz = 400000,
        };

        if (i2c_master_bus_add_device(codec_i2c_bus_, &cfg,
                                      &touch_dev_handle_) != ESP_OK) {
            ESP_LOGE(TAG, "Hardware Error: Failed to add touch device to I2C bus");
            return;
        }

        uint8_t id;
        if (ReadTouchRegister(FT6336_FIRMWARE_ID, &id, 1)) {
            ESP_LOGI(TAG, "Touch Chip identified successfully. Firmware ID: 0x%02X", id);
            touch_initialized_ = true;
        } else {
            ESP_LOGE(TAG, "Hardware Error: Touch chip FT6336 not responding");
        }
        WriteTouchRegister(FT6336_CTRL, 0x00);
        WriteTouchRegister(FT6336_THRESHOLD, 0x14);
    }

    static void LvglTouchCb(lv_indev_t *indev, lv_indev_data_t *data) {
        auto *self = static_cast<FreenoveESP32S3Display *>(
            lv_indev_get_user_data(indev));

        uint16_t x, y;
        if (self && self->ReadTouchPoint(x, y)) {
            data->state = LV_INDEV_STATE_PRESSED;
            data->point.x = x;
            data->point.y = y;
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }
    }

    static void OnTouchEvents(lv_event_t *e) {
        auto *self = static_cast<FreenoveESP32S3Display *>(lv_event_get_user_data(e));
        lv_event_code_t code = lv_event_get_code(e);

        if (code == LV_EVENT_CLICKED) {
            static uint32_t last_click = 0;
            uint32_t now = lv_tick_get();
            if (now - last_click < 350) {
                ESP_LOGI(TAG, "Touch Event: Double Click");
                self->ActionToggleSttMode();
                last_click = 0;
            } else {
                last_click = now;
                ESP_LOGI(TAG, "Touch Event: Single Click");
                self->ActionToggleChat();
            }
        } else if (code == LV_EVENT_LONG_PRESSED) {
            ESP_LOGI(TAG, "Touch Event: Long Press");
            self->ActionToggleSttMode();
        }
    }

    /* ================= HARDWARE INIT ================= */

    void InitializeI2c() {
        ESP_LOGI(TAG, "Initializing I2C Master Bus (SDA: %d, SCL: %d)", AUDIO_CODEC_I2C_SDA_PIN, AUDIO_CODEC_I2C_SCL_PIN);
        i2c_master_bus_config_t cfg = {
            .i2c_port = AUDIO_CODEC_I2C_NUM,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .flags = {.enable_internal_pullup = 1},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &codec_i2c_bus_));
    }

    void InitializeSpi() {
        ESP_LOGI(TAG, "Initializing SPI Bus (MOSI: %d, MISO: %d, SCLK: %d)", DISPLAY_MOSI_PIN, DISPLAY_MIS0_PIN, DISPLAY_SCK_PIN);
        spi_bus_config_t cfg = {};
        cfg.mosi_io_num = DISPLAY_MOSI_PIN;
        cfg.miso_io_num = DISPLAY_MIS0_PIN;
        cfg.sclk_io_num = DISPLAY_SCK_PIN;
        cfg.max_transfer_sz =
                DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(
            spi_bus_initialize(LCD_SPI_HOST, &cfg, SPI_DMA_CH_AUTO));
    }


    void InitializeLcdDisplay() {
        ESP_LOGI(TAG, "Initializing ILI9341 Display Panel (CS: %d, DC: %d, RST: %d)", DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);
        esp_lcd_panel_io_handle_t io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_cfg = {};
        io_cfg.cs_gpio_num = DISPLAY_CS_PIN;
        io_cfg.dc_gpio_num = DISPLAY_DC_PIN;
        io_cfg.spi_mode = DISPLAY_SPI_MODE;
        io_cfg.pclk_hz = DISPLAY_SPI_SCLK_HZ;
        io_cfg.trans_queue_depth = 10;
        io_cfg.lcd_cmd_bits = 8;
        io_cfg.lcd_param_bits = 8;

        ESP_ERROR_CHECK(
            esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_cfg, &io));

        esp_lcd_panel_dev_config_t panel_cfg = {};
        panel_cfg.reset_gpio_num = DISPLAY_RST_PIN;
        panel_cfg.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_cfg.bits_per_pixel = 16;

        ESP_ERROR_CHECK(
            esp_lcd_new_panel_ili9341(io, &panel_cfg, &panel));

        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new SpiLcdDisplay(
            io, panel,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
            {
            .text_font = &font_viet_20,
                .icon_font = &font_awesome_16_4,
            .emoji_font = DISPLAY_HEIGHT >= 240 ? font_emoji_64_init() : font_emoji_32_init(),
            .font_16 = &font_viet_16,
            .font_20 = &font_viet_20,
            .font_24 = &font_viet_24
            });
        ESP_LOGI(TAG, "Display object created successfully");
    }

    void InitializeButtons() {
        ESP_LOGI(TAG, "Configuring Boot Button (GPIO: %d)", BOOT_BUTTON_GPIO);
        boot_button_.OnClick([this]() {
            ActionToggleChat();
        });

        boot_button_.OnLongPress([this]() {
            ActionToggleSttMode();
        });

        boot_button_.OnDoubleClick([this]() {
            ActionToggleSttMode();
        });

        boot_button_.OnMultipleClick([this]() {
            ActionToggleAudioTesting();
        }, 3);
    }

public:
    FreenoveESP32S3Display() : boot_button_(BOOT_BUTTON_GPIO) {
        ESP_LOGI(TAG, "Freenove Board Initialization Start");
        InitializeI2c();
        InitializeSpi();
        InitializeTouch();
        InitializeLcdDisplay();
        InitializeButtons();

        if (touch_initialized_) {
            lvgl_port_lock(0);
            lv_indev_t *indev = lv_indev_create();
            lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
            lv_indev_set_read_cb(indev, LvglTouchCb);
            lv_indev_set_user_data(indev, this);

            lv_obj_t *content = display_->GetContentObject();

            lv_obj_add_flag(content, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_flag(content, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_scrollbar_mode(content, LV_SCROLLBAR_MODE_AUTO);
            lv_obj_set_scroll_dir(content, LV_DIR_VER);

            lv_obj_add_event_cb(content, OnTouchEvents, LV_EVENT_ALL, this);
            lvgl_port_unlock();
            ESP_LOGI(TAG, "LVGL Touch interface registered");
        }

        GetBacklight()->SetBrightness(100);
        current_ps_mode_ = false; // Initial state is full brightness
        ESP_LOGI(TAG, "Freenove Board Initialization Complete");
    }

    ~FreenoveESP32S3Display() {
        if (touch_dev_handle_)
            i2c_master_bus_rm_device(touch_dev_handle_);
    }

    Led *GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    AudioCodec *GetAudioCodec() override {
        static Es8311AudioCodec codec(codec_i2c_bus_, AUDIO_CODEC_I2C_NUM, AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR, true, true);
        return &codec;
    }

    Display *GetDisplay() override { return display_; }

    Backlight *GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN,
                                      DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    void SetPowerSaveMode(bool enable) override {
        if (current_ps_mode_ == enable) return; // Silent if no state change

        WifiBoard::SetPowerSaveMode(enable);

        auto backlight = GetBacklight();
        if (backlight) {
            ESP_LOGI(TAG, "PowerSave Mode: %s (Brightness set to %d%%)", enable ? "ENABLED" : "DISABLED", enable ? 10 : 100);
            backlight->SetBrightness(enable ? 10 : 100);
        }
        current_ps_mode_ = enable;
    }

    bool GetBatteryLevel(
        int &level, bool &charging, bool &discharging) override {
        static uint32_t last_check = 0;
        uint32_t now = esp_log_timestamp();

        if (last_check && (now - last_check) < 10000) {
            level = last_battery_level_;
            charging = last_charging_state_;
            discharging = !charging;
            return true;
        }

        adc_oneshot_unit_handle_t handle = nullptr;
        adc_oneshot_unit_init_cfg_t init_cfg = {.unit_id = ADC_UNIT_1};

        if (adc_oneshot_new_unit(&init_cfg, &handle) == ESP_OK) {
            adc_oneshot_chan_cfg_t chan_cfg = {
                .atten = ADC_ATTEN_DB_12,
                .bitwidth = ADC_BITWIDTH_DEFAULT
            };
            adc_oneshot_config_channel(
                handle, ADC_CHANNEL_8, &chan_cfg);

            int adc_raw = 0;
            if (adc_oneshot_read(handle, ADC_CHANNEL_8, &adc_raw) == ESP_OK) {
                int real_mv = ((adc_raw * 3300) / 4095) * 2;
                level = std::clamp((real_mv - 3500) / 6, 0, 100);
                last_battery_level_ = level;
            }
            adc_oneshot_del_unit(handle);
            last_check = now;
        }

        level = last_battery_level_;
        charging = last_charging_state_;
        discharging = !charging;
        return true;
    }
};

DECLARE_BOARD(FreenoveESP32S3Display);
