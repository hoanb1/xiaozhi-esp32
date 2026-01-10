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

LV_FONT_DECLARE(font_viet_20);
LV_FONT_DECLARE(font_awesome_16_4);

class FreenoveESP32S3Display : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay *display_;
    i2c_master_bus_handle_t codec_i2c_bus_;
    i2c_master_dev_handle_t touch_dev_handle_ = nullptr;
    bool touch_initialized_ = false;

    int last_battery_level_ = 0;
    int last_voltage_mv_ = 0;
    bool last_charging_state_ = false;
    bool stt_only_active_ = false;

    bool ReadTouchRegister(uint8_t reg, uint8_t *data, size_t len) {
        if (!touch_initialized_ && reg != FT6336_FIRMWARE_ID) return false;
        esp_err_t ret = i2c_master_transmit_receive(touch_dev_handle_, &reg, 1, data, len, -1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C read failed at reg 0x%02X: %s", reg, esp_err_to_name(ret));
        }
        return ret == ESP_OK;
    }

    bool WriteTouchRegister(uint8_t reg, uint8_t value) {
        uint8_t write_buf[2] = {reg, value};
        esp_err_t ret = i2c_master_transmit(touch_dev_handle_, write_buf, 2, -1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C write failed at reg 0x%02X: %s", reg, esp_err_to_name(ret));
        }
        return ret == ESP_OK;
    }

    // --- CÔNG THỨC MAPPING TỌA ĐỘ MỚI DỰA TRÊN DỮ LIỆU TEST ---
    bool ReadTouchPoint(uint16_t &x, uint16_t &y) {
        uint8_t touch_count = 0;
        if (!ReadTouchRegister(FT6336_TD_STATUS, &touch_count, 1)) return false;
        touch_count &= 0x0F;
        if (touch_count == 0 || touch_count > 2) return false;

        uint8_t data[4];
        if (!ReadTouchRegister(FT6336_TOUCH1_XH, data, 4)) return false;

        uint16_t raw_x = ((data[0] & 0x0F) << 8) | data[1];
        uint16_t raw_y = ((data[2] & 0x0F) << 8) | data[3];

        // Biến đổi dựa trên 5 điểm test:
        // Top-Left [252, 19] -> Cần [0, 0]
        // Top-Right [20, 28] -> Cần [320, 0]
        // Bottom-Right [24, 187] -> Cần [320, 240]
        x = raw_x;
        y = raw_y;

        // Giới hạn an toàn
        if (x >= 320) x = 319;
        if (y >= 240) y = 239;

        ESP_LOGI(TAG, "Touch Physical: [%d,%d]", x, y);
        return true;
    }

    void InitializeTouch() {
        ESP_LOGI(TAG, "Initializing FT6336 touch controller...");
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = FT6336_ADDR,
            .scl_speed_hz = 400000,
        };

        if (i2c_master_bus_add_device(codec_i2c_bus_, &dev_cfg, &touch_dev_handle_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add touch device");
            return;
        }

        uint8_t chip_id;
        if (ReadTouchRegister(FT6336_FIRMWARE_ID, &chip_id, 1)) {
            ESP_LOGI(TAG, "Touch controller found. Chip ID: 0x%02X", chip_id);
            touch_initialized_ = true;
        }

        WriteTouchRegister(FT6336_CTRL, 0x00);
        WriteTouchRegister(FT6336_THRESHOLD, 0x14);
    }

    static void LvglTouchCb(lv_indev_t *indev, lv_indev_data_t *data) {
        FreenoveESP32S3Display *self = (FreenoveESP32S3Display *) lv_indev_get_user_data(indev);
        uint16_t x, y;
        if (self && self->ReadTouchPoint(x, y)) {
            data->point.x = (lv_coord_t) x;
            data->point.y = (lv_coord_t) y;
            data->state = LV_INDEV_STATE_PRESSED;
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }
    }

    static void OnDisplayClicked(lv_event_t *e) {
        lv_event_code_t code = lv_event_get_code(e);
        // Nhận diện sự kiện click để bật/tắt nghe
        if(code == LV_EVENT_CLICKED) {
            ESP_LOGI(TAG, ">>>> DISPLAY CLICKED - TOGGLE CHAT <<<<");
            Application::GetInstance().ToggleChatState();
        }
    }

    void InitializeSpi() {
        ESP_LOGI(TAG, "Initializing SPI bus for LCD...");
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = DISPLAY_MIS0_PIN;
        buscfg.sclk_io_num = DISPLAY_SCK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        ESP_LOGI(TAG, "Creating ILI9341 LCD panel...");
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new SpiLcdDisplay(panel_io, panel,
                                     DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                     DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                     {
                                         .text_font = &font_viet_20,
                                         .icon_font = &font_awesome_16_4,
                                         .emoji_font = DISPLAY_HEIGHT >= 240
                                                           ? font_emoji_64_init()
                                                           : font_emoji_32_init(),
                                     });
    }

    void InitializeI2c() {
        ESP_LOGI(TAG, "Initializing I2C master bus...");
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = AUDIO_CODEC_I2C_NUM,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = 1},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto &app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting &&
                !WifiStation::GetInstance().IsConnected()) {
                ESP_LOGI(TAG, "Starting state and no WiFi: Resetting configuration");
                ResetWifiConfiguration();
                return;
            }
            ESP_LOGI(TAG, "Boot button Click: Toggle chat state");
            app.ToggleChatState();
        });

        boot_button_.OnLongPress([this]() {
            auto &app = Application::GetInstance();
            stt_only_active_ = !stt_only_active_;
            ESP_LOGI(TAG, "Boot button LongPress: STT Only Mode %s", stt_only_active_ ? "ON" : "OFF");
            app.SetSttOnlyMode(stt_only_active_);
        });

        boot_button_.OnDoubleClick([this]() {
            auto &app = Application::GetInstance();
            ESP_LOGI(TAG, "Boot button DoubleClick: Toggle Audio Testing");

            if (app.GetDeviceState() == kDeviceStateAudioTesting) {
                app.ExitAudioTestingMode();
            } else {
                app.EnterAudioTestingMode();
            }
        });
    }

public:
    FreenoveESP32S3Display() : boot_button_(BOOT_BUTTON_GPIO) {
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

            // TĂNG VÙNG NHẬN SỰ KIỆN: Gán trực tiếp vào màn hình (screen)
            // Thay vì chỉ vùng Content, bấm bất cứ đâu trên màn hình cũng sẽ toggle chat
            lv_obj_t* scr = lv_screen_active();
            lv_obj_add_flag(scr, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(scr, OnDisplayClicked, LV_EVENT_CLICKED, nullptr);

            lvgl_port_unlock();
        }

        GetBacklight()->SetBrightness(100);
    }

    ~FreenoveESP32S3Display() {
        if (touch_dev_handle_ != nullptr) i2c_master_bus_rm_device(touch_dev_handle_);
    }

    virtual Led *GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec *GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(codec_i2c_bus_, AUDIO_CODEC_I2C_NUM,
                                            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_GPIO_MCLK,
                                            AUDIO_I2S_GPIO_BCLK,
                                            AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
                                            AUDIO_CODEC_PA_PIN,
                                            AUDIO_CODEC_ES8311_ADDR, true, true);
        return &audio_codec;
    }

    virtual Display *GetDisplay() override { return display_; }

    virtual Backlight *GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN,
                                      DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual bool GetBatteryLevel(int &level, bool &charging, bool &discharging) override {
        static uint32_t last_check = 0;
        uint32_t now = esp_log_timestamp();

        if (last_check > 0 && (now - last_check) < 10000) {
            level = last_battery_level_;
            charging = last_charging_state_;
            discharging = !last_charging_state_;
            return true;
        }

        adc_oneshot_unit_handle_t handle = nullptr;
        adc_oneshot_unit_init_cfg_t init_config = {.unit_id = ADC_UNIT_1};

        if (adc_oneshot_new_unit(&init_config, &handle) == ESP_OK) {
            adc_oneshot_chan_cfg_t chan_config = {
                .atten = ADC_ATTEN_DB_12,
                .bitwidth = ADC_BITWIDTH_DEFAULT
            };
            adc_oneshot_config_channel(handle, ADC_CHANNEL_8, &chan_config);

            adc_cali_handle_t cali_handle = NULL;
            adc_cali_curve_fitting_config_t cali_config = {
                .unit_id = ADC_UNIT_1,
                .atten = ADC_ATTEN_DB_12,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            bool cali_enabled = (adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle) == ESP_OK);

            int adc_raw = 0;
            if (adc_oneshot_read(handle, ADC_CHANNEL_8, &adc_raw) == ESP_OK) {
                int voltage_mv = 0;
                if (cali_enabled) {
                    adc_cali_raw_to_voltage(cali_handle, adc_raw, &voltage_mv);
                } else {
                    voltage_mv = (adc_raw * 3300) / 4095;
                }

                int real_voltage_mv = voltage_mv * 2;

                if (real_voltage_mv > 4185) {
                    last_charging_state_ = true;
                } else if (last_voltage_mv_ > 0 && (real_voltage_mv - last_voltage_mv_) > 25) {
                    last_charging_state_ = true;
                } else if (last_voltage_mv_ > 0 && (last_voltage_mv_ - real_voltage_mv) > 10) {
                    last_charging_state_ = false;
                }

                if (real_voltage_mv >= 4150) level = 100;
                else if (real_voltage_mv <= 3500) level = 0;
                else level = (real_voltage_mv - 3500) / 6.5;

                last_battery_level_ = level;
                last_voltage_mv_ = real_voltage_mv;
                ESP_LOGI(TAG, "Battery level updated: %d%% (%dmV)", level, real_voltage_mv);
            }

            if (cali_enabled) adc_cali_delete_scheme_curve_fitting(cali_handle);
            adc_oneshot_del_unit(handle);
            last_check = now;
        }

        level = last_battery_level_;
        charging = last_charging_state_;
        discharging = !last_charging_state_;
        return true;
    }
};

DECLARE_BOARD(FreenoveESP32S3Display);
