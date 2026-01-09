// path: main/boards/freenove-esp32s3-display-2.8-lcd/freenove_esp32s3_display_board.cc

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

#include "esp_lcd_ili9341.h"
#include <hal/adc_types.h>

#define TAG "FreenoveESP32S3Display"

LV_FONT_DECLARE(font_viet_16);
LV_FONT_DECLARE(font_awesome_16_4);

class FreenoveESP32S3Display : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay *display_;
    i2c_master_bus_handle_t codec_i2c_bus_;

    int last_battery_level_ = 0;
    int last_voltage_mv_ = 0;
    bool last_charging_state_ = false;

    void InitializeSpi() {
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
                                         .text_font = &font_viet_16,
                                         .icon_font = &font_awesome_16_4,
                                         .emoji_font = DISPLAY_HEIGHT >= 240
                                                           ? font_emoji_64_init()
                                                           : font_emoji_32_init(),
                                     });
    }

    void InitializeI2c() {
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
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

public:
    FreenoveESP32S3Display() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        GetBacklight()->SetBrightness(100);
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

                // LOGIC CẢI TIẾN:
                // 1. Nếu cắm USB, điện áp tại pin sẽ rất ổn định hoặc tăng nhẹ.
                // 2. Dùng một ngưỡng Volt cao hơn (vd: 4185mV) để làm căn cứ charging khi USB cắm vào.
                // 3. Sử dụng mức delta lớn hơn để tránh báo sạc nhầm khi rút điện.

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

                ESP_LOGI(TAG, "Battery: %d%% (%dmV), Charging: %s",
                         level, real_voltage_mv, last_charging_state_ ? "YES" : "NO");
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
