// path: src/application.cpp

#include "application.h"
#include "board.h"
#include "display.h"
#include "system_info.h"
#include "ml307_ssl_transport.h"
#include "audio_codec.h"
#include "mqtt_protocol.h"
#include "websocket_protocol.h"
#include "font_awesome_symbols.h"
#include "iot/thing_manager.h"
#include "assets/lang_config.h"
#include "mcp_server.h"
#include "audio_debugger.h"
#include "freertos/task.h" // Thêm thư viện này để dùng vTaskDelay
#include <esp_sleep.h>     // Thêm thư viện Deep Sleep

#if CONFIG_USE_AUDIO_PROCESSOR
#include "afe_audio_processor.h"
#else
#include "no_audio_processor.h"
#endif

#if CONFIG_USE_AFE_WAKE_WORD
#include "afe_wake_word.h"
#elif CONFIG_USE_ESP_WAKE_WORD
#include "esp_wake_word.h"
#else
#include "no_wake_word.h"
#endif

#include <cstring>
#include <esp_log.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <arpa/inet.h>

#define TAG "Application"

// --- CẤU HÌNH THỜI GIAN TIẾT KIỆM PIN ---
// Sau 2 phút: Tắt màn hình (Vẫn nhận diện giọng nói - Voice Wakeup OK)
#define BATTERY_SAVE_DIM_TIMEOUT 60
// Sau 5 phút: Ngủ sâu (Chỉ nhận diện nút bấm - Button Wakeup Only)
#define BATTERY_SAVE_SLEEP_TIMEOUT 300

static const char *const STATE_STRINGS[] = {
    "unknown",
    "starting",
    "configuring",
    "idle",
    "connecting",
    "listening",
    "speaking",
    "upgrading",
    "activating",
    "audio_testing",
    "fatal_error",
    "invalid_state"
};

Application::Application() {
    event_group_ = xEventGroupCreate();
    background_task_ = new BackgroundTask(4096 * 7);

    // Mặc định tắt chế độ tiết kiệm pin lúc khởi động để tránh ngủ ngay lập tức
    battery_save_mode_ = false;

#if CONFIG_USE_DEVICE_AEC
    aec_mode_ = kAecOnDeviceSide;
#elif CONFIG_USE_SERVER_AEC
    aec_mode_ = kAecOnServerSide;
#else
    aec_mode_ = kAecOff;
#endif

#if CONFIG_USE_AUDIO_PROCESSOR
    audio_processor_ = std::make_unique<AfeAudioProcessor>();
#else
    audio_processor_ = std::make_unique<NoAudioProcessor>();
#endif

#if CONFIG_USE_AFE_WAKE_WORD
    wake_word_ = std::make_unique<AfeWakeWord>();
#elif CONFIG_USE_ESP_WAKE_WORD
    wake_word_ = std::make_unique<EspWakeWord>();
#else
    wake_word_ = std::make_unique<NoWakeWord>();
#endif

    esp_timer_create_args_t clock_timer_args = {
        .callback = [](void *arg) {
            Application *app = (Application *) arg;
            app->OnClockTimer();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "clock_timer",
        .skip_unhandled_events = true
    };
    esp_timer_create(&clock_timer_args, &clock_timer_handle_);
}

Application::~Application() {
    if (clock_timer_handle_ != nullptr) {
        esp_timer_stop(clock_timer_handle_);
        esp_timer_delete(clock_timer_handle_);
    }
    if (background_task_ != nullptr) {
        delete background_task_;
    }
    vEventGroupDelete(event_group_);
}

void Application::CheckNewVersion(Ota &ota) {
    const int MAX_RETRY = 10;
    int retry_count = 0;
    int retry_delay = 10;

    while (true) {
        SetDeviceState(kDeviceStateActivating);
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::CHECKING_NEW_VERSION);

        if (!ota.CheckVersion()) {
            retry_count++;
            if (retry_count >= MAX_RETRY) {
                ESP_LOGE(TAG, "Too many retries, exit version check");
                return;
            }

            char buffer[128];
            snprintf(buffer, sizeof(buffer), Lang::Strings::CHECK_NEW_VERSION_FAILED, retry_delay,
                     ota.GetCheckVersionUrl().c_str());
            Alert(Lang::Strings::ERROR, buffer, "sad", Lang::Sounds::P3_EXCLAMATION);

            ESP_LOGW(TAG, "Check new version failed, retry in %d seconds (%d/%d)", retry_delay, retry_count, MAX_RETRY);
            for (int i = 0; i < retry_delay; i++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (device_state_ == kDeviceStateIdle) {
                    break;
                }
            }
            retry_delay *= 2;
            continue;
        }
        retry_count = 0;
        retry_delay = 10;

        if (ota.HasNewVersion()) {
            Alert(Lang::Strings::OTA_UPGRADE, Lang::Strings::UPGRADING, "happy", Lang::Sounds::P3_UPGRADE);

            vTaskDelay(pdMS_TO_TICKS(3000));

            SetDeviceState(kDeviceStateUpgrading);

            display->SetIcon(FONT_AWESOME_DOWNLOAD);
            std::string message = std::string(Lang::Strings::NEW_VERSION) + ota.GetFirmwareVersion();
            display->SetChatMessage("system", message.c_str());

            auto &board = Board::GetInstance();
            board.SetPowerSaveMode(false);
            wake_word_->StopDetection();
            // Pre-close audio output to avoid audio operations during upgrade
            auto codec = board.GetAudioCodec();
            codec->EnableInput(false);
            codec->EnableOutput(false);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                audio_decode_queue_.clear();
            }
            background_task_->WaitForCompletion();
            delete background_task_;
            background_task_ = nullptr;
            vTaskDelay(pdMS_TO_TICKS(1000));

            ota.StartUpgrade([display](int progress, size_t speed) {
                char buffer[64];
                snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
                display->SetChatMessage("system", buffer);
            });

            // If upgrade success, the device will reboot and never reach here
            display->SetStatus(Lang::Strings::UPGRADE_FAILED);
            ESP_LOGI(TAG, "Firmware upgrade failed...");
            vTaskDelay(pdMS_TO_TICKS(3000));
            Reboot();
            return;
        }

        // No new version, mark the current version as valid
        ota.MarkCurrentVersionValid();
        if (!ota.HasActivationCode() && !ota.HasActivationChallenge()) {
            xEventGroupSetBits(event_group_, CHECK_NEW_VERSION_DONE_EVENT);
            // Exit the loop if done checking new version
            break;
        }

        display->SetStatus(Lang::Strings::ACTIVATION);
        // Activation code is shown to the user and waiting for the user to input
        if (ota.HasActivationCode()) {
            ShowActivationCode(ota.GetActivationCode(), ota.GetActivationMessage());
        }

        // This will block the loop until the activation is done or timeout
        for (int i = 0; i < 10; ++i) {
            ESP_LOGI(TAG, "Activating... %d/%d", i + 1, 10);
            esp_err_t err = ota.Activate();
            if (err == ESP_OK) {
                xEventGroupSetBits(event_group_, CHECK_NEW_VERSION_DONE_EVENT);
                break;
            } else if (err == ESP_ERR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(3000));
            } else {
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
            if (device_state_ == kDeviceStateIdle) {
                break;
            }
        }
    }
}

void Application::ShowActivationCode(const std::string &code, const std::string &message) {
    struct digit_sound {
        char digit;
        const std::string_view &sound;
    };
    static const std::array<digit_sound, 10> digit_sounds{
        {
            digit_sound{'0', Lang::Sounds::P3_0},
            digit_sound{'1', Lang::Sounds::P3_1},
            digit_sound{'2', Lang::Sounds::P3_2},
            digit_sound{'3', Lang::Sounds::P3_3},
            digit_sound{'4', Lang::Sounds::P3_4},
            digit_sound{'5', Lang::Sounds::P3_5},
            digit_sound{'6', Lang::Sounds::P3_6},
            digit_sound{'7', Lang::Sounds::P3_7},
            digit_sound{'8', Lang::Sounds::P3_8},
            digit_sound{'9', Lang::Sounds::P3_9}
        }
    };

    // This sentence uses 9KB of SRAM, so we need to wait for it to finish
    Alert(Lang::Strings::ACTIVATION, message.c_str(), "happy", Lang::Sounds::P3_ACTIVATION);

    for (const auto &digit: code) {
        auto it = std::find_if(digit_sounds.begin(), digit_sounds.end(),
                               [digit](const digit_sound &ds) { return ds.digit == digit; });
        if (it != digit_sounds.end()) {
            PlaySound(it->sound);
        }
    }
}

void Application::Alert(const char *status, const char *message, const char *emotion, const std::string_view &sound) {
    ESP_LOGW(TAG, "Alert %s: %s [%s]", status, message, emotion);
    auto display = Board::GetInstance().GetDisplay();
    display->SetStatus(status);
    display->SetEmotion(emotion);
    display->SetChatMessage("system", message);
    if (!sound.empty()) {
        ResetDecoder();
        PlaySound(sound);
    }
}

void Application::DismissAlert() {
    if (device_state_ == kDeviceStateIdle) {
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::STANDBY);
        display->SetEmotion("neutral");
        display->SetChatMessage("system", "");
    }
}

void Application::PlaySound(const std::string_view &sound) {
    // Wait for the previous sound to finish
    {
        std::unique_lock<std::mutex> lock(mutex_);
        audio_decode_cv_.wait(lock, [this]() {
            return audio_decode_queue_.empty();
        });
    }
    background_task_->WaitForCompletion();

    const char *data = sound.data();
    size_t size = sound.size();
    for (const char *p = data; p < data + size;) {
        auto p3 = (BinaryProtocol3 *) p;
        p += sizeof(BinaryProtocol3);

        auto payload_size = ntohs(p3->payload_size);
        AudioStreamPacket packet;
        packet.sample_rate = 16000;
        packet.frame_duration = 60;
        packet.payload.resize(payload_size);
        memcpy(packet.payload.data(), p3->payload, payload_size);
        p += payload_size;

        std::lock_guard<std::mutex> lock(mutex_);
        audio_decode_queue_.emplace_back(std::move(packet));
    }
}

void Application::EnterAudioTestingMode() {
    ESP_LOGI(TAG, "Entering audio testing mode");
    ResetDecoder();
    SetDeviceState(kDeviceStateAudioTesting);
}

void Application::ExitAudioTestingMode() {
    ESP_LOGI(TAG, "Exiting audio testing mode");
    SetDeviceState(kDeviceStateWifiConfiguring);
    // Copy audio_testing_queue_ to audio_decode_queue_
    std::lock_guard<std::mutex> lock(mutex_);
    audio_decode_queue_ = std::move(audio_testing_queue_);
    audio_decode_cv_.notify_all();
}

void Application::ToggleChatState() {
    if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (device_state_ == kDeviceStateWifiConfiguring) {
        EnterAudioTestingMode();
        return;
    } else if (device_state_ == kDeviceStateAudioTesting) {
        ExitAudioTestingMode();
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    // -------------------------------------------------------------
    // LOGIC CHO STT ONLY MODE
    // -------------------------------------------------------------
    if (stt_only_mode_) {
        // Nếu đang nghe hoặc nói -> Người dùng muốn TẮT (Nghỉ)
        if (device_state_ == kDeviceStateListening || device_state_ == kDeviceStateSpeaking || device_state_ ==
            kDeviceStateConnecting) {
            ESP_LOGI(TAG, "STT Mode: User toggled to PAUSE (Idle)");
            // Đặt cờ ManualStop để Watchdog không tự bật lại
            listening_mode_ = kListeningModeManualStop;
            Schedule([this]() {
                if (protocol_->IsAudioChannelOpened()) {
                    protocol_->CloseAudioChannel();
                } else {
                    SetDeviceState(kDeviceStateIdle);
                }
            });
        }
        // Nếu đang nghỉ -> Người dùng muốn BẬT (Nghe lại)
        else {
            ESP_LOGI(TAG, "STT Mode: User toggled to RESUME (Listening)");
            // Đặt cờ Realtime để Watchdog có thể hỗ trợ nếu rớt mạng
            listening_mode_ = kListeningModeRealtime;
            Schedule([this]() {
                // QUAN TRỌNG: Mở kênh TRƯỚC khi gửi WakeWord
                if (!protocol_->IsAudioChannelOpened()) {
                    SetDeviceState(kDeviceStateConnecting);
                    if (!protocol_->OpenAudioChannel()) {
                         SetDeviceState(kDeviceStateIdle);
                        return;
                    }
                }

                SetDeviceState(kDeviceStateListening);
            });
        }
        return;
    }

    // -------------------------------------------------------------
    // LOGIC CHO CHẾ ĐỘ THƯỜNG (NORMAL MODE)
    // -------------------------------------------------------------
    if (device_state_ == kDeviceStateIdle) {
        Schedule([this]() {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    return;
                }
            }

            SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
        });
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {
        Schedule([this]() {
            protocol_->CloseAudioChannel();
        });
    }
}

void Application::StartListening() {
    if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (device_state_ == kDeviceStateWifiConfiguring) {
        EnterAudioTestingMode();
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (device_state_ == kDeviceStateIdle) {
        Schedule([this]() {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    return;
                }
            }

            SetListeningMode(kListeningModeManualStop);
        });
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
            SetListeningMode(kListeningModeManualStop);
        });
    }
}

void Application::StopListening() {
    if (device_state_ == kDeviceStateAudioTesting) {
        ExitAudioTestingMode();
        return;
    }

    const std::array<int, 3> valid_states = {
        kDeviceStateListening,
        kDeviceStateSpeaking,
        kDeviceStateIdle,
    };
    // If not valid, do nothing
    if (std::find(valid_states.begin(), valid_states.end(), device_state_) == valid_states.end()) {
        return;
    }

    Schedule([this]() {
        if (device_state_ == kDeviceStateListening) {
            protocol_->SendStopListening();
            SetDeviceState(kDeviceStateIdle);
        }
    });
}

// -------------------------------------------------------------------------
// SEPARATE MESSAGE HANDLERS
// -------------------------------------------------------------------------

void Application::HandleSttMode(const cJSON *root) {
    auto display = Board::GetInstance().GetDisplay();
    auto type = cJSON_GetObjectItem(root, "type");

    // 1. Handle STT
    if (strcmp(type->valuestring, "stt") == 0) {
        auto text = cJSON_GetObjectItem(root, "text");
        if (cJSON_IsString(text)) {
            ESP_LOGI(TAG, "STT Mode >> %s", text->valuestring);
            Schedule([this, display, message = std::string(text->valuestring)]() {
                display->SetChatMessage("user", message.c_str());
            });
        }
    }
    // 2. Handle TTS
    else if (strcmp(type->valuestring, "tts") == 0) {
        auto state = cJSON_GetObjectItem(root, "state");
        if (state && strcmp(state->valuestring, "stop") == 0) {
             // Logic dừng nếu cần
        }
    }
    // 3. Handle System Commands
    else if (strcmp(type->valuestring, "system") == 0) {
        auto command = cJSON_GetObjectItem(root, "command");
        if (cJSON_IsString(command) && strcmp(command->valuestring, "reboot") == 0) {
            Schedule([this]() { Reboot(); });
        }
    }

    // --- BỔ SUNG QUAN TRỌNG: XỬ LÝ MCP TRONG CHẾ ĐỘ STT ---
#if CONFIG_IOT_PROTOCOL_MCP
    else if (strcmp(type->valuestring, "mcp") == 0) {
        auto payload = cJSON_GetObjectItem(root, "payload");
        if (cJSON_IsObject(payload)) {
            ESP_LOGI(TAG, "STT Mode received MCP Command");
            McpServer::GetInstance().ParseMessage(payload);
        }
    }
#endif
    // -----------------------------------------------------
}

void Application::HandleNormalMode(const cJSON *root) {
    auto display = Board::GetInstance().GetDisplay();
    auto type = cJSON_GetObjectItem(root, "type");

    if (strcmp(type->valuestring, "tts") == 0) {
        auto state = cJSON_GetObjectItem(root, "state");
        if (strcmp(state->valuestring, "start") == 0) {
            Schedule([this]() {
                aborted_ = false;
                if (device_state_ == kDeviceStateIdle || device_state_ == kDeviceStateListening) {
                    SetDeviceState(kDeviceStateSpeaking);
                }
            });
        } else if (strcmp(state->valuestring, "stop") == 0) {
            Schedule([this]() {
                background_task_->WaitForCompletion();
                if (device_state_ == kDeviceStateSpeaking) {
                    if (listening_mode_ == kListeningModeManualStop) {
                        SetDeviceState(kDeviceStateIdle);
                    } else {
                        SetDeviceState(kDeviceStateListening);
                    }
                }
            });
        } else if (strcmp(state->valuestring, "sentence_start") == 0) {
            auto text = cJSON_GetObjectItem(root, "text");
            if (cJSON_IsString(text)) {
                ESP_LOGI(TAG, "<< %s", text->valuestring);
                Schedule([this, display, message = std::string(text->valuestring)]() {
                    display->SetChatMessage("assistant", message.c_str());
                });
            }
        }
    } else if (strcmp(type->valuestring, "stt") == 0) {
        auto text = cJSON_GetObjectItem(root, "text");
        if (cJSON_IsString(text)) {
            ESP_LOGI(TAG, ">> %s", text->valuestring);
            Schedule([this, display, message = std::string(text->valuestring)]() {
                display->SetChatMessage("user", message.c_str());
            });
        }
    } else if (strcmp(type->valuestring, "llm") == 0) {
        auto emotion = cJSON_GetObjectItem(root, "emotion");
        if (cJSON_IsString(emotion)) {
            Schedule([this, display, emotion_str = std::string(emotion->valuestring)]() {
                display->SetEmotion(emotion_str.c_str());
            });
        }
#if CONFIG_IOT_PROTOCOL_MCP
    } else if (strcmp(type->valuestring, "mcp") == 0) {
        auto payload = cJSON_GetObjectItem(root, "payload");
        if (cJSON_IsObject(payload)) {
            McpServer::GetInstance().ParseMessage(payload);
        }
#endif
#if CONFIG_IOT_PROTOCOL_XIAOZHI
    } else if (strcmp(type->valuestring, "iot") == 0) {
            auto commands = cJSON_GetObjectItem(root, "commands");
            if (cJSON_IsArray(commands)) {
                auto &thing_manager = iot::ThingManager::GetInstance();
                for (int i = 0; i < cJSON_GetArraySize(commands); ++i) {
                    auto command = cJSON_GetArrayItem(commands, i);
                    thing_manager.Invoke(command);
                }
            }
#endif
    } else if (strcmp(type->valuestring, "system") == 0) {
        auto command = cJSON_GetObjectItem(root, "command");
        if (cJSON_IsString(command)) {
            if (strcmp(command->valuestring, "reboot") == 0) {
                Schedule([this]() { Reboot(); });
            }
        }
    } else if (strcmp(type->valuestring, "alert") == 0) {
        auto status = cJSON_GetObjectItem(root, "status");
        auto message = cJSON_GetObjectItem(root, "message");
        auto emotion = cJSON_GetObjectItem(root, "emotion");
        if (cJSON_IsString(status) && cJSON_IsString(message) && cJSON_IsString(emotion)) {
            Alert(status->valuestring, message->valuestring, emotion->valuestring, Lang::Sounds::P3_VIBRATION);
        }
    } else if (strcmp(type->valuestring, "battery_save") == 0) {
        bool enable = cJSON_IsTrue(cJSON_GetObjectItem(root, "enable"));
        SetBatterySaverMode(enable);
    }
}


void Application::Start() {
    auto &board = Board::GetInstance();
    SetDeviceState(kDeviceStateStarting);

    /* Setup the display */
    auto display = board.GetDisplay();

    /* Setup the audio codec */
    auto codec = board.GetAudioCodec();
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(codec->output_sample_rate(), 1, OPUS_FRAME_DURATION_MS);
    opus_encoder_ = std::make_unique<OpusEncoderWrapper>(16000, 1, OPUS_FRAME_DURATION_MS);
    if (aec_mode_ != kAecOff) {
        ESP_LOGI(TAG, "AEC mode: %d, setting opus encoder complexity to 0", aec_mode_);
        opus_encoder_->SetComplexity(0);
    } else if (board.GetBoardType() == "ml307") {
        ESP_LOGI(TAG, "ML307 board detected, setting opus encoder complexity to 5");
        opus_encoder_->SetComplexity(5);
    } else {
        ESP_LOGI(TAG, "WiFi board detected, setting opus encoder complexity to 0");
        opus_encoder_->SetComplexity(0);
    }

    if (codec->input_sample_rate() != 16000) {
        input_resampler_.Configure(codec->input_sample_rate(), 16000);
        reference_resampler_.Configure(codec->input_sample_rate(), 16000);
    }
    codec->Start();

#if CONFIG_USE_AUDIO_PROCESSOR
    xTaskCreatePinnedToCore([](void *arg) {
        Application *app = (Application *) arg;
        app->AudioLoop();
        vTaskDelete(NULL);
    }, "audio_loop", 4096 * 2, this, 8, &audio_loop_task_handle_, 1);
#else
    xTaskCreate([](void *arg) {
        Application *app = (Application *) arg;
        app->AudioLoop();
        vTaskDelete(NULL);
    }, "audio_loop", 4096 * 2, this, 8, &audio_loop_task_handle_);
#endif

    /* Start the clock timer to update the status bar */
    esp_timer_start_periodic(clock_timer_handle_, 1000000);

    /* Wait for the network to be ready */
    board.StartNetwork();

    // Update the status bar immediately to show the network state
    display->UpdateStatusBar(true);

    // Check for new firmware version or get the MQTT broker address
    Ota ota;
    // CheckNewVersion(ota); // Disabled for faster boot

    // Initialize the protocol
    display->SetStatus(Lang::Strings::LOADING_PROTOCOL);

    // Add MCP common tools before initializing the protocol
#if CONFIG_IOT_PROTOCOL_MCP
    McpServer::GetInstance().AddCommonTools();
#endif

    if (ota.HasMqttConfig()) {
        protocol_ = std::make_unique<MqttProtocol>();
    } else if (ota.HasWebsocketConfig()) {
        protocol_ = std::make_unique<WebsocketProtocol>();
    } else {
        ESP_LOGW(TAG, "No protocol specified in the OTA config, using MQTT");
        protocol_ = std::make_unique<MqttProtocol>();
    }

    protocol_->OnNetworkError([this](const std::string &message) {
        SetDeviceState(kDeviceStateIdle);
        Alert(Lang::Strings::ERROR, message.c_str(), "sad", Lang::Sounds::P3_EXCLAMATION);
    });
    protocol_->OnIncomingAudio([this](AudioStreamPacket &&packet) {
        // Only add to queue if not in STT only mode
        if (stt_only_mode_) {
            ESP_LOGD(TAG, "Incoming Audio ignored in STT Only mode");
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (device_state_ == kDeviceStateSpeaking && audio_decode_queue_.size() < MAX_AUDIO_PACKETS_IN_QUEUE) {
            audio_decode_queue_.emplace_back(std::move(packet));
        }
    });
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        // Wakeup screen on new session
        board.SetPowerSaveMode(false);
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(
                TAG,
                "Server sample rate %d does not match device output sample rate %d, resampling may cause distortion",
                protocol_->server_sample_rate(), codec->output_sample_rate());
        }

#if CONFIG_IOT_PROTOCOL_XIAOZHI
        auto &thing_manager = iot::ThingManager::GetInstance();
        protocol_->SendIotDescriptors(thing_manager.GetDescriptorsJson());
        std::string states;
        if (thing_manager.GetStatesJson(states, false)) {
            protocol_->SendIotStates(states);
        }
#endif
    });

    // -------------------------------------------------------------
    // FIX: LOGIC KHI KẾT THÚC SESSION (GOODBYE)
    // -------------------------------------------------------------
    protocol_->OnAudioChannelClosed([this, &board]() {
        // NOTE: We do NOT force power save here immediately.
        // We let the idle timer handle the dimming to avoid screen flickering off too fast.
        // board.SetPowerSaveMode(true);

        Schedule([this]() {
            auto display = Board::GetInstance().GetDisplay();

            if (stt_only_mode_) {
                // Khi Server gửi Goodbye (Session Closed):
                ESP_LOGI(TAG, "STT Mode: Server sent Goodbye. Stopping.");
                SetDeviceState(kDeviceStateIdle);
                listening_mode_ = kListeningModeManualStop;
            } else {
                SetDeviceState(kDeviceStateIdle);
                display->SetChatMessage("system", "");
            }
        });
    });

    // Main Message Dispatcher
    protocol_->OnIncomingJson([this](const cJSON *root) {
        if (stt_only_mode_) {
            HandleSttMode(root);
        } else {
            HandleNormalMode(root);
        }
    });

    bool protocol_started = protocol_->Start();

    audio_debugger_ = std::make_unique<AudioDebugger>();
    audio_processor_->Initialize(codec);
    audio_processor_->OnOutput([this](std::vector<int16_t> &&data) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (audio_send_queue_.size() >= MAX_AUDIO_PACKETS_IN_QUEUE) {
                ESP_LOGW(TAG, "Too many audio packets in queue, drop the newest packet");
                return;
            }
        }
        background_task_->Schedule([this, data = std::move(data)]() mutable {
            opus_encoder_->Encode(std::move(data), [this](std::vector<uint8_t> &&opus) {
                AudioStreamPacket packet;
                packet.payload = std::move(opus);
#ifdef CONFIG_USE_SERVER_AEC
                {
                    std::lock_guard<std::mutex> lock(timestamp_mutex_);
                    if (!timestamp_queue_.empty()) {
                        packet.timestamp = timestamp_queue_.front();
                        timestamp_queue_.pop_front();
                    } else {
                        packet.timestamp = 0;
                    }

                    if (timestamp_queue_.size() > 3) {
                        // Limit queue size to 3
                        timestamp_queue_.pop_front();
                        return;
                    }
                }
#endif
                std::lock_guard<std::mutex> lock(mutex_);
                if (audio_send_queue_.size() >= MAX_AUDIO_PACKETS_IN_QUEUE) {
                    ESP_LOGW(TAG, "Too many audio packets in queue, drop the oldest packet");
                    audio_send_queue_.pop_front();
                }
                audio_send_queue_.emplace_back(std::move(packet));
                xEventGroupSetBits(event_group_, SEND_AUDIO_EVENT);
            });
        });
    });
    audio_processor_->OnVadStateChange([this](bool speaking) {
        if (device_state_ == kDeviceStateListening) {
            Schedule([this, speaking]() {
                if (speaking) {
                    voice_detected_ = true;
                } else {
                    voice_detected_ = false;
                }
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            });
        }
    });

    wake_word_->Initialize(codec);
    wake_word_->OnWakeWordDetected([this](const std::string &wake_word) {
        Schedule([this, &wake_word]() {
            if (!protocol_) {
                return;
            }

            if (device_state_ == kDeviceStateIdle) {
                wake_word_->EncodeWakeWordData();

                if (!protocol_->IsAudioChannelOpened()) {
                    SetDeviceState(kDeviceStateConnecting);
                    if (!protocol_->OpenAudioChannel()) {
                        wake_word_->StartDetection();
                        return;
                    }
                }

                ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
#if CONFIG_USE_AFE_WAKE_WORD
                AudioStreamPacket packet;
                // Encode and send the wake word data to the server
                while (wake_word_->GetWakeWordOpus(packet.payload)) {
                    protocol_->SendAudio(packet);
                }
                // Set the chat state to wake word detected
              //  protocol_->SendWakeWordDetected(wake_word);
#else
                // Play the pop up sound to indicate the wake word is detected
                // And wait 60ms to make sure the queue has been processed by audio task
                ResetDecoder();
                PlaySound(Lang::Sounds::P3_POPUP);
                vTaskDelay(pdMS_TO_TICKS(60));
#endif
                SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
            } else if (device_state_ == kDeviceStateSpeaking) {
                AbortSpeaking(kAbortReasonWakeWordDetected);
            } else if (device_state_ == kDeviceStateActivating) {
                SetDeviceState(kDeviceStateIdle);
            }
        });
    });
    wake_word_->StartDetection();

    // Wait for the new version check to finish
    // xEventGroupWaitBits(event_group_, CHECK_NEW_VERSION_DONE_EVENT, pdTRUE, pdFALSE, portMAX_DELAY); // Disabled since check is skipped
    SetDeviceState(kDeviceStateIdle);

    has_server_time_ = ota.HasServerTime();
    if (protocol_started) {
       // std::string message = std::string(Lang::Strings::VERSION) + ota.GetCurrentVersion();
       // display->ShowNotification(message.c_str());
        display->SetChatMessage("system", "");
        // Play the success sound to indicate the device is ready
        ResetDecoder();
        PlaySound(Lang::Sounds::P3_SUCCESS);
    }

    // Print heap stats
    SystemInfo::PrintHeapStats();

    // Enter the main event loop
    MainEventLoop();
}

void Application::OnClockTimer() {
    clock_ticks_++;

    auto display = Board::GetInstance().GetDisplay();
    // Battery Saver Logic: Reduce display refresh rate in Idle
    if (battery_save_mode_ && device_state_ == kDeviceStateIdle) {
        // Slow refresh rate
        if (clock_ticks_ % 5 == 0) {
             display->UpdateStatusBar();
        }

        // Auto Dim (Low power) - 2 minutes
        // TRONG GIAI ĐOẠN NÀY: Màn hình tắt, nhưng VOICE WAKEUP VẪN HOẠT ĐỘNG
        if (clock_ticks_ == BATTERY_SAVE_DIM_TIMEOUT) {
            ESP_LOGI(TAG, "Auto-dimming screen due to inactivity");
            Board::GetInstance().SetPowerSaveMode(true);
        }

        // Auto Deep Sleep - 5 minutes
        // GIAI ĐOẠN NÀY: Tắt CPU -> CHỈ CÓ THỂ WAKEUP BẰNG NÚT BẤM
        if (clock_ticks_ >= BATTERY_SAVE_SLEEP_TIMEOUT) {
            EnterDeepSleep();
        }

    } else {
    display->UpdateStatusBar();
        }

    // Print the debug info every 10 seconds
    if (clock_ticks_ % 10 == 0) {
        // SystemInfo::PrintTaskCpuUsage(pdMS_TO_TICKS(1000));
        // SystemInfo::PrintTaskList();
        SystemInfo::PrintHeapStats();

        // If we have synchronized server time, set the status to clock "HH:MM" if the device is idle
        if (has_server_time_) {
            if (device_state_ == kDeviceStateIdle && !stt_only_mode_) {
                // Do not overwrite status in STT mode
                Schedule([this]() {
                    // Set status to clock "HH:MM"
                    time_t now = time(NULL);
                    char time_str[64];
                    strftime(time_str, sizeof(time_str), "%H:%M  ", localtime(&now));
                    Board::GetInstance().GetDisplay()->SetStatus(time_str);
                });
            }
        }
    }
}

// Add a async task to MainLoop
void Application::Schedule(std::function<void()> callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    xEventGroupSetBits(event_group_, SCHEDULE_EVENT);
}

// The Main Event Loop controls the chat state and websocket connection
// If other tasks need to access the websocket or chat state,
// they should use Schedule to call this function
void Application::MainEventLoop() {
    // Raise the priority of the main event loop to avoid being interrupted by background tasks (which has priority 2)
    vTaskPrioritySet(NULL, 3);

    while (true) {
        auto bits = xEventGroupWaitBits(event_group_, SCHEDULE_EVENT | SEND_AUDIO_EVENT, pdTRUE, pdFALSE,
                                        portMAX_DELAY);

        if (bits & SEND_AUDIO_EVENT) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto packets = std::move(audio_send_queue_);
            lock.unlock();
            for (auto &packet: packets) {
                if (!protocol_->SendAudio(packet)) {
                    break;
                }
            }
        }

        if (bits & SCHEDULE_EVENT) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto &task: tasks) {
                task();
            }
        }
    }
}

// The Audio Loop is used to input and output audio data
void Application::AudioLoop() {
    auto codec = Board::GetInstance().GetAudioCodec();
    while (true) {
        OnAudioInput();
        if (codec->output_enabled()) {
            OnAudioOutput();
        }
    }
}

void Application::OnAudioOutput() {
    if (busy_decoding_audio_) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto codec = Board::GetInstance().GetAudioCodec();
    const int max_silence_seconds = 10;

    std::unique_lock<std::mutex> lock(mutex_);
    if (audio_decode_queue_.empty()) {
        // Disable the output if there is no audio data for a long time
        if (device_state_ == kDeviceStateIdle) {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_output_time_).count();
            if (duration > max_silence_seconds) {
                codec->EnableOutput(false);
            }
        }
        return;
    }

    auto packet = std::move(audio_decode_queue_.front());
    audio_decode_queue_.pop_front();
    lock.unlock();
    audio_decode_cv_.notify_all();

    // Synchronize the sample rate and frame duration
    SetDecodeSampleRate(packet.sample_rate, packet.frame_duration);

    busy_decoding_audio_ = true;
    background_task_->Schedule([this, codec, packet = std::move(packet)]() mutable {
        busy_decoding_audio_ = false;
        if (aborted_) {
            return;
        }

        std::vector<int16_t> pcm;
        if (!opus_decoder_->Decode(std::move(packet.payload), pcm)) {
            return;
        }
        // Resample if the sample rate is different
        if (opus_decoder_->sample_rate() != codec->output_sample_rate()) {
            int target_size = output_resampler_.GetOutputSamples(pcm.size());
            std::vector<int16_t> resampled(target_size);
            output_resampler_.Process(pcm.data(), pcm.size(), resampled.data());
            pcm = std::move(resampled);
        }
        codec->OutputData(pcm);
#ifdef CONFIG_USE_SERVER_AEC
        std::lock_guard<std::mutex> lock(timestamp_mutex_);
        timestamp_queue_.push_back(packet.timestamp);
#endif
        last_output_time_ = std::chrono::steady_clock::now();
    });
}

void Application::OnAudioInput() {
    if (device_state_ == kDeviceStateAudioTesting) {
        if (audio_testing_queue_.size() >= AUDIO_TESTING_MAX_DURATION_MS / OPUS_FRAME_DURATION_MS) {
            ExitAudioTestingMode();
            return;
        }
        std::vector<int16_t> data;
        int samples = OPUS_FRAME_DURATION_MS * 16000 / 1000;
        if (ReadAudio(data, 16000, samples)) {
            background_task_->Schedule([this, data = std::move(data)]() mutable {
                opus_encoder_->Encode(std::move(data), [this](std::vector<uint8_t> &&opus) {
                    AudioStreamPacket packet;
                    packet.payload = std::move(opus);
                    packet.frame_duration = OPUS_FRAME_DURATION_MS;
                    packet.sample_rate = 16000;
                    std::lock_guard<std::mutex> lock(mutex_);
                    audio_testing_queue_.push_back(std::move(packet));
                });
            });
            return;
        }
    }

    if (wake_word_->IsDetectionRunning()) {
        std::vector<int16_t> data;
        int samples = wake_word_->GetFeedSize();
        if (samples > 0) {
            if (ReadAudio(data, 16000, samples)) {
                wake_word_->Feed(data);
                return;
            }
        }
    }

    if (audio_processor_->IsRunning()) {
        std::vector<int16_t> data;
        int samples = audio_processor_->GetFeedSize();
        if (samples > 0) {
            if (ReadAudio(data, 16000, samples)) {
                audio_processor_->Feed(data);
                return;
            }
        }
    }

    vTaskDelay(pdMS_TO_TICKS(OPUS_FRAME_DURATION_MS / 2));
}

bool Application::ReadAudio(std::vector<int16_t> &data, int sample_rate, int samples) {
    auto codec = Board::GetInstance().GetAudioCodec();
    if (!codec->input_enabled()) {
        return false;
    }

    if (codec->input_sample_rate() != sample_rate) {
        data.resize(samples * codec->input_sample_rate() / sample_rate);
        if (!codec->InputData(data)) {
            return false;
        }
        if (codec->input_channels() == 2) {
            auto mic_channel = std::vector<int16_t>(data.size() / 2);
            auto reference_channel = std::vector<int16_t>(data.size() / 2);
            for (size_t i = 0, j = 0; i < mic_channel.size(); ++i, j += 2) {
                mic_channel[i] = data[j];
                reference_channel[i] = data[j + 1];
            }
            auto resampled_mic = std::vector<int16_t>(input_resampler_.GetOutputSamples(mic_channel.size()));
            auto resampled_reference = std::vector<int16_t>(
                reference_resampler_.GetOutputSamples(reference_channel.size()));
            input_resampler_.Process(mic_channel.data(), mic_channel.size(), resampled_mic.data());
            reference_resampler_.Process(reference_channel.data(), reference_channel.size(),
                                         resampled_reference.data());
            data.resize(resampled_mic.size() + resampled_reference.size());
            for (size_t i = 0, j = 0; i < resampled_mic.size(); ++i, j += 2) {
                data[j] = resampled_mic[i];
                data[j + 1] = resampled_reference[i];
            }
        } else {
            auto resampled = std::vector<int16_t>(input_resampler_.GetOutputSamples(data.size()));
            input_resampler_.Process(data.data(), data.size(), resampled.data());
            data = std::move(resampled);
        }
    } else {
        data.resize(samples);
        if (!codec->InputData(data)) {
            return false;
        }
    }

    // Audio Debugging: Send raw audio data
    if (audio_debugger_) {
        audio_debugger_->Feed(data);
    }

    return true;
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    protocol_->SendAbortSpeaking(reason);
}

void Application::SetListeningMode(ListeningMode mode) {
    listening_mode_ = mode;
    SetDeviceState(kDeviceStateListening);
}

void Application::SetDeviceState(DeviceState state) {
    if (device_state_ == state) {
        return;
    }

    clock_ticks_ = 0;
    device_state_ = state;
    ESP_LOGI(TAG, "STATE: %s", STATE_STRINGS[device_state_]);

    // Restore power (brightness) when active
    // FIX: Do not call SetPowerSaveMode during 'starting' as WiFi is not init yet
    if (state != kDeviceStateIdle && state != kDeviceStateUnknown && state != kDeviceStateStarting) {
        Board::GetInstance().SetPowerSaveMode(false);
    }

    // OPTIMIZATION: Only wait for background task if NOT in STT only mode.
    if (!stt_only_mode_) {
        background_task_->WaitForCompletion();
    }

    auto &board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto led = board.GetLed();
    led->OnStateChanged();
    switch (state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
            display->SetStatus(Lang::Strings::STANDBY);
            display->SetEmotion("neutral");
            audio_processor_->Stop();
            wake_word_->StartDetection();
            break;
        case kDeviceStateConnecting:
            display->SetStatus(Lang::Strings::CONNECTING);
            display->SetEmotion("neutral");
            display->SetChatMessage("system", "");
            timestamp_queue_.clear();
            break;
        case kDeviceStateListening:
            display->SetStatus(Lang::Strings::LISTENING);
            display->SetEmotion("neutral");
            // Update the IoT states before sending the start listening command
#if CONFIG_IOT_PROTOCOL_XIAOZHI
            UpdateIotStates();
#endif

            // Make sure the audio processor is running
            if (!audio_processor_->IsRunning()) {
                ESP_LOGI(TAG, "STT Loop Debug: Starting audio_processor_ and sending StartListening protocol");
                // Send the start listening command
                protocol_->SendStartListening(listening_mode_);
                opus_encoder_->ResetState();
                audio_processor_->Start();
                wake_word_->StopDetection();
            } else {
                ESP_LOGW(TAG, "STT Loop Debug: audio_processor_ was already running!");
            }
            break;
        case kDeviceStateSpeaking:
            // Continuous STT Logic: If STT Only mode is on, skip Speaking state and jump back to Listening immediately
            if (stt_only_mode_) {
                ESP_LOGI(TAG, "STT Loop Debug: Skipping speaking state due to stt_only_mode, jumping to LISTENING");
                Schedule([this]() {
                    device_state_ = kDeviceStateListening;
                    ESP_LOGI(TAG, "STATE: listening (Jump from Speaking)");
                    if (!audio_processor_->IsRunning()) {
                        protocol_->SendStartListening(listening_mode_);
                        opus_encoder_->ResetState();
                        audio_processor_->Start();
                        wake_word_->StopDetection();
                    }
                    Board::GetInstance().GetLed()->OnStateChanged();
                });
                break;
            }
            display->SetStatus(Lang::Strings::SPEAKING);

            if (listening_mode_ != kListeningModeRealtime) {
                audio_processor_->Stop();
                // Only AFE wake word can be detected in speaking mode
#if CONFIG_USE_AFE_WAKE_WORD
                wake_word_->StartDetection();
#else
                wake_word_->StopDetection();
#endif
            }
            ResetDecoder();
            break;
        default:
            // Do nothing
            break;
    }
}

void Application::ResetDecoder() {
    std::lock_guard<std::mutex> lock(mutex_);
    opus_decoder_->ResetState();
    audio_decode_queue_.clear();
    audio_decode_cv_.notify_all();
    last_output_time_ = std::chrono::steady_clock::now();
    auto codec = Board::GetInstance().GetAudioCodec();
    codec->EnableOutput(true);
}

void Application::SetDecodeSampleRate(int sample_rate, int frame_duration) {
    if (opus_decoder_->sample_rate() == sample_rate && opus_decoder_->duration_ms() == frame_duration) {
        return;
    }

    opus_decoder_.reset();
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(sample_rate, 1, frame_duration);

    auto codec = Board::GetInstance().GetAudioCodec();
    if (opus_decoder_->sample_rate() != codec->output_sample_rate()) {
        ESP_LOGI(TAG, "Resampling audio from %d to %d", opus_decoder_->sample_rate(), codec->output_sample_rate());
        output_resampler_.Configure(opus_decoder_->sample_rate(), codec->output_sample_rate());
    }
}

void Application::UpdateIotStates() {
#if CONFIG_IOT_PROTOCOL_XIAOZHI
    auto &thing_manager = iot::ThingManager::GetInstance();
    std::string states;
    if (thing_manager.GetStatesJson(states, true)) {
        protocol_->SendIotStates(states);
    }
#endif
}

void Application::Reboot() {
    ESP_LOGI(TAG, "Rebooting...");
    esp_restart();
}

void Application::WakeWordInvoke(const std::string &wake_word) {
    if (device_state_ == kDeviceStateIdle) {
        ToggleChatState();
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {
        Schedule([this]() {
            if (protocol_) {
                protocol_->CloseAudioChannel();
            }
        });
    }
}

bool Application::CanEnterSleepMode() {
    if (device_state_ != kDeviceStateIdle) {
        return false;
    }

    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        return false;
    }

    // Now it is safe to enter sleep mode
    return true;
}

void Application::SendMcpMessage(const std::string &payload) {
    Schedule([this, payload]() {
        if (protocol_) {
            protocol_->SendMcpMessage(payload);
        }
    });
}

void Application::SetAecMode(AecMode mode) {
    aec_mode_ = mode;
    Schedule([this]() {
        auto &board = Board::GetInstance();
        auto display = board.GetDisplay();
        switch (aec_mode_) {
            case kAecOff:
                audio_processor_->EnableDeviceAec(false);
                display->ShowNotification(Lang::Strings::RTC_MODE_OFF);
                break;
            case kAecOnServerSide:
                audio_processor_->EnableDeviceAec(false);
                display->ShowNotification(Lang::Strings::RTC_MODE_ON);
                break;
            case kAecOnDeviceSide:
                audio_processor_->EnableDeviceAec(true);
                display->ShowNotification(Lang::Strings::RTC_MODE_ON);
                break;
        }

        // If the AEC mode is changed, close the audio channel
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
    });
}

void Application::SetSttOnlyMode(bool enable) {
    stt_only_mode_ = enable;
    auto display = Board::GetInstance().GetDisplay();

    // Update UI signal
    display->SetSttMode(enable);

    // Reset Listen Mode flag to default (Realtime/Running)
    listening_mode_ = kListeningModeRealtime;

    if (enable) {
        Schedule([this]() {
            // DELAY trước khi bắt đầu để ổn định hệ thống và tránh tiếng ồn nút bấm
            vTaskDelay(pdMS_TO_TICKS(600));

            // QUAN TRỌNG: Mở kênh TRƯỚC khi gửi WakeWord
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                     SetDeviceState(kDeviceStateIdle);
                     return;
                }
            }

                SetDeviceState(kDeviceStateListening);
        });
    } else {
        Schedule([this]() {
            SetDeviceState(kDeviceStateIdle);
        });
    }
}

void Application::SetBatterySaverMode(bool enable) {
    battery_save_mode_ = enable;
    ESP_LOGI(TAG, "Battery save mode: %d", enable);
    // Apply immediate effect if enabling and already idle
    if (enable && device_state_ == kDeviceStateIdle) {
        Board::GetInstance().SetPowerSaveMode(true);
    } else {
        Board::GetInstance().SetPowerSaveMode(false);
    }
}

void Application::EnterDeepSleep() {
    ESP_LOGI(TAG, "Entering Deep Sleep (Wakeup: Button GPIO0)...");
    Board::GetInstance().GetDisplay()->SetStatus("Deep Sleep");

    // Đợi một chút để UI kịp cập nhật
    vTaskDelay(pdMS_TO_TICKS(500));

    // Cấu hình nút BOOT (GPIO 0) để đánh thức từ Deep Sleep
    // 0 = Low level (khi nhấn nút thì GPIO 0 về mức thấp)
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);

    // Bắt đầu ngủ sâu (CPU tắt, Mic tắt -> Không dùng Voice được ở giai đoạn này)
    esp_deep_sleep_start();
}