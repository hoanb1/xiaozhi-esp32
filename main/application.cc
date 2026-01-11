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
#include "freertos/task.h"
#include <esp_sleep.h>

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

// --- BATTERY SAVER CONFIGURATION (Seconds) ---
#define BATTERY_SAVE_DIM_TIMEOUT   60
#define BATTERY_SAVE_OFF_TIMEOUT   120
#define BATTERY_SAVE_SLEEP_TIMEOUT 600

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
    ESP_LOGI(TAG, "Initializing Application instance");
    event_group_ = xEventGroupCreate();
    background_task_ = new BackgroundTask(4096 * 7);

    battery_save_mode_ = true;

#if CONFIG_USE_DEVICE_AEC
    aec_mode_ = kAecOnDeviceSide;
    ESP_LOGI(TAG, "[Init] AEC Mode: Device Side");
#elif CONFIG_USE_SERVER_AEC
    aec_mode_ = kAecOnServerSide;
    ESP_LOGI(TAG, "[Init] AEC Mode: Server Side");
#else
    aec_mode_ = kAecOff;
    ESP_LOGI(TAG, "[Init] AEC Mode: Off");
#endif

#if CONFIG_USE_AUDIO_PROCESSOR
    audio_processor_ = std::make_unique<AfeAudioProcessor>();
    ESP_LOGI(TAG, "[Init] Audio Processor: AFE");
#else
    audio_processor_ = std::make_unique<NoAudioProcessor>();
    ESP_LOGW(TAG, "[Init] Audio Processor: NONE");
#endif

#if CONFIG_USE_AFE_WAKE_WORD
    wake_word_ = std::make_unique<AfeWakeWord>();
    ESP_LOGI(TAG, "[Init] Wake-word Engine: AFE");
#elif CONFIG_USE_ESP_WAKE_WORD
    wake_word_ = std::make_unique<EspWakeWord>();
    ESP_LOGI(TAG, "[Init] Wake-word Engine: ESP");
#else
    wake_word_ = std::make_unique<NoWakeWord>();
    ESP_LOGW(TAG, "[Init] Wake-word Engine: NONE");
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
    ESP_LOGW(TAG, "Destroying Application instance");
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
    ESP_LOGI(TAG, "[OTA] Checking for new firmware version");
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
                ESP_LOGE(TAG, "[OTA] Max retries reached, exiting version check");
                return;
            }

            char buffer[128];
            snprintf(buffer, sizeof(buffer), Lang::Strings::CHECK_NEW_VERSION_FAILED, retry_delay,
                     ota.GetCheckVersionUrl().c_str());
            Alert(Lang::Strings::ERROR, buffer, "sad", Lang::Sounds::P3_EXCLAMATION);

            ESP_LOGW(TAG, "[OTA] Check failed, retry in %d seconds (%d/%d)", retry_delay, retry_count, MAX_RETRY);
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
            ESP_LOGI(TAG, "[OTA] New version found: %s. Starting upgrade process...", ota.GetFirmwareVersion().c_str());
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
            if (codec->input_enabled()) codec->EnableInput(false);
            if (codec->output_enabled()) codec->EnableOutput(false);

            {
                std::lock_guard<std::mutex> lock(mutex_);
                audio_decode_queue_.clear();
            }
            background_task_->WaitForCompletion();
            delete background_task_;
            background_task_ = nullptr;
            vTaskDelay(pdMS_TO_TICKS(1000));

            ota.StartUpgrade([display](int progress, size_t speed) {
                ESP_LOGD(TAG, "[OTA] Progress: %d%% (%u KB/s)", progress, (unsigned int)(speed / 1024));
                char buffer[64];
                snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
                display->SetChatMessage("system", buffer);
            });

            // If upgrade success, the device will reboot and never reach here
            display->SetStatus(Lang::Strings::UPGRADE_FAILED);
            ESP_LOGE(TAG, "[OTA] Firmware upgrade failed...");
            vTaskDelay(pdMS_TO_TICKS(3000));
            Reboot();
            return;
        }

        ota.MarkCurrentVersionValid();
        if (!ota.HasActivationCode() && !ota.HasActivationChallenge()) {
            ESP_LOGI(TAG, "[OTA] No updates available, activation not required");
            xEventGroupSetBits(event_group_, CHECK_NEW_VERSION_DONE_EVENT);
            break;
        }

        display->SetStatus(Lang::Strings::ACTIVATION);
        if (ota.HasActivationCode()) {
            ESP_LOGI(TAG, "[OTA] Activation required. Code: %s", ota.GetActivationCode().c_str());
            ShowActivationCode(ota.GetActivationCode(), ota.GetActivationMessage());
        }

        for (int i = 0; i < 10; ++i) {
            ESP_LOGI(TAG, "[OTA] Attempting activation... %d/%d", i + 1, 10);
            esp_err_t err = ota.Activate();
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "[OTA] Activation successful");
                xEventGroupSetBits(event_group_, CHECK_NEW_VERSION_DONE_EVENT);
                break;
            } else if (err == ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "[OTA] Activation timeout, retrying...");
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
    ESP_LOGW(TAG, "ALERT: Status=%s, Msg=%s", status, message);
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
        ESP_LOGD(TAG, "Dismissing alert UI");
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::STANDBY);
        display->SetEmotion("neutral");
        display->SetChatMessage("system", "");
    }
}

void Application::PlaySound(const std::string_view &sound) {
    ESP_LOGD(TAG, "Playing internal audio asset");
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
        packet.sample_rate = AUDIO_FILE_SAMPLE_RATE;
        packet.frame_duration = 60;
        packet.payload.resize(payload_size);
        memcpy(packet.payload.data(), p3->payload, payload_size);
        p += payload_size;

        std::lock_guard<std::mutex> lock(mutex_);
        audio_decode_queue_.emplace_back(std::move(packet));
    }
}

void Application::EnterAudioTestingMode() {
    ESP_LOGI(TAG, "[Mode] Entering audio testing");
    ResetDecoder();
    SetDeviceState(kDeviceStateAudioTesting);
}

void Application::ExitAudioTestingMode() {
    ESP_LOGI(TAG, "[Mode] Exiting audio testing");
    SetDeviceState(kDeviceStateSpeaking);
    std::lock_guard<std::mutex> lock(mutex_);
    audio_decode_queue_ = std::move(audio_testing_queue_);
    audio_decode_cv_.notify_all();
}

void Application::ToggleChatState() {
    ESP_LOGI(TAG, "[Event] ToggleChatState (Current State: %s)", STATE_STRINGS[device_state_]);
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

    // --- STT ONLY MODE LOGIC ---
    if (stt_only_mode_) {
        if (device_state_ == kDeviceStateListening || device_state_ == kDeviceStateSpeaking || device_state_ ==
            kDeviceStateConnecting) {
            ESP_LOGI(TAG, "[STT Mode] User action: PAUSE");
            listening_mode_ = kListeningModeManualStop;
            Schedule([this]() {
                if (protocol_->IsAudioChannelOpened()) {
                    protocol_->CloseAudioChannel();
                } else {
                    SetDeviceState(kDeviceStateIdle);
                }
            });
        }
        else {
            ESP_LOGI(TAG, "[STT Mode] User action: RESUME");
            listening_mode_ = kListeningModeRealtime;
            Schedule([this]() {
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

    // --- NORMAL MODE LOGIC ---
    if (device_state_ == kDeviceStateIdle) {
        ESP_LOGI(TAG, "[Normal Mode] Starting session");
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
        ESP_LOGI(TAG, "[Normal Mode] Aborting speech");
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {
        ESP_LOGI(TAG, "[Normal Mode] Stop listening manually");
        Schedule([this]() {
            protocol_->CloseAudioChannel();
        });
    }
}

void Application::StartListening() {
    ESP_LOGI(TAG, "[Event] StartListening requested");
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
    ESP_LOGI(TAG, "[Event] StopListening requested");
    if (device_state_ == kDeviceStateAudioTesting) {
        ExitAudioTestingMode();
        return;
    }

    const std::array<int, 3> valid_states = {
        kDeviceStateListening,
        kDeviceStateSpeaking,
        kDeviceStateIdle,
    };

    if (std::find(valid_states.begin(), valid_states.end(), device_state_) == valid_states.end()) {
        ESP_LOGW(TAG, "StopListening ignored in current state: %s", STATE_STRINGS[device_state_]);
        return;
    }

    Schedule([this]() {
        if (device_state_ == kDeviceStateListening) {
            protocol_->SendStopListening();
            SetDeviceState(kDeviceStateIdle);
        }
    });
}

// --- MESSAGE HANDLERS ---

void Application::HandleSttMode(const cJSON *root) {
    auto display = Board::GetInstance().GetDisplay();
    auto type = cJSON_GetObjectItem(root, "type");

    if (strcmp(type->valuestring, "stt") == 0) {
        auto text = cJSON_GetObjectItem(root, "text");
        if (cJSON_IsString(text)) {
            ESP_LOGI(TAG, "[STT-Only] Recognition: %s", text->valuestring);
            Schedule([this, display, message = std::string(text->valuestring)]() {
                display->SetChatMessage("user", message.c_str());
            });
        }
    }
    else if (strcmp(type->valuestring, "tts") == 0) {
        auto state = cJSON_GetObjectItem(root, "state");
        ESP_LOGD(TAG, "[STT-Only] TTS state: %s", state->valuestring);
        }
    else if (strcmp(type->valuestring, "system") == 0) {
        auto command = cJSON_GetObjectItem(root, "command");
        ESP_LOGI(TAG, "[STT-Only] System command: %s", command->valuestring);
        if (cJSON_IsString(command) && strcmp(command->valuestring, "reboot") == 0) {
            Schedule([this]() { Reboot(); });
        }
    }

#if CONFIG_IOT_PROTOCOL_MCP
    else if (strcmp(type->valuestring, "mcp") == 0) {
        auto payload = cJSON_GetObjectItem(root, "payload");
        if (cJSON_IsObject(payload)) {
            ESP_LOGI(TAG, "[STT-Only] Processing MCP Command");
            McpServer::GetInstance().ParseMessage(payload);
        }
    }
#endif
}

void Application::HandleNormalMode(const cJSON *root) {
    auto display = Board::GetInstance().GetDisplay();
    auto type = cJSON_GetObjectItem(root, "type");

    if (strcmp(type->valuestring, "tts") == 0) {
        auto state = cJSON_GetObjectItem(root, "state");
        ESP_LOGD(TAG, "[Normal] TTS State: %s", state->valuestring);
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
                ESP_LOGI(TAG, "[Assistant] %s", text->valuestring);
                Schedule([this, display, message = std::string(text->valuestring)]() {
                    display->SetChatMessage("assistant", message.c_str());
                });
            }
        }
    } else if (strcmp(type->valuestring, "stt") == 0) {
        auto text = cJSON_GetObjectItem(root, "text");
        if (cJSON_IsString(text)) {
            ESP_LOGI(TAG, "[User] %s", text->valuestring);
            Schedule([this, display, message = std::string(text->valuestring)]() {
                display->SetChatMessage("user", message.c_str());
            });
        }
    } else if (strcmp(type->valuestring, "llm") == 0) {
        auto emotion = cJSON_GetObjectItem(root, "emotion");
        if (cJSON_IsString(emotion)) {
            ESP_LOGD(TAG, "[LLM] Emotion change: %s", emotion->valuestring);
            Schedule([this, display, emotion_str = std::string(emotion->valuestring)]() {
                display->SetEmotion(emotion_str.c_str());
            });
        }
#if CONFIG_IOT_PROTOCOL_MCP
    } else if (strcmp(type->valuestring, "mcp") == 0) {
        auto payload = cJSON_GetObjectItem(root, "payload");
        if (cJSON_IsObject(payload)) {
            ESP_LOGI(TAG, "[Normal] Processing MCP Message");
            McpServer::GetInstance().ParseMessage(payload);
        }
#endif
#if CONFIG_IOT_PROTOCOL_XIAOZHI
    } else if (strcmp(type->valuestring, "iot") == 0) {
            auto commands = cJSON_GetObjectItem(root, "commands");
            if (cJSON_IsArray(commands)) {
                ESP_LOGI(TAG, "[IOT] Processing %d commands", cJSON_GetArraySize(commands));
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
            ESP_LOGW(TAG, "[System] Command: %s", command->valuestring);
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
        ESP_LOGI(TAG, "[System] Battery save set to: %s", enable ? "On" : "Off");
        SetBatterySaverMode(enable);
    }
}


void Application::Start() {
    ESP_LOGI(TAG, "Application service starting...");
    auto &board = Board::GetInstance();
    SetDeviceState(kDeviceStateStarting);

    auto display = board.GetDisplay();
    auto codec = board.GetAudioCodec();

    ESP_LOGI(TAG, "[Audio] Hardware SR: Out=%d, In=%d", codec->output_sample_rate(), codec->input_sample_rate());

    // Server communication uses Hardware Sample Rate (e.g., 24kHz)
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(codec->output_sample_rate(), 1, OPUS_FRAME_DURATION_MS);
    opus_encoder_ = std::make_unique<OpusEncoderWrapper>(codec->input_sample_rate(), 1, OPUS_FRAME_DURATION_MS);

    if (aec_mode_ != kAecOff) {
        ESP_LOGD(TAG, "[Audio] AEC active (%d), complexity=0", aec_mode_);
        opus_encoder_->SetComplexity(0);
    } else if (board.GetBoardType() == "ml307") {
        ESP_LOGD(TAG, "[Audio] ML307 detected, complexity=5");
        opus_encoder_->SetComplexity(5);
    } else {
        opus_encoder_->SetComplexity(0);
    }

    // Local AI Model / Wake word requires 16kHz
    if (codec->input_sample_rate() != AUDIO_MODEL_SAMPLE_RATE) {
        ESP_LOGI(TAG, "[Audio] Local Resampler: %d -> %d", codec->input_sample_rate(), AUDIO_MODEL_SAMPLE_RATE);
        input_resampler_.Configure(codec->input_sample_rate(), AUDIO_MODEL_SAMPLE_RATE);
        reference_resampler_.Configure(codec->input_sample_rate(), AUDIO_MODEL_SAMPLE_RATE);
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

    esp_timer_start_periodic(clock_timer_handle_, 1000000);

    display->SetStatus(Lang::Strings::CONNECTING);
    ESP_LOGI(TAG, "[Network] Connecting...");
    board.StartNetwork();

    display->UpdateStatusBar(true);

    Ota ota;
    display->SetStatus(Lang::Strings::LOADING_PROTOCOL);

#if CONFIG_IOT_PROTOCOL_MCP
    ESP_LOGI(TAG, "[System] Loading MCP tools");
    McpServer::GetInstance().AddCommonTools();
#endif

    if (ota.HasMqttConfig()) {
        ESP_LOGI(TAG, "[Protocol] Active: MQTT");
        protocol_ = std::make_unique<MqttProtocol>();
    } else if (ota.HasWebsocketConfig()) {
        ESP_LOGI(TAG, "[Protocol] Active: WebSocket");
        protocol_ = std::make_unique<WebsocketProtocol>();
    } else {
        ESP_LOGW(TAG, "[Protocol] No config, using default MQTT");
        protocol_ = std::make_unique<MqttProtocol>();
    }

    protocol_->OnNetworkError([this](const std::string &message) {
        ESP_LOGE(TAG, "[Network] Protocol error: %s", message.c_str());
        SetDeviceState(kDeviceStateIdle);
        Alert(Lang::Strings::ERROR, message.c_str(), "sad", Lang::Sounds::P3_EXCLAMATION);
    });
    protocol_->OnIncomingAudio([this](AudioStreamPacket &&packet) {
        if (stt_only_mode_) {
            ESP_LOGV(TAG, "[Audio] Incoming stream ignored (STT Mode)");
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (device_state_ == kDeviceStateSpeaking && audio_decode_queue_.size() < MAX_AUDIO_PACKETS_IN_QUEUE) {
            audio_decode_queue_.emplace_back(std::move(packet));
        }
    });
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        ESP_LOGI(TAG, "[Protocol] Audio channel opened (SR: %d)", protocol_->server_sample_rate());
        board.SetPowerSaveMode(false);
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(
                TAG,
                "[Audio] SR mismatch! Server: %d vs Device: %d",
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

    protocol_->OnAudioChannelClosed([this]() {
        ESP_LOGI(TAG, "[Protocol] Audio channel closed");
        Schedule([this]() {
            auto display = Board::GetInstance().GetDisplay();

            if (stt_only_mode_) {
                ESP_LOGI(TAG, "[STT-Only] Session ended");
                SetDeviceState(kDeviceStateIdle);
                listening_mode_ = kListeningModeManualStop;
            } else {
                SetDeviceState(kDeviceStateIdle);
                display->SetChatMessage("system", "");
            }
        });
    });

    protocol_->OnIncomingJson([this](const cJSON *root) {
        if (stt_only_mode_) {
            HandleSttMode(root);
        } else {
            HandleNormalMode(root);
        }
    });

    ESP_LOGI(TAG, "[Protocol] Connection engine starting");
    bool protocol_started = protocol_->Start();

    audio_debugger_ = std::make_unique<AudioDebugger>();
    audio_processor_->Initialize(codec);
    audio_processor_->OnOutput([this](std::vector<int16_t> &&data) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (audio_send_queue_.size() >= MAX_AUDIO_PACKETS_IN_QUEUE) {
                return;
            }
        }
        // Encode and send audio at hardware sample rate (e.g. 24kHz)
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
                        timestamp_queue_.pop_front();
                        return;
                    }
                }
#endif
                std::lock_guard<std::mutex> lock(mutex_);
                if (audio_send_queue_.size() >= MAX_AUDIO_PACKETS_IN_QUEUE) {
                    audio_send_queue_.pop_front();
                }
                audio_send_queue_.emplace_back(std::move(packet));
                xEventGroupSetBits(event_group_, SEND_AUDIO_EVENT);
            });
        });
    });
    audio_processor_->OnVadStateChange([this](bool speaking) {
        if (device_state_ == kDeviceStateListening) {
            ESP_LOGV(TAG, "[Audio] VAD: %s", speaking ? "Speaking" : "Silence");
            Schedule([this, speaking]() {
                voice_detected_ = speaking;
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            });
        }
    });

    wake_word_->Initialize(codec);
    wake_word_->OnWakeWordDetected([this](const std::string &wake_word) {
        ESP_LOGI(TAG, "[Wake] Detected: %s", wake_word.c_str());
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

#if CONFIG_USE_AFE_WAKE_WORD
                AudioStreamPacket packet;
                while (wake_word_->GetWakeWordOpus(packet.payload)) {
                    protocol_->SendAudio(packet);
                }
#else
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

    SetDeviceState(kDeviceStateIdle);

    has_server_time_ = ota.HasServerTime();
    if (protocol_started) {
        std::string message = std::string(Lang::Strings::VERSION) + ota.GetCurrentVersion();
        display->ShowNotification(message.c_str());
        display->SetChatMessage("system", "");
        ResetDecoder();
        PlaySound(Lang::Sounds::P3_SUCCESS);
    }

    ESP_LOGI(TAG, "Application service READY. Heap: %lu", (unsigned long)esp_get_free_heap_size());
    SystemInfo::PrintHeapStats();

    MainEventLoop();
}

void Application::OnClockTimer() {
    clock_ticks_++;

    auto display = Board::GetInstance().GetDisplay();
    if (battery_save_mode_ && device_state_ == kDeviceStateIdle) {
        if (clock_ticks_ % 5 == 0) {
             display->UpdateStatusBar();
        }

        // --- 3-STAGE BATTERY SAVER LOGIC ---

        if (clock_ticks_ == BATTERY_SAVE_DIM_TIMEOUT) {
            ESP_LOGI(TAG, "[Power] Stage 1: Dimming screen");
            Board::GetInstance().SetPowerSaveMode(true);
        }

        if (clock_ticks_ == BATTERY_SAVE_OFF_TIMEOUT) {
            ESP_LOGI(TAG, "[Power] Stage 2: LCD Off");
            auto backlight = Board::GetInstance().GetBacklight();
            if (backlight) backlight->SetBrightness(0);
        }

        if (clock_ticks_ >= BATTERY_SAVE_SLEEP_TIMEOUT) {
            ESP_LOGW(TAG, "[Power] Stage 3: Deep Sleep");
            EnterDeepSleep();
        }

    } else {
    display->UpdateStatusBar();
        }

    if (clock_ticks_ % 10 == 0) {
        SystemInfo::PrintHeapStats();

        if (has_server_time_) {
            if (device_state_ == kDeviceStateIdle && !stt_only_mode_) {
                Schedule([this]() {
                    time_t now = time(NULL);
                    char time_str[64];
                    strftime(time_str, sizeof(time_str), "%H:%M  ", localtime(&now));
                    Board::GetInstance().GetDisplay()->SetStatus(time_str);
                });
            }
        }
    }
}

void Application::Schedule(std::function<void()> callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    xEventGroupSetBits(event_group_, SCHEDULE_EVENT);
}

void Application::MainEventLoop() {
    vTaskPrioritySet(NULL, 3);
    ESP_LOGI(TAG, "Main Event Loop started");

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

void Application::AudioLoop() {
    auto codec = Board::GetInstance().GetAudioCodec();
    ESP_LOGI(TAG, "Audio task loop started");
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
        if (device_state_ == kDeviceStateIdle) {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_output_time_).count();
            if (duration > max_silence_seconds) {
                if (codec->output_enabled()) codec->EnableOutput(false);
            }
        }
        return;
    }

    auto packet = std::move(audio_decode_queue_.front());
    audio_decode_queue_.pop_front();
    lock.unlock();
    audio_decode_cv_.notify_all();

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
    auto codec = Board::GetInstance().GetAudioCodec();

    // 1. Handling Audio Testing Mode (If applicable)
    if (device_state_ == kDeviceStateAudioTesting) {
        if (audio_testing_queue_.size() >= AUDIO_TESTING_MAX_DURATION_MS / OPUS_FRAME_DURATION_MS) {
            ExitAudioTestingMode();
            return;
        }
        std::vector<int16_t> data;
        // Use AUDIO_MODEL_SAMPLE_RATE (16kHz) for testing?
        // Or hardware rate? Let's assume 16kHz for simplicity in test engine
        int samples = OPUS_FRAME_DURATION_MS * AUDIO_MODEL_SAMPLE_RATE / 1000;
        if (ReadAudio(data, AUDIO_MODEL_SAMPLE_RATE, samples)) {
            background_task_->Schedule([this, data = std::move(data)]() mutable {
                opus_encoder_->Encode(std::move(data), [this](std::vector<uint8_t> &&opus) {
                    AudioStreamPacket packet;
                    packet.payload = std::move(opus);
                    packet.frame_duration = OPUS_FRAME_DURATION_MS;
                    packet.sample_rate = AUDIO_MODEL_SAMPLE_RATE;
                    std::lock_guard<std::mutex> lock(mutex_);
                    audio_testing_queue_.push_back(std::move(packet));
                });
            });
            return;
        }
    }

    // 2. Local Processing: Wake Word & AFE (Must be 16kHz)
    if (wake_word_->IsDetectionRunning() || audio_processor_->IsRunning()) {
        std::vector<int16_t> data_16k;
        int samples_needed = wake_word_->IsDetectionRunning() ? wake_word_->GetFeedSize() : audio_processor_->GetFeedSize();

        if (samples_needed > 0) {
            if (ReadAudio(data_16k, AUDIO_MODEL_SAMPLE_RATE, samples_needed)) {
                if (wake_word_->IsDetectionRunning()) wake_word_->Feed(data_16k);
                if (audio_processor_->IsRunning()) audio_processor_->Feed(data_16k);
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

    // Determine how many hardware samples we need to read
    if (codec->input_sample_rate() != sample_rate) {
        // Resampling needed (usually from 24kHz hardware to 16kHz engine)
        data.resize(samples * codec->input_sample_rate() / sample_rate);
        if (!codec->InputData(data)) {
            return false;
        }

        // Handling Multi-channel if necessary
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
            // Mono Resampling 24k -> 16k
            auto resampled = std::vector<int16_t>(input_resampler_.GetOutputSamples(data.size()));
            input_resampler_.Process(data.data(), data.size(), resampled.data());
            data = std::move(resampled);
        }
    } else {
        // Direct read (Usually 24kHz for Server or 16kHz if hardware matches)
        data.resize(samples);
        if (!codec->InputData(data)) {
            return false;
        }
    }

    if (audio_debugger_) {
        audio_debugger_->Feed(data);
    }

    return true;
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking (%d)", (int)reason);
    aborted_ = true;
    protocol_->SendAbortSpeaking(reason);
}

void Application::SetListeningMode(ListeningMode mode) {
    listening_mode_ = mode;
    SetDeviceState(kDeviceStateListening);
}

void Application::SetDeviceState(DeviceState state) {
    if (device_state_ == state) return;

    DeviceState old_state = device_state_;
    clock_ticks_ = 0;
    device_state_ = state;
    ESP_LOGI(TAG, "STATE: %s -> %s", STATE_STRINGS[old_state], STATE_STRINGS[device_state_]);

    auto &board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto led = board.GetLed();
    auto codec = board.GetAudioCodec();
    led->OnStateChanged();

    if (state != kDeviceStateIdle && state != kDeviceStateUnknown && state != kDeviceStateStarting) {
        board.SetPowerSaveMode(false);
    }

    if (!stt_only_mode_) {
        background_task_->WaitForCompletion();
    }

    switch (state) {
        case kDeviceStateIdle:
            display->SetStatus(Lang::Strings::STANDBY);
            display->SetEmotion("neutral");
            audio_processor_->Stop();

            if (codec->output_enabled()) codec->EnableOutput(false);
            if (!codec->input_enabled()) codec->EnableInput(true);

            wake_word_->StartDetection();
            break;

        case kDeviceStateConnecting:
            display->SetStatus(Lang::Strings::CONNECTING);
            display->SetChatMessage("system", "");
            timestamp_queue_.clear();
            break;

        case kDeviceStateListening:
            display->SetStatus(Lang::Strings::LISTENING);

            if (codec->output_enabled()) codec->EnableOutput(false);
            if (!codec->input_enabled()) codec->EnableInput(true);

            if (!audio_processor_->IsRunning()) {
                protocol_->SendStartListening(listening_mode_);
                opus_encoder_->ResetState();
                audio_processor_->Start();
                wake_word_->StopDetection();
            }
            break;

        case kDeviceStateSpeaking:
            if (stt_only_mode_) {
                device_state_ = kDeviceStateListening;

                if (codec->output_enabled()) codec->EnableOutput(false);
                if (!codec->input_enabled()) codec->EnableInput(true);

                if (!audio_processor_->IsRunning()) {
                    protocol_->SendStartListening(listening_mode_);
                    opus_encoder_->ResetState();
                    audio_processor_->Start();
                    wake_word_->StopDetection();
                }
                led->OnStateChanged();
                break;
            }
            display->SetStatus(Lang::Strings::SPEAKING);

            if (codec->input_enabled()) codec->EnableInput(false);
            if (!codec->output_enabled()) codec->EnableOutput(true);

            if (listening_mode_ != kListeningModeRealtime) {
                audio_processor_->Stop();
                wake_word_->StartDetection();
            }
            ResetDecoder();
            break;

        case kDeviceStateAudioTesting:
            display->SetStatus("Testing mode");
            if (!codec->input_enabled()) codec->EnableInput(true);
            if (codec->output_enabled()) codec->EnableOutput(false);

            wake_word_->StopDetection();
            audio_processor_->Stop();
            break;

        case kDeviceStateUpgrading:
            display->SetStatus(Lang::Strings::UPGRADING);
            if (codec->input_enabled()) codec->EnableInput(false);
            if (codec->output_enabled()) codec->EnableOutput(false);
            break;

        case kDeviceStateActivating:
            display->SetStatus(Lang::Strings::ACTIVATION);
            break;

        case kDeviceStateFatalError:
            display->SetStatus(Lang::Strings::ERROR);
            display->SetEmotion("sad");
            if (codec->input_enabled()) codec->EnableInput(false);
            if (codec->output_enabled()) codec->EnableOutput(false);
            break;

        default:
            break;
    }
}

void Application::ResetDecoder() {
    ESP_LOGV(TAG, "[Audio] Resetting decoder");
    std::lock_guard<std::mutex> lock(mutex_);
    opus_decoder_->ResetState();
    audio_decode_queue_.clear();
    audio_decode_cv_.notify_all();
    last_output_time_ = std::chrono::steady_clock::now();
}

void Application::SetDecodeSampleRate(int sample_rate, int frame_duration) {
    if (opus_decoder_->sample_rate() == sample_rate && opus_decoder_->duration_ms() == frame_duration) {
        return;
    }

    ESP_LOGD(TAG, "[Audio] Decoder SR: %d, Duration: %dms", sample_rate, frame_duration);
    opus_decoder_.reset();
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(sample_rate, 1, frame_duration);

    auto codec = Board::GetInstance().GetAudioCodec();
    if (opus_decoder_->sample_rate() != codec->output_sample_rate()) {
        ESP_LOGD(TAG, "[Audio] Output resampling: %d -> %d", opus_decoder_->sample_rate(), codec->output_sample_rate());
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
    ESP_LOGW(TAG, "Rebooting device...");
    esp_restart();
}

void Application::WakeWordInvoke(const std::string &wake_word) {
    ESP_LOGI(TAG, "[Wake] Invoke triggered");
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
    return (device_state_ == kDeviceStateIdle) && !(protocol_ && protocol_->IsAudioChannelOpened());
}

void Application::SendMcpMessage(const std::string &payload) {
    ESP_LOGD(TAG, "[MCP] Outbound: %s", payload.c_str());
    Schedule([this, payload]() {
        if (protocol_) {
            protocol_->SendMcpMessage(payload);
        }
    });
}

void Application::SetAecMode(AecMode mode) {
    ESP_LOGI(TAG, "Setting AEC mode: %d", (int)mode);
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

        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
    });
}

void Application::SetSttOnlyMode(bool enable) {
    ESP_LOGI(TAG, "[Mode] STT Only: %s", enable ? "On" : "Off");
    stt_only_mode_ = enable;
    auto display = Board::GetInstance().GetDisplay();

    display->SetSttMode(enable);
    listening_mode_ = kListeningModeRealtime;

    if (enable) {
        Schedule([this]() {
            vTaskDelay(pdMS_TO_TICKS(600));

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
    ESP_LOGI(TAG, "[Power] Battery Saver: %s", enable ? "On" : "Off");
    battery_save_mode_ = enable;
    Board::GetInstance().SetPowerSaveMode(enable && device_state_ == kDeviceStateIdle);
}

void Application::EnterDeepSleep() {
    ESP_LOGW(TAG, "!!! DEEP SLEEP !!!");
    Board::GetInstance().GetDisplay()->SetStatus("Deep Sleep");

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
    esp_deep_sleep_start();
}