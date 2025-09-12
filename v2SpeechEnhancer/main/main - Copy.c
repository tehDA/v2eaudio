/*
 * REAL-TIME SPEECH ENHANCER V7 - ADF v2.6 COMPATIBLE
 *
 * This version is written specifically for the ESP-ADF release/v2.6 API.
 * It removes LED logic to focus on core audio functionality.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "board.h"
#include "audio_pipeline.h"
#include "audio_element.h"
#include "audio_event_iface.h"
#include "i2s_stream.h"
#include "a2dp_stream.h"
#include "esp_peripherals.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "bluetooth_service.h"

static const char *TAG = "SPEECH_ENHANCER_V7";

// Enums and Global Variables
typedef enum {
    AFE_MODE_AEC_NS_AGC,
    AFE_MODE_NS_ONLY,
} afe_processing_mode_t;

static afe_processing_mode_t current_afe_mode = AFE_MODE_AEC_NS_AGC;
static const esp_afe_sr_iface_t *afe_handle;


// --- Main Application ---
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Speech Enhancer V7");

    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    bluetooth_service_cfg_t bt_cfg = { .device_name = "ESP_Speech_Enhancer" };
    bluetooth_service_start(&bt_cfg);

    audio_pipeline_handle_t pipeline;
    audio_element_handle_t i2s_stream_reader, afe_handler, i2s_stream_writer;

    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);

    // Input Element
    i2s_stream_cfg_t i2s_cfg_reader = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg_reader.type = AUDIO_STREAM_READER;
    i2s_stream_reader = i2s_stream_init(&i2s_cfg_reader);

    // AFE (Audio Front-End) Element
    // ADF v2.6 API: Use the ESP_AFE_SR_HANDLE macro
    afe_handle = &ESP_AFE_SR_HANDLE;
    // ADF v2.6 API: Use the AFE_CONFIG_DEFAULT macro
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_handler = afe_handle->create_from_config(&afe_config);

    // Wired Output Element
    i2s_stream_cfg_t i2s_cfg_writer = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg_writer.type = AUDIO_STREAM_WRITER;
    // ADF v2.6 API: Configuration uses the nested `i2s_config` struct
    i2s_cfg_writer.i2s_config.sample_rate = 16000;
    i2s_cfg_writer.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg_writer);

    audio_pipeline_register(pipeline, i2s_stream_reader, "mic");
    audio_pipeline_register(pipeline, afe_handler, "afe");
    audio_pipeline_register(pipeline, i2s_stream_writer, "wired_out");

    audio_pipeline_link(pipeline, (const char *[]){"mic", "afe", "wired_out"}, 3);

    // ADF v2.6 API: Initialize peripherals and get event listener
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t periph_set = esp_periph_set_init(&periph_cfg);
    audio_board_key_init(periph_set);
    audio_event_iface_handle_t evt = esp_periph_set_get_event_iface(periph_set);
    audio_pipeline_set_listener(pipeline, evt);
    bluetooth_service_add_listener(evt);

    audio_pipeline_run(pipeline);
    ESP_LOGI(TAG, "Pipeline running. System is ready.");

    while (1) {
        // ADF v2.6 API: Use audio_event_iface_listen
        audio_event_iface_msg_t msg;
        if (audio_event_iface_listen(evt, &msg, portMAX_DELAY) != ESP_OK) {
            ESP_LOGE(TAG, "Event interface error");
            continue;
        }

        // ADF v2.6 API: Check for peripheral source type and button commands
        if (msg.source_type == AUDIO_ELEMENT_TYPE_PERIPH &&
           (msg.cmd == PERIPH_TOUCH_TAP || msg.cmd == PERIPH_BUTTON_TAP)) {
            if ((int)msg.data == get_input_mode_id()) {
                current_afe_mode = (afe_processing_mode_t)((current_afe_mode + 1) % 2);
                ESP_LOGW(TAG, "MODE button pressed. New AFE mode: %d", current_afe_mode);
                
                audio_pipeline_pause(pipeline);
                audio_pipeline_unlink(pipeline);
                audio_pipeline_deinit_element(pipeline, afe_handler);

                afe_config = AFE_CONFIG_DEFAULT();
                if (current_afe_mode == AFE_MODE_NS_ONLY) {
                    // ADF v2.6 API: The config struct is different
                    afe_config.aec_init = false;
                    afe_config.agc_init = false;
                }
                afe_handler = afe_handle->create_from_config(&afe_config);
                audio_pipeline_register(pipeline, afe_handler, "afe");

                audio_pipeline_link(pipeline, (const char *[]){"mic", "afe", "wired_out"}, 3);
                audio_pipeline_resume(pipeline);
            }
        }

        // ADF v2.6 API: Check for Bluetooth service source and connection state commands
        if (msg.source_type == BLUETOOTH_SERVICE_TYPE) {
            if (msg.cmd == BLUETOOTH_A2DP_CONNECTION_STATE_CONNECTED) {
                ESP_LOGI(TAG, "Bluetooth connected.");
            } else if (msg.cmd == BLUETOOTH_A2DP_CONNECTION_STATE_DISCONNECTED) {
                ESP_LOGW(TAG, "Bluetooth disconnected.");
            }
        }
    }
}