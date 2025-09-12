#include "esp_system.h"
#include "esp_log.h"
#include "audio_pipeline.h"
#include "audio_element.h"
#include "audio_event_iface.h"
#include "i2s_stream.h"
#include "filter_resample.h"
#include "board.h"

static const char *TAG = "SPEECH_ENHANCER";

void app_main(void)
{
    ESP_LOGI(TAG, "[1.0] Initialize board");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[2.0] Create audio pipeline");
    audio_pipeline_handle_t pipeline;
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[2.1] Create I2S stream reader and writer");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_READER;
    audio_element_handle_t i2s_reader = i2s_stream_init(&i2s_cfg);

    i2s_cfg.type = AUDIO_STREAM_WRITER;
    audio_element_handle_t i2s_writer = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "[2.2] Create resample filter (optional for speech clarity)");
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = 48000;   // assume mic at 48k
    rsp_cfg.src_ch = 2;         // stereo mic
    rsp_cfg.dest_rate = 16000;  // target 16k for speech clarity
    rsp_cfg.dest_ch = 1;        // mono
    audio_element_handle_t filter = rsp_filter_init(&rsp_cfg);

    ESP_LOGI(TAG, "[3.0] Register elements to pipeline");
    audio_pipeline_register(pipeline, i2s_reader, "i2s_reader");
    audio_pipeline_register(pipeline, filter, "filter");
    audio_pipeline_register(pipeline, i2s_writer, "i2s_writer");

    ESP_LOGI(TAG, "[3.1] Link elements together");
    const char *link_tag[3] = {"i2s_reader", "filter", "i2s_writer"};
    audio_pipeline_link(pipeline, &link_tag[0], 3);

    ESP_LOGI(TAG, "[4.0] Start pipeline");
    audio_pipeline_run(pipeline);

    ESP_LOGI(TAG, "[4.1] Loop forever");
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Cleanup (never reached in this demo)
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    audio_pipeline_unregister(pipeline, i2s_reader);
    audio_pipeline_unregister(pipeline, filter);
    audio_pipeline_unregister(pipeline, i2s_writer);

    audio_pipeline_deinit(pipeline);
    audio_element_deinit(i2s_reader);
    audio_element_deinit(filter);
    audio_element_deinit(i2s_writer);
}
