// SPDX-License-Identifier: MIT
// eaudio — LyraTD-MSC (ESP32 + ZL38063) with ADC buttons/LEDs
// Target: ESP-IDF v5.3.x, ESP-ADF master — LIGHT CLEAN (NS+AGC), TYPE1 "M"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

// ADC One-Shot + Calibration (IDF 5.x)
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ADF
#include "audio_pipeline.h"
#include "audio_element.h"
#include "audio_common.h"
#include "i2s_stream.h"
#include "algorithm_stream.h"
#include "audio_hal.h"
#include "board.h"

static const char *TAG = "eaudio_main";

// ===== LEDs =====
#define GPIO_LED_MODE     2
#define GPIO_LED_CLEAN    4
#define GPIO_LED_BF       5

// ===== ADC buttons =====
#define BTN_ADC_UNIT          ADC_UNIT_1
#define BTN_ADC_CHANNEL       ADC_CHANNEL_6
#define BTN_ADC_ATTEN         ADC_ATTEN_DB_12
#define BTN_ADC_BITWIDTH      ADC_BITWIDTH_DEFAULT
#define BTN_SAMPLE_COUNT      8
#define BTN_STABLE_SAMPLES    3
#define BTN_POLL_MS           30

// Audio
#define E_AUDIO_RATE_HZ          16000
#define E_AUDIO_VOL_HP_DEFAULT   70
#define E_AEC_INIT_DELAY_MS      70
#define E_AEC_MIN_MS             0
#define E_AEC_MAX_MS             300

typedef enum { MODE_PASSTHROUGH=0, MODE_CLEAN=1, MODE_BF_PLACEHOLDER=2, MODE_MAX } run_mode_t;

typedef enum { BTN_VOL_PLUS=0, BTN_VOL_MINUS, BTN_MODE, BTN_REC, BTN_PLAY, BTN_SET, BTN_COUNT, BTN_NONE=0xFF } btn_id_t;

static const char *kButtons[BTN_COUNT] = {"VOL+", "VOL-", "MODE", "REC", "PLAY", "SET"};
static const int   kButtonLevels_mV[BTN_COUNT] = {2880, 2270, 1800, 1340, 820, 300};
static int         sThresh_mV[BTN_COUNT - 1];

// --------- Globals ---------
static audio_board_handle_t board = NULL;

// CLEAN pipeline
static audio_pipeline_handle_t pipe_clean = NULL;
static audio_element_handle_t  clean_reader = NULL;
static audio_element_handle_t  clean_algo   = NULL;
static audio_element_handle_t  clean_writer = NULL;
static ringbuf_handle_t        clean_writer_rb = NULL;

// PASS pipeline
static audio_pipeline_handle_t pipe_pass = NULL;
static audio_element_handle_t  pass_reader = NULL;
static audio_element_handle_t  pass_writer = NULL;

// State
static run_mode_t run_mode = MODE_CLEAN;
static int  hp_volume = E_AUDIO_VOL_HP_DEFAULT;
static bool hp_mute   = false;
static int  aec_delay_ms = E_AEC_INIT_DELAY_MS;

// ADC
static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t s_adc_cali = NULL;
static bool s_adc_cali_enabled = false;

// ===== LED helpers =====
static void leds_hw_write(bool m, bool c, bool b) {
    gpio_set_level(GPIO_LED_MODE, m);
    gpio_set_level(GPIO_LED_CLEAN, c);
    gpio_set_level(GPIO_LED_BF, b);
}
static void leds_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<GPIO_LED_MODE) | (1ULL<<GPIO_LED_CLEAN) | (1ULL<<GPIO_LED_BF),
        .mode = GPIO_MODE_OUTPUT, .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    leds_hw_write(false,false,false);
}
static void leds_show_mode(run_mode_t m) {
    leds_hw_write(m==MODE_PASSTHROUGH, m==MODE_CLEAN, m==MODE_BF_PLACEHOLDER);
}

// ===== Volume / Mute =====
static void apply_volume(void) {
    if (!board) return;
    audio_hal_set_mute(board->audio_hal, hp_mute);
    if (!hp_mute) audio_hal_set_volume(board->audio_hal, hp_volume);
    ESP_LOGI(TAG, "HP vol=%d mute=%d", hp_volume, hp_mute);
}

// ===== AEC delay (kept for future, harmless while AEC is OFF) =====
static void apply_aec_delay(void) {
    if (aec_delay_ms < E_AEC_MIN_MS) aec_delay_ms = E_AEC_MIN_MS;
    if (aec_delay_ms > E_AEC_MAX_MS) aec_delay_ms = E_AEC_MAX_MS;
    if (clean_algo && clean_writer_rb) {
        algo_stream_set_delay(clean_algo, clean_writer_rb, aec_delay_ms);
        ESP_LOGI(TAG, "AEC delay -> %d ms", aec_delay_ms);
    }
}

// ===== ADC buttons =====
static void adc_init_buttons(void)
{
    for (int i = 0; i < BTN_COUNT - 1; ++i) sThresh_mV[i] = (kButtonLevels_mV[i] + kButtonLevels_mV[i+1]) / 2;

    adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = BTN_ADC_UNIT, .ulp_mode = ADC_ULP_MODE_DISABLE};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {.bitwidth = BTN_ADC_BITWIDTH, .atten = BTN_ADC_ATTEN};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, BTN_ADC_CHANNEL, &chan_cfg));

    adc_cali_line_fitting_config_t cali_cfg = {.unit_id = BTN_ADC_UNIT, .atten = BTN_ADC_ATTEN, .bitwidth = BTN_ADC_BITWIDTH};
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali) == ESP_OK) {
        s_adc_cali_enabled = true;
        ESP_LOGI(TAG, "ADC calibration: line fitting enabled");
    } else {
        s_adc_cali_enabled = false;
        s_adc_cali = NULL;
        ESP_LOGW(TAG, "ADC calibration not available, using raw-to-mV scaling");
    }
}
static int adc_read_avg_mv(void)
{
    int acc_mv = 0;
    for (int i = 0; i < BTN_SAMPLE_COUNT; ++i) {
        int raw = 0, mv = 0;
        if (adc_oneshot_read(s_adc, BTN_ADC_CHANNEL, &raw) != ESP_OK) continue;
        if (s_adc_cali_enabled) adc_cali_raw_to_voltage(s_adc_cali, raw, &mv);
        else mv = (raw * 3300) / 4095;
        acc_mv += mv;
    }
    return acc_mv / BTN_SAMPLE_COUNT;
}
static btn_id_t decode_button_from_mv(int mv)
{
    if (mv > (kButtonLevels_mV[0] + 200)) return BTN_NONE;
    for (int i = 0; i < BTN_COUNT - 1; ++i) if (mv >= sThresh_mV[i]) return (btn_id_t)i;
    return (mv >= 0) ? (btn_id_t)(BTN_COUNT - 1) : BTN_NONE;
}

// ===== I2S helper =====
static void i2s_set_clk_mono_16k(audio_element_handle_t i2s_elem)
{ i2s_stream_set_clk(i2s_elem, E_AUDIO_RATE_HZ, 16, 1); }

// ===== Pipelines =====
static void stop_pipeline(audio_pipeline_handle_t p) {
    if (!p) return;
    audio_pipeline_stop(p);
    audio_pipeline_wait_for_stop(p);
    audio_pipeline_terminate(p);
}

static bool build_pipeline_clean(void)
{
    // reader
    i2s_stream_cfg_t r_cfg = I2S_STREAM_CFG_DEFAULT();
    r_cfg.type = AUDIO_STREAM_READER; r_cfg.task_core = 1; r_cfg.stack_in_ext = true;
    clean_reader = i2s_stream_init(&r_cfg);
    if (!clean_reader) { ESP_LOGE(TAG, "i2s reader init failed"); return false; }
    i2s_set_clk_mono_16k(clean_reader);

    // writer
    i2s_stream_cfg_t w_cfg = I2S_STREAM_CFG_DEFAULT();
    w_cfg.type = AUDIO_STREAM_WRITER; w_cfg.task_core = 1; w_cfg.stack_in_ext = true;
    clean_writer = i2s_stream_init(&w_cfg);
    if (!clean_writer) { ESP_LOGE(TAG, "i2s writer init failed"); audio_element_deinit(clean_reader); clean_reader=NULL; return false; }
    i2s_set_clk_mono_16k(clean_writer);

    // algorithm — TYPE1 "M" (single mic), NS+AGC only
    algorithm_stream_cfg_t a_cfg = ALGORITHM_STREAM_CFG_DEFAULT();
    a_cfg.input_type        = ALGORITHM_STREAM_INPUT_TYPE1; // one input stream
    a_cfg.input_format      = "M";                          // single microphone channel
    a_cfg.sample_rate       = E_AUDIO_RATE_HZ;
    a_cfg.algo_mask         = (ALGORITHM_STREAM_USE_NS | ALGORITHM_STREAM_USE_AGC); // AEC OFF (lighter)
    a_cfg.rec_linear_factor = 1;   // must be > 0 on ADF master
    a_cfg.ref_linear_factor = 1;   // keep > 0 even unused
    a_cfg.stack_in_ext      = true;

    clean_algo = algo_stream_init(&a_cfg);
    if (!clean_algo) {
        ESP_LOGE(TAG, "algorithm_stream init failed — CLEAN disabled.");
        audio_element_deinit(clean_reader); clean_reader = NULL;
        audio_element_deinit(clean_writer); clean_writer = NULL;
        return false;
    }

    audio_pipeline_cfg_t p_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipe_clean = audio_pipeline_init(&p_cfg);
    if (!pipe_clean) {
        ESP_LOGE(TAG, "pipeline init failed");
        audio_element_deinit(clean_algo);   clean_algo   = NULL;
        audio_element_deinit(clean_reader); clean_reader = NULL;
        audio_element_deinit(clean_writer); clean_writer = NULL;
        return false;
    }

    ESP_ERROR_CHECK(audio_pipeline_register(pipe_clean, clean_reader, "reader"));
    ESP_ERROR_CHECK(audio_pipeline_register(pipe_clean, clean_algo,   "algo"));
    ESP_ERROR_CHECK(audio_pipeline_register(pipe_clean, clean_writer, "writer"));
    const char *links[] = {"reader", "algo", "writer"};
    ESP_ERROR_CHECK(audio_pipeline_link(pipe_clean, links, 3));

    clean_writer_rb = audio_element_get_output_ringbuf(clean_writer);
    apply_aec_delay(); // harmless with AEC off
    return true;
}

static bool build_pipeline_pass(void)
{
    i2s_stream_cfg_t r_cfg = I2S_STREAM_CFG_DEFAULT();
    r_cfg.type = AUDIO_STREAM_READER; r_cfg.task_core = 1; r_cfg.stack_in_ext = true;
    pass_reader = i2s_stream_init(&r_cfg);
    if (!pass_reader) { ESP_LOGE(TAG, "pass reader init failed"); return false; }
    i2s_set_clk_mono_16k(pass_reader);

    i2s_stream_cfg_t w_cfg = I2S_STREAM_CFG_DEFAULT();
    w_cfg.type = AUDIO_STREAM_WRITER; w_cfg.task_core = 1; w_cfg.stack_in_ext = true;
    pass_writer = i2s_stream_init(&w_cfg);
    if (!pass_writer) { ESP_LOGE(TAG, "pass writer init failed"); audio_element_deinit(pass_reader); pass_reader=NULL; return false; }
    i2s_set_clk_mono_16k(pass_writer);

    audio_pipeline_cfg_t p_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipe_pass = audio_pipeline_init(&p_cfg);
    if (!pipe_pass) {
        ESP_LOGE(TAG, "pass pipeline init failed");
        audio_element_deinit(pass_reader); pass_reader = NULL;
        audio_element_deinit(pass_writer); pass_writer = NULL;
        return false;
    }
    ESP_ERROR_CHECK(audio_pipeline_register(pipe_pass, pass_reader, "reader"));
    ESP_ERROR_CHECK(audio_pipeline_register(pipe_pass, pass_writer, "writer"));
    const char *links[] = {"reader", "writer"};
    ESP_ERROR_CHECK(audio_pipeline_link(pipe_pass, links, 2));
    return true;
}

static void start_mode(run_mode_t m)
{
    stop_pipeline(pipe_clean);
    stop_pipeline(pipe_pass);

    bool ran = false;
    if (m == MODE_CLEAN && pipe_clean)     { audio_pipeline_run(pipe_clean); ran = true; }
    else if ((m == MODE_PASSTHROUGH || m == MODE_BF_PLACEHOLDER) && pipe_pass) { audio_pipeline_run(pipe_pass); ran = true; }

    if (!ran && pipe_pass) { ESP_LOGW(TAG, "Mode %d unavailable; fallback to PASS", (int)m); audio_pipeline_run(pipe_pass); m = MODE_PASSTHROUGH; }
    leds_show_mode(m);
    ESP_LOGI(TAG, "Mode -> %d (0=pass,1=clean,2=bf)", m);
}

// ===== Buttons =====
static void button_task(void *arg)
{
    int stable_count = 0;
    btn_id_t last = BTN_NONE;

    while (1) {
        int mv = adc_read_avg_mv();
        btn_id_t cur = decode_button_from_mv(mv);

        if (cur == last && cur != BTN_NONE) {
            if (++stable_count >= BTN_STABLE_SAMPLES) {
                ESP_LOGI(TAG, "Button: %s (%d mV)", kButtons[cur], mv);
                switch (cur) {
                    case BTN_VOL_MINUS: hp_volume = (hp_volume >= 5) ? hp_volume - 5 : 0;   apply_volume(); break;
                    case BTN_VOL_PLUS:  hp_volume = (hp_volume <= 95) ? hp_volume + 5 : 100; apply_volume(); break;
                    case BTN_MODE:      run_mode = (run_mode + 1) % MODE_MAX; start_mode(run_mode); break;
                    case BTN_SET:       hp_mute = !hp_mute; apply_volume(); break;
                    case BTN_REC:       aec_delay_ms -= 10; apply_aec_delay(); break;
                    case BTN_PLAY:      aec_delay_ms += 10; apply_aec_delay(); break;
                    default: break;
                }
                // wait release
                do { vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS)); mv = adc_read_avg_mv(); }
                while (decode_button_from_mv(mv) == cur);
                stable_count = 0; last = BTN_NONE;
            }
        } else {
            stable_count = (cur == BTN_NONE) ? 0 : 1;
            last = cur;
        }
        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));
    }
}

// ===== app_main =====
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    leds_init();
    leds_show_mode(run_mode);

    board = audio_board_init();
    AUDIO_NULL_CHECK(TAG, board, return);
    audio_hal_ctrl_codec(board->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
    hp_volume = E_AUDIO_VOL_HP_DEFAULT;
    apply_volume();

    adc_init_buttons();
    xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);

    bool have_clean = build_pipeline_clean();
    if (!have_clean && run_mode == MODE_CLEAN) run_mode = MODE_PASSTHROUGH;
    bool have_pass  = build_pipeline_pass();
    if (!have_pass) { ESP_LOGE(TAG, "PASS pipeline missing; nothing to run!"); while (1) vTaskDelay(pdMS_TO_TICKS(1000)); }

    start_mode(run_mode);

    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}
