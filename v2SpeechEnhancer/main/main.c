// main.c — ESP32-LyraTD-MSC v2.2  |  IDF v5.3.x + ADF master (older audio_hal API)
// - LEDs via IS31FL3216 (standard ADF peripheral)
// - Buttons via esp_adc/adc_oneshot: median + hysteresis + debounce
// - I2S MASTER full-duplex @ 48 kHz / 16-bit
// - Startup tone; PLAY toggles bridge
// - MODE cycles: 0=RAW, 1=ENH (HPF+AGC+Limiter), 2=DSP (Twolf) + post-gain/EQ/limiter
// - In DSP mode, VOL± adjusts Twolf HAL volume (+ small software post-gain trim)
// - SET in DSP mode toggles BRIGHT tilt EQ
//
// Build note: add audio_hal to REQUIRES in main/CMakeLists.txt, e.g.:
//   idf_component_register(SRCS "main.c" INCLUDE_DIRS "." REQUIRES audio_hal esp_peripherals driver esp_driver_i2s)

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "soc/clk_tree_defs.h"

#include "esp_adc/adc_oneshot.h"   // modern ADC API

#include "esp_peripherals.h"
#include "periph_is31fl3216.h"
#include "audio_hal.h"              // Twolf (ZL38063) HAL volume/mute/start

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------- Board pins (LyraTD-MSC v2.2)
#define PIN_I2S_BCLK    GPIO_NUM_5
#define PIN_I2S_LRCLK   GPIO_NUM_25
#define PIN_I2S_DOUT    GPIO_NUM_26   // ESP32 -> DSP SDIN (playback)
#define PIN_I2S_DIN     GPIO_NUM_35   // DSP SDOUT -> ESP32 (capture)

#define PIN_TOP_LDO_EN  GPIO_NUM_21   // Top board LDO (ACTIVE-LOW, hold LOW to enable)
#define PIN_DSP_RESET   GPIO_NUM_19   // ZL38063 reset (active-low) — keep released (HIGH)
#define PIN_PA_EN       GPIO_NUM_22   // Power amp enable (HIGH on)
#define PIN_HP_DET      GPIO_NUM_36   // Headphone detect (informational)

// ---------- Audio / tasks
#define SAMPLE_RATE_HZ  48000
#define TONE_MS         1200

// ---------- Button ladder (ADC1_CH3 = GPIO39) — from schematic
// Set = 0.30V, Play = 0.82V, Rec = 1.34V, Mode = 1.80V, Vol- = 2.27V, Vol+ = 2.88V
#define MV(x)           (x)
#define BTN_B0          MV(150)
#define BTN_B1          MV((300+820)/2)    // 560 mV
#define BTN_B2          MV((820+1340)/2)   // 1080 mV
#define BTN_B3          MV((1340+1800)/2)  // 1570 mV
#define BTN_B4          MV((1800+2270)/2)  // 2035 mV
#define BTN_B5          MV((2270+2880)/2)  // 2575 mV
#define BTN_B6          MV(3300)

#define BTN_HYS_MV      60
#define DEBOUNCE_MEDS   2

// ---------- Types
typedef enum { BTN_NONE=0, BTN_SET, BTN_PLAY, BTN_REC, BTN_MODE, BTN_VOLDN, BTN_VOLUP } btn_t;
typedef enum { MODE_RAW=0, MODE_ENH=1, MODE_DSP=2 } proc_mode_t;

static const char *TAG = "eaudio_main";

// ---------- Globals
static esp_periph_set_handle_t   g_periph_set = NULL;
static esp_periph_handle_t       g_led        = NULL;

static adc_oneshot_unit_handle_t g_adc        = NULL;

static i2s_chan_handle_t g_tx = NULL;
static i2s_chan_handle_t g_rx = NULL;

static audio_hal_handle_t        g_hal        = NULL;  // Twolf HAL
// NOTE: older ADF requires passing a function table for the specific codec:
extern audio_hal_func_t          AUDIO_CODEC_ZL38063_DEFAULT_HANDLE;

static volatile bool        g_bridge_on   = false;     // PLAY toggles this
static volatile proc_mode_t g_mode        = MODE_ENH;  // start Enhanced
static volatile int         g_volume_pct  = 80;        // tone/ENH target

// DSP-mode post-processing controls
static volatile float       g_dsp_gain_db   = 6.0f;    // post-gain after DSP
static volatile bool        g_dsp_bright_on = true;    // tilt EQ on/off

// =====================================================================
//                        LEDS
// =====================================================================
static void leds_init(void)
{
    esp_periph_config_t pcfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    g_periph_set = esp_periph_set_init(&pcfg);

    periph_is31fl3216_cfg_t cfg = (periph_is31fl3216_cfg_t){0};
    cfg.state = IS31FL3216_STATE_ON;
    for (int i=0; i<14; ++i) { cfg.is31fl3216_pattern |= (1u<<i); cfg.duty[i] = 2; }

    g_led = periph_is31fl3216_init(&cfg);
    if (!g_led) { ESP_LOGW(TAG, "IS31FL3216 init failed"); return; }
    esp_periph_start(g_periph_set, g_led);
    periph_is31fl3216_set_state(g_led, IS31FL3216_STATE_ON);
    periph_is31fl3216_set_blink_pattern(g_led, cfg.is31fl3216_pattern);
    ESP_LOGI(TAG, "IS31FL3216 started (baseline faint ON ch0..13).");
}

static inline void leds_flash_all(uint8_t duty)
{
    for (int i=0;i<14;++i) periph_is31fl3216_set_duty(g_led, i, duty);
}

static void leds_task(void *arg)
{
    int pos = 0;
    while (1) {
        for (int ch=0; ch<14; ++ch) periph_is31fl3216_set_duty(g_led, ch, 2);
        periph_is31fl3216_set_duty(g_led, pos, 60);
        periph_is31fl3216_set_blink_pattern(g_led, (1u<<pos));
        pos = (pos+1)%14;
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

// =====================================================================
//                        POWER RAILS
// =====================================================================
static void rails_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<PIN_TOP_LDO_EN) | (1ULL<<PIN_DSP_RESET) | (1ULL<<PIN_PA_EN),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io);

    gpio_set_level(PIN_TOP_LDO_EN, 0);  // active-LOW -> enable
    gpio_set_level(PIN_DSP_RESET,   1); // release reset
    gpio_set_level(PIN_PA_EN,       1); // PA on

    ESP_LOGI(TAG, "Top LDO ON (GPIO21 LOW), DSP reset released, PA enabled.");

    gpio_config_t in = { .pin_bit_mask=(1ULL<<PIN_HP_DET), .mode=GPIO_MODE_INPUT };
    gpio_config(&in);
    ESP_LOGI(TAG, "Headphone detect: %d", gpio_get_level(PIN_HP_DET));
}

// =====================================================================
//                        I2S MASTER Full-Duplex
// =====================================================================
static esp_err_t i2s_open_master(void)
{
    esp_err_t err;

    i2s_chan_config_t cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    cfg.auto_clear = true;
    err = i2s_new_channel(&cfg, &g_tx, &g_rx);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2s_new_channel err=%d", err); return err; }

    i2s_std_config_t std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCLK,
            .ws   = PIN_I2S_LRCLK,
            .dout = PIN_I2S_DOUT,
            .din  = PIN_I2S_DIN,
            .invert_flags = { .mclk_inv=false, .bclk_inv=false, .ws_inv=false },
        },
    };
    err = i2s_channel_init_std_mode(g_tx, &std);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2s tx init err=%d", err); return err; }
    err = i2s_channel_init_std_mode(g_rx, &std);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2s rx init err=%d", err); return err; }

    ESP_ERROR_CHECK(i2s_channel_enable(g_tx));
    ESP_ERROR_CHECK(i2s_channel_enable(g_rx));
    ESP_LOGI(TAG, "I2S0 MASTER @ %d Hz, 16-bit stereo (BCLK=%d, LRCLK=%d, DOUT=%d, DIN=%d)",
             SAMPLE_RATE_HZ, PIN_I2S_BCLK, PIN_I2S_LRCLK, PIN_I2S_DOUT, PIN_I2S_DIN);
    return ESP_OK;
}

// =====================================================================
//                        BUTTONS (ADC oneshot)
// =====================================================================
static btn_t decode_button_mv_hyst(int mv, btn_t prev_bin)
{
    int b0=BTN_B0, b1=BTN_B1, b2=BTN_B2, b3=BTN_B3, b4=BTN_B4, b5=BTN_B5, b6=BTN_B6;

    switch (prev_bin) {
        case BTN_SET:   b1 += BTN_HYS_MV; break;
        case BTN_PLAY:  b1 -= BTN_HYS_MV; b2 += BTN_HYS_MV; break;
        case BTN_REC:   b2 -= BTN_HYS_MV; b3 += BTN_HYS_MV; break;
        case BTN_MODE:  b3 -= BTN_HYS_MV; b4 += BTN_HYS_MV; break;
        case BTN_VOLDN: b4 -= BTN_HYS_MV; b5 += BTN_HYS_MV; break;
        case BTN_VOLUP: b5 -= BTN_HYS_MV; break;
        default: break;
    }

    if (mv < b0) return BTN_NONE;
    if (mv < b1) return BTN_SET;
    if (mv < b2) return BTN_PLAY;
    if (mv < b3) return BTN_REC;
    if (mv < b4) return BTN_MODE;
    if (mv < b5) return BTN_VOLDN;
    if (mv < b6) return BTN_VOLUP;
    return BTN_NONE;
}

static void buttons_init(void)
{
    adc_oneshot_unit_init_cfg_t u = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&u, &g_adc);
    adc_oneshot_chan_cfg_t ch = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12 };
    adc_oneshot_config_channel(g_adc, ADC_CHANNEL_3, &ch); // GPIO39
}

static inline void dsp_hal_volume_step(int delta)
{
    if (!g_hal) return;
    int v = 80;
    audio_hal_get_volume(g_hal, &v);
    v += delta;
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    audio_hal_set_volume(g_hal, v);
    ESP_LOGI(TAG, "Twolf HAL volume -> %d", v);
}

static void buttons_task(void *arg)
{
    btn_t prev_bin = BTN_NONE;     // previous decoded bin (for hysteresis)
    btn_t last_sent = BTN_NONE;    // last button we emitted
    int   same_count = 0;

    while (1) {
        // Median of 7 samples
        int raw[7];
        for (int i=0;i<7;++i){ adc_oneshot_read(g_adc, ADC_CHANNEL_3, &raw[i]); vTaskDelay(pdMS_TO_TICKS(2)); }
        for (int i=0;i<7;++i) for (int j=i+1;j<7;++j) if (raw[j]<raw[i]){ int t=raw[i]; raw[i]=raw[j]; raw[j]=t; }
        int mv = (raw[3] * 3300) / 4095; // approx mV

        btn_t cur = decode_button_mv_hyst(mv, prev_bin);

        if (cur == prev_bin) same_count++;
        else                 same_count = 1;

        if (same_count >= DEBOUNCE_MEDS && cur != BTN_NONE && cur != last_sent) {
            const char *name = (cur==BTN_SET?"SET":cur==BTN_PLAY?"PLAY":cur==BTN_REC?"REC":
                                cur==BTN_MODE?"MODE":cur==BTN_VOLDN?"VOL-":"VOL+");
            ESP_LOGI(TAG, "Button: %s (~%dmV)", name, mv);

            leds_flash_all(90);
            vTaskDelay(pdMS_TO_TICKS(60));
            leds_flash_all(2);

            if (cur == BTN_PLAY) {
                g_bridge_on = !g_bridge_on;
                ESP_LOGW(TAG, "Bridge %s", g_bridge_on ? "ON" : "OFF");
            } else if (cur == BTN_MODE) {
                g_mode = (g_mode == MODE_RAW) ? MODE_ENH : (g_mode == MODE_ENH ? MODE_DSP : MODE_RAW);
                const char *mname = (g_mode==MODE_RAW?"RAW": g_mode==MODE_ENH?"ENHANCED":"DSP");
                ESP_LOGW(TAG, "Mode -> %s", mname);
                int blinks = (g_mode==MODE_RAW)?1:(g_mode==MODE_ENH?2:3);
                for (int i=0;i<blinks;++i){ leds_flash_all(120); vTaskDelay(pdMS_TO_TICKS(80)); leds_flash_all(2); vTaskDelay(pdMS_TO_TICKS(100)); }
            } else if (cur == BTN_VOLDN) {
                if (g_mode == MODE_DSP) {
                    dsp_hal_volume_step(-3);                 // DSP volume (clean)
                    g_dsp_gain_db -= 0.5f;                   // fine post-gain
                    if (g_dsp_gain_db < -6.0f) g_dsp_gain_db = -6.0f;
                    ESP_LOGI(TAG, "DSP post-gain: %.1f dB", g_dsp_gain_db);
                } else {
                    g_volume_pct = (g_volume_pct>=5)? g_volume_pct-5 : 0;
                    ESP_LOGI(TAG, "ENH target Volume=%d%%", g_volume_pct);
                }
            } else if (cur == BTN_VOLUP) {
                if (g_mode == MODE_DSP) {
                    dsp_hal_volume_step(+3);
                    g_dsp_gain_db += 0.5f;
                    if (g_dsp_gain_db > 18.0f) g_dsp_gain_db = 18.0f;
                    ESP_LOGI(TAG, "DSP post-gain: %.1f dB", g_dsp_gain_db);
                } else {
                    g_volume_pct = (g_volume_pct<=95)? g_volume_pct+5 : 100;
                    ESP_LOGI(TAG, "ENH target Volume=%d%%", g_volume_pct);
                }
            } else if (cur == BTN_SET) {
                if (g_mode == MODE_DSP) {
                    g_dsp_bright_on = !g_dsp_bright_on;
                    ESP_LOGW(TAG, "DSP BRIGHT: %s", g_dsp_bright_on ? "ON" : "OFF");
                    for (int i=0;i<3;++i){ leds_flash_all(120); vTaskDelay(pdMS_TO_TICKS(60)); leds_flash_all(2); vTaskDelay(pdMS_TO_TICKS(80)); }
                }
            }

            last_sent = cur;
        }

        prev_bin = cur;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// =====================================================================
//                        Simple Enhance (HPF + AGC + limiter)
// =====================================================================
typedef struct {
    float b0,b1,b2,a1,a2;   // biquad
    float z1L,z2L,z1R,z2R;
    float agc_gain;
} proc_t;

static proc_t g_proc;

static void proc_init(proc_t *p)
{
    memset(p, 0, sizeof(*p));
    const float fs=48000.f, fc=120.f, Q=0.7071f;
    const float w0 = 2.f*(float)M_PI*fc/fs;
    const float c = cosf(w0), s = sinf(w0), alpha = s/(2.f*Q);
    const float a0 = 1.f + alpha;

    const float b0 =  (1.f + c)*0.5f;
    const float b1 = -(1.f + c);
    const float b2 =  (1.f + c)*0.5f;
    const float a1 =  -2.f * c;
    const float a2 =   1.f - alpha;

    p->b0 = b0/a0; p->b1 = b1/a0; p->b2 = b2/a0;
    p->a1 = a1/a0; p->a2 = a2/a0;
    p->agc_gain = 1.0f;
}

static inline float soft_clip(float x)
{
    const float t = 0.95f;
    if (x >  t) return t + (x - t) * 0.1f;
    if (x < -t) return -t + (x + t) * 0.1f;
    return x;
}

static inline int16_t clamp16_from_float(float v)
{
    int32_t s = (int32_t)(v * 32767.f);
    if (s >  32767) s =  32767;
    if (s < -32768) s = -32768;
    return (int16_t)s;
}

static void process_block(proc_t *p, int16_t *buf, size_t frames, float target_rms)
{
    double acc = 0.0;
    for (size_t n=0;n<frames;++n) {
        float xL = (float)buf[2*n+0] * (1.0f/32768.f);
        float xR = (float)buf[2*n+1] * (1.0f/32768.f);
        float yL = p->b0*xL + p->z1L;
        p->z1L = p->b1*xL + p->z2L - p->a1*yL;
        p->z2L = p->b2*xL           - p->a2*yL;
        float yR = p->b0*xR + p->z1R;
        p->z1R = p->b1*xR + p->z2R - p->a1*yR;
        p->z2R = p->b2*xR           - p->a2*yR;
        acc += (double)yL*yL + (double)yR*yR;
        buf[2*n+0] = clamp16_from_float(yL);
        buf[2*n+1] = clamp16_from_float(yR);
    }

    float rms = sqrtf((float)(acc / (frames*2)));
    float g_des = (rms > 1e-6f) ? (target_rms / rms) : 4.0f;
    if (g_des < 0.5f) g_des = 0.5f;
    if (g_des > 8.0f) g_des = 8.0f;
    const float a_up = 0.30f, a_dn = 0.08f;
    float alpha = (g_des > p->agc_gain) ? a_up : a_dn;
    p->agc_gain = p->agc_gain + alpha * (g_des - p->agc_gain);

    for (size_t n=0;n<frames;++n) {
        float yL = (float)buf[2*n+0] * (1.0f/32768.f) * p->agc_gain;
        float yR = (float)buf[2*n+1] * (1.0f/32768.f) * p->agc_gain;
        yL = soft_clip(yL);
        yR = soft_clip(yR);
        buf[2*n+0] = clamp16_from_float(yL);
        buf[2*n+1] = clamp16_from_float(yR);
    }
}

// =====================================================================
//                        DSP post-processing (gain + bright tilt + limiter)
// =====================================================================
typedef struct { float lpL, lpR; } dsp_post_t;
static dsp_post_t g_post;

static inline void dsp_post_init(void) { memset(&g_post, 0, sizeof(g_post)); }

static inline void dsp_post_process(int16_t *buf, size_t frames, float gain_db, bool bright_on)
{
    const float lin = powf(10.0f, gain_db / 20.0f);
    const float alpha = 1.0f - expf(-2.0f * (float)M_PI * 2000.0f / (float)SAMPLE_RATE_HZ);
    const float bright_k = bright_on ? 0.35f : 0.0f;

    for (size_t n=0; n<frames; ++n) {
        float xL = (float)buf[2*n+0] * (1.0f/32768.0f);
        g_post.lpL += alpha * (xL - g_post.lpL);
        float yL = (xL + bright_k * (xL - g_post.lpL)) * lin;
        yL = soft_clip(yL);
        buf[2*n+0] = clamp16_from_float(yL);

        float xR = (float)buf[2*n+1] * (1.0f/32768.0f);
        g_post.lpR += alpha * (xR - g_post.lpR);
        float yR = (xR + bright_k * (xR - g_post.lpR)) * lin;
        yR = soft_clip(yR);
        buf[2*n+1] = clamp16_from_float(yR);
    }
}

// =====================================================================
//                        Startup Tone (smooth)
// =====================================================================
static void tone_task(void *arg)
{
    const float freq = 880.0f;
    const int   N    = 256;
    const int   total_frames = (SAMPLE_RATE_HZ * TONE_MS) / 1000;
    int16_t *buf = (int16_t*)heap_caps_malloc(N*2*sizeof(int16_t), MALLOC_CAP_8BIT);
    if (!buf){ vTaskDelete(NULL); return; }

    static float phase = 0.f;
    const float dphi = 2.f*(float)M_PI*freq / (float)SAMPLE_RATE_HZ;
    const int fade_frames = (SAMPLE_RATE_HZ * 12) / 1000;

    int generated = 0;
    ESP_LOGI(TAG, "Startup tone ~%d ms", TONE_MS);
    while (generated < total_frames) {
        int frames = (total_frames - generated) > N ? N : (total_frames - generated);
        for (int i=0;i<frames;++i) {
            float env = 1.0f;
            if (generated < fade_frames) env = (float)generated / (float)fade_frames;
            else if (generated > total_frames - fade_frames) env = (float)(total_frames - generated) / (float)fade_frames;

            float s = sinf(phase) * env;
            phase += dphi; if (phase > 2.f*(float)M_PI) phase -= 2.f*(float)M_PI;

            float amp = (g_volume_pct/100.0f) * 0.95f;
            int16_t v = clamp16_from_float(s * amp);
            buf[2*i+0] = v; buf[2*i+1] = v;
            generated++;
        }
        size_t wrote=0;
        i2s_channel_write(g_tx, buf, frames*2*sizeof(int16_t), &wrote, pdMS_TO_TICKS(50));
    }
    free(buf);
    vTaskDelete(NULL);
}

// =====================================================================
//                        Bridge (RAW / ENH / DSP)
// =====================================================================
static void bridge_task(void *arg)
{
    const size_t N = 256; // frames per chunk
    int16_t *buf = (int16_t*)heap_caps_malloc(N*2*sizeof(int16_t), MALLOC_CAP_8BIT);
    if (!buf){ vTaskDelete(NULL); return; }

    proc_init(&g_proc);
    dsp_post_init();
    ESP_LOGI(TAG, "Bridge ready. PLAY toggles audio; MODE cycles RAW/ENH/DSP. SET toggles BRIGHT in DSP.");

    while (1) {
        if (!g_bridge_on) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

        size_t got=0, put=0;
        if (i2s_channel_read(g_rx, buf, N*2*sizeof(int16_t), &got, pdMS_TO_TICKS(50)) == ESP_OK && got) {
            if (g_mode == MODE_ENH) {
                float target = 0.22f + (g_volume_pct/100.0f) * 0.23f; // ~ -13..-7 dBFS
                process_block(&g_proc, buf, got/(2*sizeof(int16_t)), target);
            } else if (g_mode == MODE_DSP) {
                dsp_post_process(buf, got/(2*sizeof(int16_t)), g_dsp_gain_db, g_dsp_bright_on);
            }
            (void)i2s_channel_write(g_tx, buf, got, &put, pdMS_TO_TICKS(50));
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// =====================================================================
//                        Optional control image loader (stub)
// =====================================================================
extern const uint8_t twolf_control_srec_start[] asm("_binary_twolf_control_srec_start");
extern const uint8_t twolf_control_srec_end[]   asm("_binary_twolf_control_srec_end");

#define TWOLF_I2C_PORT      I2C_NUM_0
#define TWOLF_I2C_SDA       GPIO_NUM_18
#define TWOLF_I2C_SCL       GPIO_NUM_23
#define TWOLF_I2C_HZ        400000
#define TWOLF_WRITE_TIMEOUT_MS 20
#define TWOLF_MAX_BURST     16

static bool                      s_twolf_i2c_ready = false;
static uint8_t                   s_twolf_i2c_addr  = 0;   // 7-bit address
static i2c_master_bus_handle_t   s_twolf_bus       = NULL;
static i2c_master_dev_handle_t   s_twolf_dev       = NULL;

static inline int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

static bool parse_hex_byte(const char *s, uint8_t *out)
{
    int hi = hex_nibble(s[0]);
    int lo = hex_nibble(s[1]);
    if (hi < 0 || lo < 0) return false;
    *out = (uint8_t)((hi << 4) | lo);
    return true;
}

static bool parse_hex_word(const char *s, uint16_t *out)
{
    uint8_t hi, lo;
    if (!parse_hex_byte(s, &hi) || !parse_hex_byte(s + 2, &lo)) return false;
    *out = (uint16_t)((hi << 8) | lo);
    return true;
}

static esp_err_t twolf_i2c_init_once(void)
{
    if (s_twolf_i2c_ready) {
        return ESP_OK;
    }

    i2c_master_bus_config_t cfg = {
        .i2c_port = TWOLF_I2C_PORT,
        .sda_io_num = TWOLF_I2C_SDA,
        .scl_io_num = TWOLF_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    esp_err_t err = i2c_new_master_bus(&cfg, &s_twolf_bus);
    if (err == ESP_OK) {
        s_twolf_i2c_ready = true;
    }
    return err;
}

static esp_err_t twolf_send_words(const uint16_t *words, size_t count)
{
    if (!s_twolf_dev || !count) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[2 * (TWOLF_MAX_BURST + 1)];
    if (count > (TWOLF_MAX_BURST + 1)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < count; ++i) {
        buf[2 * i + 0] = (uint8_t)(words[i] >> 8);
        buf[2 * i + 1] = (uint8_t)(words[i] & 0xFF);
    }
    return i2c_master_transmit(s_twolf_dev, buf, count * 2, TWOLF_WRITE_TIMEOUT_MS);
}

static esp_err_t twolf_send_command(uint16_t cmd)
{
    return twolf_send_words(&cmd, 1);
}

static void twolf_release_device(void)
{
    if (s_twolf_dev) {
        i2c_master_bus_rm_device(s_twolf_dev);
        s_twolf_dev = NULL;
    }
    s_twolf_i2c_addr = 0;
}

static esp_err_t twolf_create_device(uint8_t addr)
{
    if (!s_twolf_bus) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_twolf_dev && s_twolf_i2c_addr == addr) {
        return ESP_OK;
    }

    if (s_twolf_dev) {
        twolf_release_device();
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = TWOLF_I2C_HZ,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };
    esp_err_t err = i2c_master_bus_add_device(s_twolf_bus, &dev_cfg, &s_twolf_dev);
    if (err == ESP_OK) {
        s_twolf_i2c_addr = addr;
    }
    return err;
}

static esp_err_t twolf_hbi_write(uint16_t addr, const uint16_t *vals, size_t words)
{
    if (!words) {
        return ESP_OK;
    }

    uint8_t page = (uint8_t)(addr >> 8);
    uint8_t offset = (uint8_t)((addr & 0xFFu) / 2u);
    esp_err_t err;

    if (page == 0) {
        uint16_t cmd = (uint16_t)(0x8000u | ((uint16_t)offset << 8) | (uint16_t)(words - 1));
        uint16_t temp[TWOLF_MAX_BURST + 1];
        if (words > TWOLF_MAX_BURST) {
            return ESP_ERR_INVALID_ARG;
        }
        temp[0] = cmd | 0x0080u;  // direct write
        memcpy(&temp[1], vals, words * sizeof(uint16_t));
        return twolf_send_words(temp, words + 1);
    }

    uint16_t command_buf[2];
    if (page != 0xFF) {
        page = (uint8_t)(page - 1);
    }
    command_buf[0] = (uint16_t)(0xFE00u | page);  // select page
    uint16_t write_cmd = (uint16_t)(((uint16_t)offset << 8) | (uint16_t)(words - 1));
    command_buf[1] = (uint16_t)(write_cmd | 0x0080u);

    if (words > TWOLF_MAX_BURST) {
        return ESP_ERR_INVALID_ARG;
    }

    err = twolf_send_words(command_buf, 1);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t payload[TWOLF_MAX_BURST + 1];
    payload[0] = command_buf[1];
    memcpy(&payload[1], vals, words * sizeof(uint16_t));
    return twolf_send_words(payload, words + 1);
}

static esp_err_t twolf_hbi_read(uint16_t addr, uint16_t *vals, size_t words)
{
    if (!words) {
        return ESP_OK;
    }

    if (!s_twolf_dev) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t page = (uint8_t)(addr >> 8);
    uint8_t offset = (uint8_t)((addr & 0xFFu) / 2u);
    uint16_t cmd;
    esp_err_t err;

    if (page == 0) {
        cmd = (uint16_t)(0x8000u | ((uint16_t)offset << 8) | (uint16_t)(words - 1));
    } else {
        if (page != 0xFF) {
            page = (uint8_t)(page - 1);
        }
        err = twolf_send_command((uint16_t)(0xFE00u | page));
        if (err != ESP_OK) {
            return err;
        }
        cmd = (uint16_t)(((uint16_t)offset << 8) | (uint16_t)(words - 1));
    }

    uint8_t cmd_bytes[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    size_t read_len = words * 2;
    uint8_t data[2 * TWOLF_MAX_BURST];
    if (words > TWOLF_MAX_BURST) {
        return ESP_ERR_INVALID_ARG;
    }

    err = i2c_master_transmit_receive(s_twolf_dev, cmd_bytes, sizeof(cmd_bytes),
                                      data, read_len, TWOLF_WRITE_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    for (size_t i = 0; i < words; ++i) {
        vals[i] = (uint16_t)((data[2 * i] << 8) | data[2 * i + 1]);
    }
    return ESP_OK;
}

static esp_err_t twolf_detect_address(void)
{
    if (s_twolf_dev) {
        return ESP_OK;
    }
    static const uint8_t candidates[] = { 0x45, 0x52 };
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        esp_err_t err = twolf_create_device(candidates[i]);
        if (err != ESP_OK) {
            continue;
        }
        err = twolf_send_command(0xFFFFu);  // HBI_NO_OP
        if (err == ESP_OK) {
            return ESP_OK;
        }
        twolf_release_device();
    }
    return ESP_ERR_NOT_FOUND;
}

static inline void twolf_try_load_control_image(void)
{
    if (twolf_i2c_init_once() != ESP_OK) {
        ESP_LOGW(TAG, "Twolf I2C init failed; skipping control image");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    if (twolf_detect_address() != ESP_OK) {
        ESP_LOGW(TAG, "Twolf I2C address not found; control image not loaded");
        return;
    }

    size_t len = (size_t)(twolf_control_srec_end - twolf_control_srec_start);
    char *text = malloc(len + 1);
    if (!text) {
        ESP_LOGW(TAG, "Out of memory parsing Twolf control image");
        return;
    }
    memcpy(text, twolf_control_srec_start, len);
    text[len] = '\0';

    uint16_t batch_addr = 0;
    uint16_t batch_vals[TWOLF_MAX_BURST];
    size_t   batch_count = 0;
    uint16_t prev_addr = 0;
    bool     have_batch = false;
    size_t   records = 0;
    size_t   failures = 0;

    char *saveptr = NULL;
    for (char *line = strtok_r(text, "\r\n", &saveptr);
         line != NULL;
         line = strtok_r(NULL, "\r\n", &saveptr)) {
        while (*line == ' ' || *line == '\t') { ++line; }
        if (*line == '\0' || *line == ';') {
            continue;
        }
        if (!(line[0] == 'S' && line[1] == '1')) {
            continue;
        }

        uint8_t count;
        uint16_t addr;
        uint16_t value;
        uint8_t checksum;

        if (!parse_hex_byte(line + 2, &count) ||
            !parse_hex_word(line + 4, &addr) ||
            !parse_hex_word(line + 8, &value) ||
            !parse_hex_byte(line + 12, &checksum)) {
            ESP_LOGW(TAG, "Malformed S1 record: %s", line);
            continue;
        }

        uint32_t sum = count + ((addr >> 8) & 0xFF) + (addr & 0xFF) + ((value >> 8) & 0xFF) + (value & 0xFF);
        uint8_t expected = (uint8_t)(~sum);
        if (checksum != expected) {
            ESP_LOGW(TAG, "Checksum mismatch for 0x%04x", addr);
            continue;
        }

        if (!have_batch) {
            batch_addr = addr;
            batch_vals[0] = value;
            batch_count = 1;
            prev_addr = addr;
            have_batch = true;
        } else if ((uint16_t)(prev_addr + 2) == addr && batch_count < TWOLF_MAX_BURST) {
            batch_vals[batch_count++] = value;
            prev_addr = addr;
        } else {
            esp_err_t err = twolf_hbi_write(batch_addr, batch_vals, batch_count);
            if (err != ESP_OK) {
                ++failures;
                ESP_LOGW(TAG, "Twolf write 0x%04x (%u words) failed: %d", batch_addr, (unsigned)batch_count, err);
            }
            records += batch_count;
            batch_addr = addr;
            batch_vals[0] = value;
            batch_count = 1;
            prev_addr = addr;
        }
    }

    if (have_batch) {
        esp_err_t err = twolf_hbi_write(batch_addr, batch_vals, batch_count);
        if (err != ESP_OK) {
            ++failures;
            ESP_LOGW(TAG, "Twolf write 0x%04x (%u words) failed: %d", batch_addr, (unsigned)batch_count, err);
        }
        records += batch_count;
    }

    free(text);

    if (records) {
        ESP_LOGI(TAG, "Twolf control image applied (%u registers, %u failures)", (unsigned)records, (unsigned)failures);
    }

    typedef struct {
        uint16_t    addr;
        uint16_t    expected;
        const char *desc;
    } twolf_verify_reg_t;

    static const twolf_verify_reg_t verify_regs[] = {
        { 0x0282, 0x0002, "DMIC slot 0 source (MIC2)" },
        { 0x0284, 0x0004, "DMIC slot 1 source (MIC4)" },
        { 0x0286, 0x0006, "DMIC slot 2 source (MIC6)" },
        { 0x0328, 0x3040, "Beamformer routed to downstream tap" },
        { 0x032A, 0x010C, "Downstream tap feeding limiter" },
        { 0x0338, 0x4801, "Headphone DAC crosspoint enabled" },
        { 0x03A4, 0x0002, "PLL M1 (48 kHz base)" },
        { 0x03A6, 0x0064, "PLL M2 (48 kHz base)" },
        { 0x03A8, 0x01F4, "PLL fractional divider (48 kHz)" },
        { 0x03B8, 0x00C2, "I2S SDOUT trim" },
    };

    bool config_ok = true;
    for (size_t i = 0; i < sizeof(verify_regs) / sizeof(verify_regs[0]); ++i) {
        uint16_t value = 0;
        const twolf_verify_reg_t *reg = &verify_regs[i];
        if (twolf_hbi_read(reg->addr, &value, 1) != ESP_OK) {
            ESP_LOGW(TAG, "Twolf readback failed for 0x%04x (%s)", reg->addr, reg->desc);
            config_ok = false;
            continue;
        }
        if (value != reg->expected) {
            ESP_LOGW(TAG, "Twolf %s mismatch: 0x%04x (expected 0x%04x)",
                     reg->desc, value, reg->expected);
            config_ok = false;
        } else {
            ESP_LOGI(TAG, "Twolf %s = 0x%04x", reg->desc, value);
        }
    }

    if (config_ok) {
        ESP_LOGI(TAG, "Twolf control image verified: DMIC beamformer, downstream path, and PLL configured for 48 kHz playback.");
    } else {
        ESP_LOGW(TAG, "Twolf control image verification found mismatches; see logs above.");
    }
}
// =====================================================================
//                        app_main
// =====================================================================
void app_main(void)
{
    ESP_LOGI(TAG, "Application start");

    leds_init();
    xTaskCreate(leds_task, "leds", 2048, NULL, 3, NULL);

    rails_init();
    twolf_try_load_control_image();
    ESP_ERROR_CHECK(i2s_open_master());

    // ---- Twolf (ZL38063) HAL bring-up for older audio_hal API ----
    audio_hal_codec_config_t hal_cfg = {0};
    hal_cfg.codec_mode = AUDIO_HAL_CODEC_MODE_BOTH;
    hal_cfg.adc_input  = AUDIO_HAL_ADC_INPUT_ALL;
    hal_cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_ALL;
    // Twolf is I2S SLAVE; ESP32 is MASTER @ 48k/16
    hal_cfg.i2s_iface.mode    = AUDIO_HAL_MODE_SLAVE;
    hal_cfg.i2s_iface.fmt     = AUDIO_HAL_I2S_NORMAL;
    hal_cfg.i2s_iface.samples = AUDIO_HAL_48K_SAMPLES;
    hal_cfg.i2s_iface.bits    = AUDIO_HAL_BIT_LENGTH_16BITS;

    g_hal = audio_hal_init(&hal_cfg, &AUDIO_CODEC_ZL38063_DEFAULT_HANDLE);
    if (g_hal) {
        audio_hal_ctrl_codec(g_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
        ESP_LOGI(TAG, "Twolf HAL started (codec running, waiting for control image)");

    } else {
        ESP_LOGW(TAG, "Twolf HAL init failed (volume control unavailable)");
    }

    twolf_try_load_control_image();

    if (g_hal) {
        audio_hal_set_volume(g_hal, 90);   // loud, clean starting point
        audio_hal_set_mute(g_hal, false);
        ESP_LOGI(TAG, "Twolf HAL volume restored after control image load");
    }

    buttons_init();
    xTaskCreate(buttons_task, "buttons", 3072, NULL, 4, NULL);

    xTaskCreate(tone_task,   "tone",   3072, NULL, 5, NULL);
    xTaskCreate(bridge_task, "bridge", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Ready. PLAY=bridge, MODE cycles RAW/ENH/DSP; VOL± controls ENH/DSP; SET toggles BRIGHT in DSP.");
}
