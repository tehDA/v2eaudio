#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IS31FL3216_CUSTOM_ADDR 0x74  // AD=0 -> 0b1110100x (0x74 write / 0x75 read)

typedef struct {
    i2c_port_t i2c_port;
    gpio_num_t sda_io_num;
    gpio_num_t scl_io_num;
    uint32_t clk_speed_hz;
    uint8_t i2c_address;   // optional: override default (0x74)
} is31fl3216_custom_config_t;

/**
 * @brief Initialize IS31FL3216 LED driver
 */
esp_err_t is31fl3216_custom_init(const is31fl3216_custom_config_t *config);

/**
 * @brief Reset the device (shutdown cycle)
 */
esp_err_t is31fl3216_custom_reset(void);

/**
 * @brief Set brightness of a single LED channel (0–255)
 *
 * @param channel LED channel index (0–15)
 * @param brightness Brightness (0 = off, 255 = max)
 */
esp_err_t is31fl3216_custom_set_led(uint8_t channel, uint8_t brightness);

/**
 * @brief Update all LEDs at once
 *
 * @param values Pointer to array of 16 brightness values (0–255)
 */
esp_err_t is31fl3216_custom_set_all(const uint8_t *values);

/**
 * @brief Deinitialize driver (only deletes I²C driver if we installed it)
 */
esp_err_t is31fl3216_custom_deinit(void);

#ifdef __cplusplus
}
#endif
