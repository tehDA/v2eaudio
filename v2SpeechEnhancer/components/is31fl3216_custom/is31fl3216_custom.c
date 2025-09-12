#include "is31fl3216_custom.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_check.h"
#include <string.h>   // for memcpy
#include <inttypes.h> // for PRIu32 in logging

#define TAG "IS31FL3216C"

// Registers (from IS31FL3216 datasheet)
#define REG_SHUTDOWN   0x00
#define REG_PWM_BASE   0x01  // 0x01–0x10 (16 channels PWM)
#define REG_UPDATE     0x16

static i2c_port_t s_i2c_port = I2C_NUM_0;
static uint8_t    s_dev_addr = IS31FL3216_CUSTOM_ADDR;
static bool       s_we_installed_driver = false;

// Internal I2C helpers
static esp_err_t write_register(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = { reg, val };
    return i2c_master_write_to_device(s_i2c_port, s_dev_addr, data, sizeof(data), pdMS_TO_TICKS(100));
}

static esp_err_t write_registers(uint8_t start_reg, const uint8_t *values, size_t len)
{
    if (!values || len > 16) return ESP_ERR_INVALID_ARG;
    uint8_t buf[1 + 16];
    buf[0] = start_reg;
    memcpy(&buf[1], values, len);
    return i2c_master_write_to_device(s_i2c_port, s_dev_addr, buf, 1 + len, pdMS_TO_TICKS(100));
}

// Public API
esp_err_t is31fl3216_custom_init(const is31fl3216_custom_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    s_i2c_port = config->i2c_port;
    s_dev_addr = config->i2c_address ? config->i2c_address : IS31FL3216_CUSTOM_ADDR;

    // Configure pins/clock (idempotent)
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_io_num,
        .scl_io_num = config->scl_io_num,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = config->clk_speed_hz,
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(s_i2c_port, &conf), TAG, "i2c_param_config failed");

    // Try to install the driver; tolerate if already installed
    esp_err_t r = i2c_driver_install(s_i2c_port, conf.mode, 0, 0, 0);
    if (r == ESP_OK) {
        s_we_installed_driver = true;
    } else if (r == ESP_ERR_INVALID_STATE || r == ESP_FAIL) {
        ESP_LOGW(TAG, "I2C driver already installed on port %d; reusing existing driver", s_i2c_port);
        s_we_installed_driver = false;
    } else {
        return r;
    }

    // Wake device
    ESP_RETURN_ON_ERROR(write_register(REG_SHUTDOWN, 0x01), TAG, "Failed to exit shutdown");

    ESP_LOGI(TAG, "IS31FL3216 initialized (addr=0x%02X, SDA=%d, SCL=%d, freq=%" PRIu32 "Hz)",
             s_dev_addr, (int)config->sda_io_num, (int)config->scl_io_num, config->clk_speed_hz);

    return ESP_OK;
}

esp_err_t is31fl3216_custom_reset(void)
{
    ESP_RETURN_ON_ERROR(write_register(REG_SHUTDOWN, 0x00), TAG, "Failed shutdown");
    vTaskDelay(pdMS_TO_TICKS(10));
    return write_register(REG_SHUTDOWN, 0x01);
}

esp_err_t is31fl3216_custom_set_led(uint8_t channel, uint8_t brightness)
{
    if (channel >= 16) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(write_register(REG_PWM_BASE + channel, brightness), TAG, "PWM write failed");
    return write_register(REG_UPDATE, 0x00);
}

esp_err_t is31fl3216_custom_set_all(const uint8_t *values)
{
    ESP_RETURN_ON_ERROR(write_registers(REG_PWM_BASE, values, 16), TAG, "Bulk PWM write failed");
    return write_register(REG_UPDATE, 0x00);
}

esp_err_t is31fl3216_custom_deinit(void)
{
    if (s_we_installed_driver) {
        i2c_driver_delete(s_i2c_port);
        s_we_installed_driver = false;
    }
    return ESP_OK;
}
