#include "is31fl3216_custom.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>   // for memcpy
#include <inttypes.h> // for PRIu32 in logging
#include "soc/clk_tree_defs.h"

#define TAG "IS31FL3216C"

// Registers (from IS31FL3216 datasheet)
#define REG_SHUTDOWN   0x00
#define REG_PWM_BASE   0x01  // 0x01–0x10 (16 channels PWM)
#define REG_UPDATE     0x16

static i2c_port_t                 s_i2c_port = I2C_NUM_0;
static uint8_t                    s_dev_addr = IS31FL3216_CUSTOM_ADDR;
static i2c_master_bus_handle_t    s_bus      = NULL;
static i2c_master_dev_handle_t    s_device   = NULL;
static bool                       s_we_created_bus = false;

// Internal I2C helpers
static esp_err_t write_register(uint8_t reg, uint8_t val)
{
    if (!s_device) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t data[2] = { reg, val };
    return i2c_master_transmit(s_device, data, sizeof(data), 100);
}

static esp_err_t write_registers(uint8_t start_reg, const uint8_t *values, size_t len)
{
    if (!values || len > 16) return ESP_ERR_INVALID_ARG;
    if (!s_device) return ESP_ERR_INVALID_STATE;
    uint8_t buf[1 + 16];
    buf[0] = start_reg;
    memcpy(&buf[1], values, len);
    return i2c_master_transmit(s_device, buf, 1 + len, 100);
}

// Public API
esp_err_t is31fl3216_custom_init(const is31fl3216_custom_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    s_i2c_port = config->i2c_port;
    s_dev_addr = config->i2c_address ? config->i2c_address : IS31FL3216_CUSTOM_ADDR;

    if (s_device) {
        ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(s_device), TAG, "Failed to remove previous device");
        s_device = NULL;
    }

    if (!s_bus) {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = s_i2c_port,
            .sda_io_num = config->sda_io_num,
            .scl_io_num = config->scl_io_num,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = true,
            },
        };
        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "Failed to create I2C bus");
        s_we_created_bus = true;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_dev_addr,
        .scl_speed_hz = config->clk_speed_hz,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };
    esp_err_t err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_device);
    if (err != ESP_OK) {
        if (s_we_created_bus) {
            i2c_del_master_bus(s_bus);
            s_bus = NULL;
            s_we_created_bus = false;
        }
        return err;
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
    if (s_device) {
        i2c_master_bus_rm_device(s_device);
        s_device = NULL;
    }
    if (s_bus && s_we_created_bus) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        s_we_created_bus = false;
    }
    return ESP_OK;
}
