#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "board_pins.h"
#include "app_config.h"
#include "driver/i2c_master.h"
#include "soc/clk_tree_defs.h"

i2c_master_bus_handle_t board_i2c_bus_handle;
i2c_master_bus_handle_t module_i2c_bus_handle;

static const char *TAG = "I2C Driver";

esp_err_t i2c_driver_init(void)
{

    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = CONFIG_BOARD_I2C_PORT,
        .sda_io_num = (gpio_num_t)BOARD_I2C_SDA,
        .scl_io_num = (gpio_num_t)BOARD_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags.enable_internal_pullup = 0
    };

    if(i2c_new_master_bus(&i2c_bus_config, &board_i2c_bus_handle))
    {
        ESP_LOGE(TAG, "Main I2C driver init failed.");
        return ESP_FAIL;
    }


    i2c_bus_config.i2c_port = CONFIG_MOD_I2C_PORT;
    i2c_bus_config.sda_io_num = (gpio_num_t)MOD_I2C_SDA;
    i2c_bus_config.scl_io_num = (gpio_num_t)MOD_I2C_SCL;

    if(i2c_new_master_bus(&i2c_bus_config, &module_i2c_bus_handle))
    {
        ESP_LOGE(TAG, "Main I2C driver init failed.");
        return ESP_FAIL;
    }

    return ESP_OK;
}