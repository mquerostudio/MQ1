#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"

extern i2c_master_bus_handle_t board_i2c_bus_handle;
extern i2c_master_bus_handle_t module_i2c_bus_handle;

esp_err_t i2c_driver_init(void);