#include "esp_err.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_cst816s.h"

extern esp_lcd_panel_handle_t panel_handle;
extern esp_lcd_touch_handle_t tp_handle;
extern SemaphoreHandle_t touch_mux;


void display_init(void);