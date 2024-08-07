#include "esp_err.h"
#include "esp_log.h"

#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

// #include "esp_lcd_touch_cst816s.h"

#include "display.h"
#include "board_pins.h"
#include "app_config.h"

static const char *TAG = "Display Driver";

esp_lcd_panel_handle_t panel_handle;
esp_lcd_touch_handle_t tp_handle;
SemaphoreHandle_t touch_mux;

static void touch_callback(esp_lcd_touch_handle_t tp_handle)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(touch_mux, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void display_init(void){

    ESP_LOGI(TAG, "SPI driver init");
    spi_bus_config_t buscfg = {
        .sclk_io_num = BOARD_LCD_SCK,
        .mosi_io_num = BOARD_LCD_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = CONFIG_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI driver init done");


    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BOARD_LCD_DC,
        .cs_gpio_num = BOARD_LCD_CS,
        .pclk_hz = CONFIG_LCD_FREQ,
        .lcd_cmd_bits = CONFIF_LCD_CMD_BITS,
        .lcd_param_bits = CONFIG_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)CONFIG_LCD_HOST, &io_config, &io_handle));
    ESP_LOGI(TAG, "Install panel IO done");

    ESP_LOGI(TAG, "Install ST7789 panel driver");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BOARD_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_LOGI(TAG, "Install ST7789 panel driver done");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    touch_mux = xSemaphoreCreateBinary();

    // esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    // esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    // ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_spi_bus_handle_t)CONFIG_BOARD_I2C_PORT, &tp_io_config, &tp_io_handle));

    // esp_lcd_touch_config_t tp_cfg = {
    //     .x_max = CONFIG_LCD_H_RES,
    //     .y_max = CONFIG_LCD_V_RES,
    //     .rst_gpio_num = BOARD_TOUCH_RST,
    //     .int_gpio_num = BOARD_TOUCH_IRQ,
    //     .levels = {
    //         .reset = 0,
    //         .interrupt = 0,
    //     },
    //     .flags = {
    //         .swap_xy = 0,
    //         .mirror_x = 0,
    //         .mirror_y = 0,
    //     },
    //     .interrupt_callback = touch_callback,
    // };
    // esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp_handle);
}
