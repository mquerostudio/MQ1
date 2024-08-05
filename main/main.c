#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "lvgl.h"

#include "driver/gpio.h"
#include "i2c_driver.h"

#include "seesaw.h"

#include "board_pins.h"
#include "app_config.h"

static const char *TAG = "main";

seesaw_handle_t seesaw_handle;

void button_task(void *arg)
{
    // Configure button input pin
    gpio_set_direction(PIN_ENCODER_SW, GPIO_MODE_INPUT);

    int64_t button_press_time = 0;

    for(;;)
    {
        if (gpio_get_level(PIN_ENCODER_SW) == 0) // Button pressed
        {
            if (button_press_time == 0)
            {
                button_press_time = esp_timer_get_time(); // Record the press start time
            }

            if ((esp_timer_get_time() - button_press_time) >= 3000000) // 5 seconds in microseconds
            {
                ESP_LOGI(TAG, "------ Unlock power off");
                ESP_ERROR_CHECK(seesaw_turnOff(seesaw_handle));
                while(1);
            }
        } else
        {
            button_press_time = 0;          // Reset press time if button is released
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Debouncing delay
    }
}

void encoder_task(void *arg)
{    
    int32_t encoder_position;

    ESP_LOGI(TAG, "------ Setup Encoder");
    encoder_position = seesaw_getEncoderPosition(seesaw_handle);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(seesaw_enableEncoderInterrupt(seesaw_handle));

    for (;;)
    {
        int32_t new_encoder_position = seesaw_getEncoderPosition(seesaw_handle);

        if(encoder_position != new_encoder_position)
        {
            ESP_LOGI(TAG, "Encoder position: %" PRId32, encoder_position);
            encoder_position = new_encoder_position;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }   
}

void app_main()
{
    ESP_LOGI(TAG, "------ Initialize Main and Module I2C.");
    ESP_ERROR_CHECK(i2c_driver_init());

    ESP_LOGI(TAG, "------ Initialize Seesaw.");
    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = (400 * 1000),
        .device_address = SEESAW_ADDRESS};
    ESP_ERROR_CHECK(seesaw_init(board_i2c_bus_handle, &i2c_dev_conf, &seesaw_handle));
    ESP_ERROR_CHECK(seesaw_SWreset(seesaw_handle));

    ESP_LOGI(TAG, "------ Lock Power On");
    ESP_ERROR_CHECK(seesaw_turnOn(seesaw_handle));
    ESP_LOGI(TAG, "------ Lock Power Done");

    // Create the button task
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(encoder_task, "encoder_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "------ Setup Neopixel");

    neopixel_config_t neopixel_config = {
        .numLEDs = 24,
        .pin = SAM21_PIN_NEOPIXEL,
        .type = (NEO_GRB + NEO_KHZ800),
    };

    // ESP_ERROR_CHECK(seesaw_neopixel_init(seesaw_handle, &neopixel_config));

    // ESP_LOGI(TAG, "------ Neopixel Off");
    // ESP_ERROR_CHECK(seesaw_show(seesaw_handle));

    
    // ESP_LOGI(TAG, "------ Neopixel On");

    // for(uint16_t i=0; i<seesaw_num_pixels(seesaw_handle); i++) 
    // {
    //     seesaw_set_pixel_color(seesaw_handle, i, seesaw_color(255, 0, 0));
    //     seesaw_show(seesaw_handle);
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }

    // for(uint16_t i=seesaw_num_pixels(seesaw_handle); i>0; i--) 
    // {
    //     seesaw_set_pixel_color(seesaw_handle, i, seesaw_color(255, 0, 0));
    //     seesaw_show(seesaw_handle);
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }


    //     // ESP_LOGI(TAG, "------ Initialize Touch.");
    //     // if (touch_init() != ESP_OK)
    //     // {
    //     //     ESP_LOGE(TAG, "Touch driver init failed.");
    //     //     return;
    //     // }

    // ESP_LOGI(TAG, "------ Initialize DISPLAY.");
    // display_init();
    // esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, CONFIG_LCD_H_RES - 1, CONFIG_LCD_V_RES - 1, color_map);

    //     ESP_LOGI(TAG, "Initialize LVGL library");
    //     lv_init();
}