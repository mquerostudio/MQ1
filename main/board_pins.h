// pin_config.h
#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// TFT Board Pins
#define BOARD_TFT_MOSI       (39)
#define BOARD_TFT_SCK        (38)
#define BOARD_TFT_CS         (40)
#define BOARD_TFT_RST        (41)
#define BOARD_TFT_DC         (42)
#define BOARD_TFT_BL         (-1)
#define BOARD_DISP_TE        (-1)

#define BOARD_TOUCH_IRQ      (48)
#define BOARD_TOUCH_RST      (-1)
    
#define TFT_WIDTH            (240)
#define TFT_HEIGHT           (320)

#define DISPLAY_BUFFER_SIZE  (TFT_WIDTH * TFT_HEIGHT)
#define DISPLAY_FULLRESH     true

// Main I2C Bus
#define BOARD_I2C_SDA        (47)
#define BOARD_I2C_SCL        (21)

// Module I2C Bus
#define MOD_I2C_SDA          (9)  
#define MOD_I2C_SCL          (10)

// Power Control
#define BOARD_POWER          (48)  

#define CONFIG_USE_DEMO_WIDGETS true
#define CONFIG_USE_DEMO_BENCHMARK false
#define CONFIG_USE_DEMO_STRESS false
#define CONFIG_USE_DEMO_MUSIC false


// // MicroSD Slot
// #define PIN_SDCARD_CLK      43  
// #define PIN_SDCARD_MISO     44  
// #define PIN_SDCARD_MOSI     2   
// #define PIN_SDCARD_CS       1   
// #define SAMD21_SDCARD_DET   9   


// // USB-C
// #define PIN_USB_DPLUS       20   
// #define PIN_USB_DMINUS      19   

// // Encoder Switch
#define PIN_ENCODER_SW      48  

// // Power Control
// #define PIN_POWER_CTRL      45  

// // I2S Audio
// #define PIN_I2S_LRCLK       14  
// #define PIN_I2S_BLCK        12  
// #define PIN_I2S_MIC         13  
// #define PIN_I2S_SPEAKER     11  

// // Battery Management
// #define SAM21_BATM_INT      10
// #define SAM21_BATM_STAT1    4
// #define SAM21_BATM_STAT2    5

// // IMU 
// #define SAM21_IMU_INT       10
// #define SAM21_IMU_RST       18

// //NeoPixel
#define SAM21_PIN_NEOPIXEL  20

// // Encoder
// #define SAM21_ENC_A         6
// #define SAM21_ENC_B         7

// // External Module
// #define PIN_MOD_0           4
// #define PIN_MOD_1           5
// #define PIN_MOD_2           6
// #define PIN_MOD_3           7
// #define PIN_MOD_4           15
// #define PIN_MOD_5           16
// #define PIN_MOD_6           17
// #define PIN_MOD_7           18
// #define PIN_MOD_8           3
// #define PIN_MOD_9           46
// #define PIN_MOD_10          8
// #define SAM21_MOD_11        14
// #define SAM21_MOD_12        13
// #define SAM21_MOD_13        8
// #define SAM21_MOD_14        2
// #define SAM21_MOD_15        3
// #define SAM21_MOD_RESET     15
// #define SAM21_MOD_PWDN      16
// #define SAM21_MOD_DET       9
// #define SAM21_MOD_EEPROM    17

#endif // PIN_CONFIG_H