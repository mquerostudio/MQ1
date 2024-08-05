#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define SEESAW_ADDRESS (0x49) ///< Default Seesaw I2C address

// LCD Config
#define CONFIG_LCD_H_RES        (240)
#define CONFIG_LCD_V_RES        (320)
#define DISP_BUF_SIZE           (CONFIG_LCD_H_RES * CONFIG_LCD_V_RES)
#define SPI_BUS_MAX_TRANSFER_SZ (DISP_BUF_SIZE * 2)


#define CONFIG_LCD_HOST         SPI2_HOST
#define CONFIG_LCD_FREQ         SPI_MASTER_FREQ_80M
#define CONFIF_LCD_CMD_BITS     (8)
#define CONFIG_LCD_PARAM_BITS   (16)

// Main I2C Config
#define CONFIG_BOARD_I2C_PORT   I2C_NUM_0

// Mod I2C Config
#define CONFIG_MOD_I2C_PORT     I2C_NUM_1

// Define a threshold for the number of button presses
#define PRESS_THRESHOLD 5

#endif // APP_CONFIG_H