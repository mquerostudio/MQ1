#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "seesaw.h"

static const char *TAG = "Seesaw Driver";


/**
 * @brief Initializes the seesaw device.
 *
 * This function initializes the seesaw device by adding it to the I2C bus and allocating memory for the device handle.
 *
 * @param bus_handle The handle to the I2C bus.
 * @param seesaw_conf The configuration of the seesaw device.
 * @param seesaw_handle Pointer to the seesaw device handle.
 * @return
 *     - ESP_OK if the seesaw device is initialized successfully.
 *     - ESP_ERR_NO_MEM if there is not enough memory to allocate the device handle.
 *     - Other error codes if the initialization fails.
 */
esp_err_t seesaw_init(const i2c_master_bus_handle_t bus_handle, const i2c_device_config_t *seesaw_conf, seesaw_handle_t *seesaw_handle)
{

    esp_err_t ret = ESP_OK;
    seesaw_handle_t out_handle = NULL;

    // Allocate memory for the device handle
    out_handle = (seesaw_handle_t)calloc(1, sizeof(out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c seesaw device");

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = seesaw_conf->scl_speed_hz,
        .device_address = seesaw_conf->device_address
    };

    // Add the I2C device to the bus
    ret = i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "i2c new bus failed");

    *seesaw_handle = out_handle;

    return ESP_OK;

err:
    if (out_handle && out_handle->i2c_dev) {
        i2c_master_bus_rm_device(out_handle->i2c_dev);
    }
    free(out_handle);
    return ret;
}

/**
 * @brief Performs a software reset on the seesaw device.
 *
 * This function sends a command to the seesaw device to perform a software reset.
 * It writes a value of 0xFF to the SEESAW_STATUS_SWRST register of the device.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return `ESP_OK` if the software reset command was successfully sent, or an error code if an error occurred.
 */

esp_err_t seesaw_SW_reset(const seesaw_handle_t seesaw_handle)
{
    return seesaw_write8(seesaw_handle->i2c_dev, SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, 0xFF);
}

/**
 * @brief Turns on the seesaw device.
 *
 * This function turns on the seesaw device by setting the GPIO pins to input mode,
 * enabling pull-up resistors, and setting the pins to a high logic level.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return
 *     - ESP_OK if the seesaw device is turned on successfully.
 *     - An error code if there was an error turning on the seesaw device.
 */
esp_err_t seesaw_turnOn(const seesaw_handle_t seesaw_handle)
{
    
    esp_err_t ret = ESP_OK;
    uint8_t pins = (1ul << 7);
    uint8_t cmd[] = {(uint8_t)(pins >> 24), (uint8_t)(pins >> 16), (uint8_t)(pins >> 8), (uint8_t)pins};

    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 4); // set as input
    if (ret != ESP_OK) {
        return ret;
    }

    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, cmd, 4);   // enable pullup
    if (ret != ESP_OK) {
        return ret;
    }

    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, cmd, 4);    // set high
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Turns off the specified pins on the seesaw device.
 *
 * This function sets the specified pins as output and sets them to a low state.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return
 *     - ESP_OK if the pins were successfully turned off.
 *     - An error code if there was an error turning off the pins.
 */
esp_err_t seesaw_turnOff(const seesaw_handle_t seesaw_handle)
{

    esp_err_t ret = ESP_OK;
    uint8_t pins = (1ul << 7);
    uint8_t cmd[] = {(uint8_t)(pins >> 24), (uint8_t)(pins >> 16), (uint8_t)(pins >> 8), (uint8_t)pins};

    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE, SEESAW_GPIO_DIRSET_BULK, cmd, 4); // set as output
    if (ret != ESP_OK) {
        return ret;
    }

    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_CLR, cmd, 4);    // set low
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Get the current position of the encoder.
 *
 * This function reads the current position of the encoder from the seesaw device.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return The current position of the encoder.
 * @note This function returns an error code if there was an error reading the encoder position.
 */
int32_t seesaw_getEncoderPosition(const seesaw_handle_t seesaw_handle)
{
    esp_err_t ret = ESP_OK;
    uint8_t buf[4];

    ret = seesaw_read(seesaw_handle->i2c_dev, SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION, buf, 4);

    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading encoder position");
        return ret;
    } else
    {
        int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
        return ret;
    }
}

/**
 * @brief Get the delta value of the encoder.
 *
 * This function reads the delta value of the encoder from the seesaw device.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return The delta value of the encoder.
 * @note This function returns an error code if there was an error reading the encoder delta.
 */
int32_t seesaw_getEncoderDelta(const seesaw_handle_t seesaw_handle)
{
    esp_err_t ret = ESP_OK;
    uint8_t buf[4];

    ret = seesaw_read(seesaw_handle->i2c_dev, SEESAW_ENCODER_BASE, SEESAW_ENCODER_DELTA, buf, 4);

    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading encoder delta");
        return ret;
    } else 
    {
        int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
        return ret;
    }
}

/**
 * @brief Sets the position of the encoder.
 *
 * This function sets the position of the encoder using the specified handle and position value.
 *
 * @param seesaw_handle The handle to the I2C master device.
 * @param pos The position value to set for the encoder.
 * @return `ESP_OK` if the position was set successfully, or an error code if an error occurred.
 */
esp_err_t seesaw_setEncoderPosition(const seesaw_handle_t seesaw_handle, int32_t pos)
{
    uint8_t buf[] = {(uint8_t)(pos >> 24), (uint8_t)(pos >> 16), (uint8_t)(pos >> 8), (uint8_t)(pos & 0xFF)};

    return seesaw_write(seesaw_handle->i2c_dev, SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION, buf, 4);
}

/**
 * @brief Enables the interrupt for the encoder on the seesaw device.
 *
 * This function enables the interrupt for the encoder on the seesaw device
 * specified by the given `seesaw_handle`.
 *
 * @param seesaw_handle The handle to the seesaw device.
 * @return `ESP_OK` if the interrupt was successfully enabled, or an error code if it failed.
 */
esp_err_t seesaw_enableEncoderInterrupt(const seesaw_handle_t seesaw_handle)
{
    return seesaw_write8(seesaw_handle->i2c_dev, SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENSET, 0x01);
}

/**
 * @brief Disable the interrupt for the encoder on the seesaw device.
 *
 * This function disables the interrupt for the encoder on the seesaw device
 * specified by the given I2C master device handle.
 *
 * @param seesaw_handle The I2C master device handle for the seesaw device.
 * @return `ESP_OK` if the interrupt was successfully disabled, or an error code if not.
 */
esp_err_t seesaw_disableEncoderInterrupt(const seesaw_handle_t seesaw_handle)
{
    return seesaw_write8(seesaw_handle->i2c_dev, SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENCLR, 0x01);
}

/**
 * @brief Writes an 8-bit value to the specified register of the seesaw device.
 *
 * This function writes an 8-bit value to the specified register of the seesaw device
 * using the provided I2C master device handle.
 *
 * @param seesaw_handle The handle to the I2C master device connected to the seesaw device.
 * @param regHigh The high byte of the register address.
 * @param regLow The low byte of the register address.
 * @param value The 8-bit value to be written to the register.
 * @return `ESP_OK` if the write operation is successful, otherwise an error code.
 */
esp_err_t seesaw_write8(const i2c_master_dev_handle_t i2c_dev, uint8_t regHigh, uint8_t regLow, uint8_t value)
{
    return seesaw_write(i2c_dev, regHigh, regLow, &value, 1);
}

/**
 * @brief Writes data to the seesaw device.
 *
 * This function writes data to the seesaw device using the specified I2C handle.
 *
 * @param seesaw_handle The I2C handle of the seesaw device.
 * @param regHigh The high byte of the register address.
 * @param regLow The low byte of the register address.
 * @param buf Pointer to the data buffer.
 * @param num The number of bytes to write.
 *
 * @return `ESP_OK` on success, or an error code if an error occurred.
 */
esp_err_t seesaw_write(const i2c_master_dev_handle_t i2c_dev, uint8_t regHigh, uint8_t regLow, uint8_t *buf, uint8_t num)
{

    uint8_t buffer[2+num];
    buffer[0] = (uint8_t) regHigh;
    buffer[1] = (uint8_t) regLow;
    for(int i = 0; i < num; i++)
    {
        buffer[i+2] = buf[i];
    }

    return i2c_master_transmit(i2c_dev, buffer, sizeof(buffer), -1);
}

/**
 * @brief Reads data from the seesaw device.
 *
 * This function reads data from the seesaw device using the provided I2C handle.
 * It transmits the specified register address (regHigh and regLow) as a prefix,
 * and receives the data into the provided buffer (buf).
 *
 * @param seesaw_handle The I2C handle for the seesaw device.
 * @param regHigh The high byte of the register address.
 * @param regLow The low byte of the register address.
 * @param buf The buffer to store the received data.
 * @param num The number of bytes to read.
 * @return `ESP_OK` if the operation is successful, otherwise an error code.
 */
esp_err_t seesaw_read(const i2c_master_dev_handle_t i2c_dev, uint8_t regHigh, uint8_t regLow, uint8_t *buf, uint8_t num)
{

    uint8_t prefix[2];
    prefix[0] = (uint8_t) regHigh;
    prefix[1] = (uint8_t) regLow;

    return i2c_master_transmit_receive(i2c_dev, prefix, sizeof(prefix), buf, sizeof(buf), -1);
}

esp_err_t seesaw_neopixel_init(const seesaw_handle_t seesaw_handle, const neopixel_config_t *neopixel_config)
{
    esp_err_t ret = ESP_OK;

    seesaw_handle->neopixel_handle->brightness = 0;
    seesaw_handle->neopixel_handle->pixels = NULL;
    seesaw_handle->neopixel_handle->endTime = 0;
    seesaw_handle->neopixel_handle->numLEDs = neopixel_config->numLEDs;
    seesaw_handle->neopixel_handle->pin = neopixel_config->pin;
    seesaw_handle->neopixel_handle->type = neopixel_config->type;

    ret = seesaw_update_type(seesaw_handle, seesaw_handle->neopixel_handle->type);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to update type");
        return ret;
    }

    ret = seesaw_update_length(seesaw_handle, seesaw_handle->neopixel_handle->numLEDs);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to update length");
        return ret;
    }

    ret = seesaw_set_pin(seesaw_handle, seesaw_handle->neopixel_handle->pin);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set pin");
        return ret;
    } else{
        return ESP_OK;
    }    
}

esp_err_t seesaw_update_type(const seesaw_handle_t seesaw_handle, uint16_t t)
{
    bool oldTreeBytesPerPixel = (seesaw_handle->neopixel_handle->wOffset == seesaw_handle->neopixel_handle->rOffset);  // false if RGBW

    seesaw_handle->neopixel_handle->wOffset = (t >> 6) & 0b11; // See notes in header file
    seesaw_handle->neopixel_handle->rOffset = (t >> 4) & 0b11; // regarding R/G/B/W offsets
    seesaw_handle->neopixel_handle->gOffset = (t >> 2) & 0b11;
    seesaw_handle->neopixel_handle->bOffset = t & 0b11;
    seesaw_handle->neopixel_handle->is800KHz = (t < 256); // 400 KHz flag is 1<<8

    // If bytes-per-pixel has changed (and pixel data was previously
    // allocated), re-allocate to new size.  Will clear any data.
    if (seesaw_handle->neopixel_handle->pixels) {
      bool newThreeBytesPerPixel = (seesaw_handle->neopixel_handle->wOffset == seesaw_handle->neopixel_handle->rOffset);
      if (newThreeBytesPerPixel != oldTreeBytesPerPixel)
        seesaw_update_length(seesaw_handle, seesaw_handle->neopixel_handle->numLEDs);
    }

    return seesaw_write8(seesaw_handle->i2c_dev, SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_SPEED, seesaw_handle->neopixel_handle->is800KHz);
}

esp_err_t seesaw_update_length(const seesaw_handle_t seesaw_handle, uint16_t n)
{
    if (seesaw_handle->neopixel_handle->pixels)
      free(seesaw_handle->neopixel_handle->pixels); // Free existing data (if any)

    // Allocate new data -- note: ALL PIXELS ARE CLEARED
    seesaw_handle->neopixel_handle->numBytes = n * ((seesaw_handle->neopixel_handle->wOffset == seesaw_handle->neopixel_handle->rOffset) ? 3 : 4);
    if ((seesaw_handle->neopixel_handle->pixels = (uint8_t *)malloc(seesaw_handle->neopixel_handle->numBytes))) {
      memset(seesaw_handle->neopixel_handle->pixels, 0, seesaw_handle->neopixel_handle->numBytes);
      seesaw_handle->neopixel_handle->numLEDs = n;
    } else {
      seesaw_handle->neopixel_handle->numLEDs = seesaw_handle->neopixel_handle->numBytes = 0;
    }

    uint8_t buf[] = {(uint8_t)(seesaw_handle->neopixel_handle->numBytes >> 8), (uint8_t)(seesaw_handle->neopixel_handle->numBytes & 0xFF)};
    return seesaw_write(seesaw_handle->i2c_dev, SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF_LENGTH, buf, 2);
}
esp_err_t seesaw_set_pin(const seesaw_handle_t seesaw_handle, uint8_t p)
{
    esp_err_t ret = ESP_OK;

    ret = seesaw_write8(seesaw_handle->i2c_dev, SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_PIN, p);

    if(ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to set pin");
        return ret;
    } else{
        seesaw_handle->neopixel_handle->pin = p;
        return ESP_OK;
    }
}

bool seesaw_can_show(const seesaw_handle_t seesaw_handle) 
{ 
    return (esp_timer_get_time() - seesaw_handle->neopixel_handle->endTime) >= 300L; 
}


esp_err_t seesaw_show(const seesaw_handle_t seesaw_handle)
{
    esp_err_t ret = ESP_OK;

    if (!seesaw_handle->neopixel_handle->pixels)
      return ret; // No pixel data to send

    // Data latch = 300+ microsecond pause in the output stream.  Rather than
    // put a delay at the end of the function, the ending time is noted and
    // the function will simply hold off (if needed) on issuing the
    // subsequent round of data until the latch time has elapsed.  This
    // allows the mainline code to start generating the next frame of data
    // rather than stalling for the latch.
    while (!seesaw_can_show(seesaw_handle))
      ;

    ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_SHOW, NULL, 0);

    seesaw_handle->neopixel_handle->endTime = esp_timer_get_time(); // Save EOD time for latch on next call

    return ret;
}

void seesaw_set_brightness(const seesaw_handle_t seesaw_handle, uint8_t b)
{
    seesaw_handle->neopixel_handle->brightness = b; 
}

// Convert separate R,G,B into packed 32-bit RGB color.
// Packed format is always RGB, regardless of LED strand color order.
uint32_t seesaw_color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

uint16_t seesaw_num_pixels(const seesaw_handle_t seesaw_handle)
{
    return seesaw_handle->neopixel_handle->numLEDs;
}

// Set pixel color from 'packed' 32-bit RGB color:
esp_err_t seesaw_set_pixel_color(const seesaw_handle_t seesaw_handle, uint16_t n, uint32_t c)
{   
    esp_err_t ret = ESP_OK;

    if(n < seesaw_num_pixels(seesaw_handle)) 
    {   
        uint8_t *p, r = (uint8_t)(c >> 16), g = (uint8_t)(c >> 8), b = (uint8_t)c;

        if (seesaw_handle->neopixel_handle->brightness) 
        {
            r = (r * seesaw_handle->neopixel_handle->brightness) >> 8;
            g = (g * seesaw_handle->neopixel_handle->brightness) >> 8;
            b = (b * seesaw_handle->neopixel_handle->brightness) >> 8;
        }

        if (seesaw_handle->neopixel_handle->wOffset == seesaw_handle->neopixel_handle->rOffset) 
        {
          p = &seesaw_handle->neopixel_handle->pixels[n * 3];
        } else 
        {
          p = &seesaw_handle->neopixel_handle->pixels[n * 4];
          uint8_t w = (uint8_t)(c >> 24);
          p[seesaw_handle->neopixel_handle->wOffset] = seesaw_handle->neopixel_handle->brightness ? ((w * seesaw_handle->neopixel_handle->brightness) >> 8) : w;
        }

        p[seesaw_handle->neopixel_handle->rOffset] = r;
        p[seesaw_handle->neopixel_handle->gOffset] = g;
        p[seesaw_handle->neopixel_handle->bOffset] = b;

        uint8_t len = (seesaw_handle->neopixel_handle->wOffset == seesaw_handle->neopixel_handle->rOffset ? 3 : 4);
        uint16_t offset = n * len;

        uint8_t writeBuf[6];
        writeBuf[0] = (offset >> 8);
        writeBuf[1] = offset;
        memcpy(&writeBuf[2], p, len);

        ret = seesaw_write(seesaw_handle->i2c_dev, SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF, writeBuf, len + 2);
    }

    return ret;
}