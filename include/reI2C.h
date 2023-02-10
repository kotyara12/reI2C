/* 
   EN: Wrapping standard library i2c.h for ease of use.
   RU: Обертка для i2c.h ESP32 для упрощения работы
   --------------------------
   (с) 2021-2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_I2C_H__
#define __RE_I2C_H__

#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Checksum calculation. Used in various sensor drivers
 * */
uint8_t CRC8(uint8_t INIT, uint8_t MSB, uint8_t LSB);
uint8_t calcCRC8(uint16_t data);

/**
 * @brief Bus initialization
 * 
 * @param i2c_num: Bus number (0 or 1)
 * @param sda_io_num: Pin number SDA
 * @param scl_io_num: Pin number SCL
 * @param pullup_enable: Do I need to include the internal pull-up resistor
 * @param clk_speed: Bus frequency
 *   
 * @return true - successful, false - failure
 * 
 * */
bool initI2C(const i2c_port_t i2c_num, const int sda_io_num, const int scl_io_num, const bool pullup_enable, const uint32_t clk_speed);

/**
 * @brief Stopping the bus and deleting resources
 * 
 * @param i2c_num: Bus number (0 or 1)
 * 
 * */
void doneI2C(const i2c_port_t i2c_num);

/**
 * @brief Preparing the command chain
 * 
 * @param i2c_num: Bus number (0 or 1)
 * @param i2c_address: Bus device address
 * @param write: Write mode (aka read mode)
 * 
 * @return Handle on a chain
 * 
 * */
i2c_cmd_handle_t prepareI2C(i2c_port_t i2c_num, const uint8_t i2c_address, const bool write);

/**
 * @brief Execute a previously started chain and release bus
 * 
 * @param i2c_num: Bus number (0 or 1)
 * @param cmd: Handle on a command chain
 * @param timeout: Execution timeout
 * 
 * @return Error code
 * 
 * */
esp_err_t execI2C(i2c_port_t i2c_num, i2c_cmd_handle_t cmd, TickType_t timeout);

/**
 * @brief Search for devices on the bus
 * 
 * @param i2c_num: Bus number (0 or 1)
 * 
 * */
void scanI2C(i2c_port_t i2c_num);

/**
 * @brief Reset through General Call
 * All devices on the same I2C bus that support the general call mode will perform a reset. 
 * Additionally, this command only works when the sensor is able to process I2C commands. 
 * 
 * @param i2c_num: Bus number (0 or 1)
 * 
 * @return Error code
 * 
 * */
esp_err_t generalCallResetI2C(i2c_port_t i2c_num);

/**
 * 
 * @brief Wake up the device
 * 
 * @param i2c_num: Bus number (0 or 1)
 * @param @param timeout: Execution timeout
 *  
 * @return esp error code
 * 
 * */
esp_err_t wakeI2C(i2c_port_t i2c_num, const uint8_t i2c_address,
  TickType_t timeout);

/**
 * @brief Read one or more bytes from the device
 * 
 * @param i2c_num: Bus number (0 or 1)
 * @param i2c_address: Device address (01 - 7F)
 * @param cmds: Pointer to the first byte of the command
 * @param cmds_size: Number of command bytes
 * @param data: Pointer to read data buffer
 * @param data_size: Data buffer size
 * @param wait_data_us: A pause between sending a command and receiving data with a bus free in µs. If 0, then the bus is not released.
 * @param @param timeout: Execution timeout
 * 
 * @return esp error code
 * 
 * */
esp_err_t readI2C(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  const uint32_t wait_data_us, const TickType_t timeout);

/**
 * @brief Read words (2 bytes each) from device with CRC check for each word
 * 
 * @param i2c_num: Bus number (0 or 1)
 * @param i2c_address: Device address (01 - 7F)
 * @param cmds: Pointer to the first byte of the command
 * @param cmds_size: Number of command bytes
 * @param data: Pointer to read data buffer
 * @param data_size: Data buffer size
 * @param wait_data_us: A pause between sending a command and receiving data with a bus free in µs. If 0, then the bus is not released.
 * @param @param timeout: Execution timeout
 * 
 * @return esp error code
 * 
 * */
esp_err_t readI2C_CRC8(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  const uint32_t wait_data_us, const uint8_t crc_init, const TickType_t timeout);

/**
 * @brief Send one or more bytes to the device
 * 
 * @param i2c_num: Bus number (0 or 1)
 * @param i2c_address: Device address (01 - 7F)
 * @param cmds: Pointer to the first byte of the command
 * @param cmds_size: Number of command bytes
 * @param data: Pointer to send data buffer
 * @param data_size: Data buffer size
 * @param @param timeout: Execution timeout
 * 
 * @return esp error code
 * 
 * */
esp_err_t writeI2C(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  TickType_t timeout);

#ifdef __cplusplus
}
#endif

#endif // __RE_I2C_H__

