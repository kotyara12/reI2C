#include "rI2C32.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" 
#include "rLog.h"

static const char * i2cTAG  = "I2C";

#define ERROR_I2C_CREATE_MUTEX    "Error creating I2C bus lock mutex!"
#define ERROR_I2C_REGISTER_READ   "Error reading device on bus %d at address 0x%.2X: #%d %s!"
#define ERROR_I2C_REGISTER_WRITE  "Error writing to device on bus %d at address 0x%.2X: #%d %s!"

xSemaphoreHandle lockI2C0, lockI2C1; 

uint8_t calcCRC8(uint16_t data)
{
  for (uint8_t bit = 0; bit < 16; bit++) {
    if (data & 0x8000) data = (data << 1) ^ 0x13100;
    else data <<= 1;
  };
  return data >>= 8;
}

/*
*	Name  : CRC-8
*	Poly  : 0x31	x^8 + x^5 + x^4 + 1
*	Init  : 0xFF
*	Revert: false
*	XorOut: 0x00
*	Check : for 0xBE,0xEF CRC is 0x92
*/
uint8_t CRC8(uint8_t INIT, uint8_t MSB, uint8_t LSB)
{
	uint8_t crc = INIT;
	uint8_t i;
	crc ^= MSB;
  for (i = 0; i < 8; i++)
	  crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
	crc ^= LSB;
	for (i = 0; i < 8; i++)
		crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
	return crc;
}

bool initI2C(const i2c_port_t i2c_num, const int sda_io_num, const int scl_io_num, const bool pullup_enable, const uint32_t clk_speed)
{
  if (i2c_num == 0) {
    if (!lockI2C0) {
      lockI2C0 = xSemaphoreCreateRecursiveMutex();
      if (!lockI2C0) {
        rlog_e(i2cTAG, ERROR_I2C_CREATE_MUTEX);
        return false;
      };
    };
  } else {
    if (!lockI2C1) {
      lockI2C1 = xSemaphoreCreateRecursiveMutex();
      if (!lockI2C1) {
        rlog_e(i2cTAG, ERROR_I2C_CREATE_MUTEX);
        return false;
      };
    };
  };

  i2c_config_t confI2C;
  confI2C.mode = I2C_MODE_MASTER;
  confI2C.sda_io_num = sda_io_num;
  confI2C.sda_pullup_en = pullup_enable;
  confI2C.scl_io_num = scl_io_num;
  confI2C.scl_pullup_en = pullup_enable;
  confI2C.master.clk_speed = clk_speed;

  esp_err_t err = i2c_param_config(i2c_num, &confI2C);
  if (err != ESP_OK) {
    rlog_e(i2cTAG, "I2C bus setup error!");
    return false;
  };

  err = i2c_driver_install(i2c_num, confI2C.mode, 0, 0, 0);
  if (err == ESP_OK) {
    rlog_i(i2cTAG, "I2C bus #%d started successfully on GPIO SDA: %d, SCL: %d", i2c_num, sda_io_num, scl_io_num);
    return true;
  }
  else {
    rlog_e(i2cTAG, "I2C bus initialization error!");
    return false;
  };
}

void doneI2C(const i2c_port_t i2c_num)
{
  i2c_driver_delete(i2c_num); 
  if (i2c_num == 0) {
    if (lockI2C0) {
      vSemaphoreDelete(lockI2C0);
    };
  } else {
    if (lockI2C1) {
      vSemaphoreDelete(lockI2C1);
    };
  };
}

void takeI2C(const i2c_port_t i2c_num)
{
  if (i2c_num == 0) {
    if (!lockI2C0) {
      lockI2C0 = xSemaphoreCreateRecursiveMutex();
      if (!lockI2C0) {
        rlog_e(i2cTAG, ERROR_I2C_CREATE_MUTEX);
        return;
      };
    };
    do {} while (xSemaphoreTakeRecursive(lockI2C0, portMAX_DELAY) != pdPASS);
  } else {
    if (!lockI2C1) {
      lockI2C1 = xSemaphoreCreateRecursiveMutex();
      if (!lockI2C1) {
        rlog_e(i2cTAG, ERROR_I2C_CREATE_MUTEX);
        return;
      };
    };
    do {} while (xSemaphoreTakeRecursive(lockI2C1, portMAX_DELAY) != pdPASS);
  };
}

void giveI2C(const i2c_port_t i2c_num)
{
  if (i2c_num == 0) {
    xSemaphoreGiveRecursive(lockI2C0);
  } else {
    xSemaphoreGiveRecursive(lockI2C1);
  };
}

i2c_cmd_handle_t prepareI2C(const uint8_t i2c_address, const bool write)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  if (write) {
    i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  }
  else {
    i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
  };
  return cmd;
}

esp_err_t execI2C(i2c_port_t i2c_num, i2c_cmd_handle_t cmd, TickType_t timeout)
{
  i2c_master_stop(cmd);
  takeI2C(i2c_num);
  esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, timeout / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  giveI2C(i2c_num);
  return err;
}

bool readI2C(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  const uint32_t wait_data_us, const TickType_t timeout)
{
  // Lock bus
  takeI2C(i2c_num);
  esp_err_t err;
  i2c_cmd_handle_t cmd;
  // Send command(s)
  cmd = i2c_cmd_link_create();
  if (cmds) {
    // Send commands
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, cmds, cmds_size, ACK_CHECK_EN);
    // If there is a pause for waiting for data, we immediately send commands and release the bus
    if (wait_data_us > 0) {
      i2c_master_stop(cmd);
      err = i2c_master_cmd_begin(i2c_num, cmd, timeout / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd);
      if (err != ESP_OK) {
        rlog_e(i2cTAG, ERROR_I2C_REGISTER_WRITE, i2c_num, i2c_address, err, esp_err_to_name(err));
        return false;
      };
      // We wait...
      ets_delay_us(wait_data_us);
      // Initializing a new batch of commands
      cmd = i2c_cmd_link_create();
    };
  };
  // Reading data
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
  i2c_master_read(cmd, data, data_size, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(i2c_num, cmd, timeout / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  // Unlock bus
  giveI2C(i2c_num);
  if (err != ESP_OK) {
    rlog_e(i2cTAG, ERROR_I2C_REGISTER_READ, i2c_num, i2c_address, err, esp_err_to_name(err));
  };
  return err == ESP_OK;
}

bool writeI2C(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  TickType_t timeout)
{
  takeI2C(i2c_num);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write(cmd, cmds, cmds_size, ACK_CHECK_EN);
  if (data) {
    i2c_master_write(cmd, data, data_size, ACK_CHECK_EN);
  };
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, timeout / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  giveI2C(i2c_num);
  if (err != ESP_OK) {
    rlog_e(i2cTAG, ERROR_I2C_REGISTER_WRITE, i2c_num, i2c_address, err, esp_err_to_name(err));
  };
  return err == ESP_OK;
}

