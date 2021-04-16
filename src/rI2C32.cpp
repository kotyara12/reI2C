#include "rI2C32.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" 
#include "rLog.h"

static const char * i2cTAG  = "I2C";

#define ERROR_I2C_CREATE_MUTEX    "Error creating I2C bus lock mutex!"
#define ERROR_I2C_PREPARE         "Error connect to device on bus %d at address 0x%.2X: #%d %s!"
#define ERROR_I2C_READ            "Error reading device on bus %d at address 0x%.2X: #%d %s!"
#define ERROR_I2C_WRITE           "Error writing to device on bus %d at address 0x%.2X: #%d %s!"

xSemaphoreHandle lockI2C[I2C_NUM_MAX];

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
  if (lockI2C[i2c_num]) {
    lockI2C[i2c_num] = xSemaphoreCreateRecursiveMutex();
    if (!lockI2C[i2c_num]) {
      rlog_e(i2cTAG, ERROR_I2C_CREATE_MUTEX);
      return false;
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
  if (lockI2C[i2c_num]) {
    vSemaphoreDelete(lockI2C[i2c_num]);
    lockI2C[i2c_num] = nullptr;
  };
}

void takeI2C(const i2c_port_t i2c_num)
{
  if (!lockI2C[i2c_num]) {
    lockI2C[i2c_num] = xSemaphoreCreateRecursiveMutex();
    if (!lockI2C[i2c_num]) {
      rlog_e(i2cTAG, ERROR_I2C_CREATE_MUTEX);
      return;
    };
  };
  do {} while (xSemaphoreTakeRecursive(lockI2C[i2c_num], portMAX_DELAY) != pdPASS);
}

void giveI2C(const i2c_port_t i2c_num)
{
  xSemaphoreGiveRecursive(lockI2C[i2c_num]);
}

i2c_cmd_handle_t prepareI2C(const uint8_t i2c_address, const bool write)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (cmd) {
    i2c_master_start(cmd);
    if (write) {
      i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    }
    else {
      i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    };
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

#define I2C_CHECK(err, message) \
  if ((err) != ESP_OK) { \
    rlog_e(i2cTAG, message, i2c_num, i2c_address, err, esp_err_to_name(err)); \
    if (cmdLink) i2c_cmd_link_delete(cmdLink); \
    giveI2C(i2c_num); \
    return err; \
  };

esp_err_t readExI2C(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  const uint32_t wait_data_us, const TickType_t timeout)
{
  // Lock bus
  takeI2C(i2c_num);
  i2c_cmd_handle_t cmdLink;
  // Send command(s)
  cmdLink = i2c_cmd_link_create();
  if (cmdLink) {
    // Send commands, if needed
    if ((cmds) && (cmds_size > 0)) {
      I2C_CHECK(i2c_master_start(cmdLink), ERROR_I2C_WRITE);
      I2C_CHECK(i2c_master_write_byte(cmdLink, (i2c_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN), ERROR_I2C_WRITE);
      I2C_CHECK(i2c_master_write(cmdLink, cmds, cmds_size, ACK_CHECK_EN), ERROR_I2C_WRITE);
      // If there is a pause for waiting for data, we immediately send commands and release the bus
      if (wait_data_us > 0) {
        I2C_CHECK(i2c_master_stop(cmdLink), ERROR_I2C_WRITE);
        I2C_CHECK(i2c_master_cmd_begin(i2c_num, cmdLink, timeout / portTICK_RATE_MS), ERROR_I2C_WRITE);
        i2c_cmd_link_delete(cmdLink);
        cmdLink = nullptr;
        // We wait...
        ets_delay_us(wait_data_us);
        // Initializing a new batch of commands
        cmdLink = i2c_cmd_link_create();
      };
    };
    // Reading data
    if (cmdLink) {
      I2C_CHECK(i2c_master_start(cmdLink), ERROR_I2C_READ);
      I2C_CHECK(i2c_master_write_byte(cmdLink, (i2c_address << 1) | I2C_MASTER_READ, ACK_CHECK_EN), ERROR_I2C_READ);
      I2C_CHECK(i2c_master_read(cmdLink, data, data_size, I2C_MASTER_LAST_NACK), ERROR_I2C_READ);
      I2C_CHECK(i2c_master_stop(cmdLink), ERROR_I2C_READ);
      I2C_CHECK(i2c_master_cmd_begin(i2c_num, cmdLink, timeout / portTICK_RATE_MS), ERROR_I2C_READ);
      i2c_cmd_link_delete(cmdLink);
      // Unlock bus
      giveI2C(i2c_num);
      return ESP_OK;
    };
  };
  // Unlock bus
  giveI2C(i2c_num);
  return ESP_FAIL;
}

bool readI2C(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  const uint32_t wait_data_us, const TickType_t timeout)
{
  return readExI2C(i2c_num, i2c_address, cmds, cmds_size, data, data_size, wait_data_us, timeout) == ESP_OK;
}

esp_err_t readI2C_CRC8(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  const uint32_t wait_data_us, const uint8_t crc_init, const TickType_t timeout)
{
  esp_err_t err = readExI2C(i2c_num, i2c_address, cmds, cmds_size, data, data_size, wait_data_us, timeout);
  if (err == ESP_OK) {
    // For each word (2 bytes) of data 1 byte CRC
    if (data_size % 3 != 0) {
      return ESP_ERR_INVALID_SIZE;
    };
    for (int i = 0; i < (data_size / 3); i++) {
      if (CRC8(crc_init, data[i*3], data[i*3+1]) != data[i*3+2]) {
        return ESP_ERR_INVALID_CRC;
      };
    };
  };
  return err;
}

esp_err_t writeExI2C(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  TickType_t timeout)
{
  takeI2C(i2c_num);
  i2c_cmd_handle_t cmdLink = i2c_cmd_link_create();
  if (cmdLink) {
    I2C_CHECK(i2c_master_start(cmdLink), ERROR_I2C_WRITE);
    I2C_CHECK(i2c_master_write_byte(cmdLink, (i2c_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN), ERROR_I2C_WRITE);
    I2C_CHECK(i2c_master_write(cmdLink, cmds, cmds_size, ACK_CHECK_EN), ERROR_I2C_WRITE);
    if ((data) && (data_size>0)) {
      I2C_CHECK(i2c_master_write(cmdLink, data, data_size, ACK_CHECK_EN), ERROR_I2C_WRITE);
    };
    I2C_CHECK(i2c_master_stop(cmdLink), ERROR_I2C_WRITE);
    I2C_CHECK(i2c_master_cmd_begin(i2c_num, cmdLink, timeout / portTICK_RATE_MS), ERROR_I2C_WRITE);
    i2c_cmd_link_delete(cmdLink);
    // Unlock bus
    giveI2C(i2c_num);
    return ESP_OK;
  };
  // Unlock bus
  giveI2C(i2c_num);
  return ESP_FAIL;
}

bool writeI2C(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  TickType_t timeout)
{
  return writeExI2C(i2c_num, i2c_address, cmds, cmds_size, data, data_size, timeout) == ESP_OK;
}