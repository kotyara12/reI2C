#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" 
#include "rom/ets_sys.h"
#include "rLog.h"
#include "reI2C.h"
#include "project_config.h"

#if CONFIG_RLOG_PROJECT_LEVEL > RLOG_LEVEL_NONE
static const char * logTAG  = "I2C";
#endif // CONFIG_RLOG_PROJECT_LEVEL

#define ERROR_I2C_CREATE_MUTEX    "Error creating I2C bus lock mutex!"
#define ERROR_I2C_PREPARE         "Error connect to device on bus %d at address 0x%.2X: #%d %s!"
#define ERROR_I2C_READ            "Error reading device on bus %d at address 0x%.2X: #%d %s!"
#define ERROR_I2C_WRITE           "Error writing to device on bus %d at address 0x%.2X: #%d %s!"

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------- Blocking access to I2C from different tasks -------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#ifdef CONFIG_I2C_LOCK
  #ifndef CONFIG_I2C_LOCK_STATIC
    #define CONFIG_I2C_LOCK_STATIC 1
  #endif
  #define I2C_LOCK_ENABLED CONFIG_I2C_LOCK
#else
  #define I2C_LOCK_ENABLED 0
#endif // CONFIG_I2C_LOCK

#if I2C_LOCK_ENABLED
  SemaphoreHandle_t _lockI2C[I2C_NUM_MAX];
  #if CONFIG_I2C_LOCK_STATIC
    StaticSemaphore_t _buffMutexI2C[I2C_NUM_MAX];
  #endif
  #define takeI2C(i2c_num) do {} while (xSemaphoreTakeRecursive(_lockI2C[(i2c_num)], portMAX_DELAY) != pdPASS)
  #define giveI2C(i2c_num) xSemaphoreGiveRecursive(_lockI2C[(i2c_num)])
#else
  #define takeI2C(i2c_num) {}
  #define giveI2C(i2c_num) {}
#endif // I2C_LOCK_ENABLED

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------- Static buffer allocation for command link --------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if defined(CONFIG_I2C_PORT0_SDA) && defined(CONFIG_I2C_PORT0_STATIC) && (CONFIG_I2C_PORT0_STATIC > 0)
  #define I2C0_USE_STATIC 1
  static uint8_t _bufferI2C0[I2C_LINK_RECOMMENDED_SIZE(CONFIG_I2C_PORT0_STATIC)] = { 0 };
  #define __i2c0_cmd_link_create() i2c_cmd_link_create_static(_bufferI2C0, sizeof(_bufferI2C0))
  #define __i2c0_cmd_link_delete(cmd_handle) i2c_cmd_link_delete_static(cmd_handle)
#else
  #define I2C0_USE_STATIC 0
  #define __i2c0_cmd_link_create() i2c_cmd_link_create()
  #define __i2c0_cmd_link_delete(cmd_handle) i2c_cmd_link_delete(cmd_handle)
#endif // CONFIG_I2C_PORT0_STATIC

#if defined(CONFIG_I2C_PORT1_SDA) && defined(CONFIG_I2C_PORT1_STATIC) && (CONFIG_I2C_PORT1_STATIC > 0)
  #define I2C1_USE_STATIC 1
  static uint8_t _bufferI2C1[I2C_LINK_RECOMMENDED_SIZE(CONFIG_I2C_PORT1_STATIC)] = { 0 };
  #define __i2c1_cmd_link_create() i2c_cmd_link_create_static(_bufferI2C1, sizeof(_bufferI2C1))
  #define __i2c1_cmd_link_delete(cmd_handle) i2c_cmd_link_delete_static(cmd_handle)
#else
  #define I2C1_USE_STATIC 0
  #define __i2c1_cmd_link_create() i2c_cmd_link_create()
  #define __i2c1_cmd_link_delete(cmd_handle) i2c_cmd_link_delete(cmd_handle)
#endif // CONFIG_I2C_PORT0_STATIC

#define _i2c_cmd_link_create(i2c_num) (i2c_num == 0) ? __i2c0_cmd_link_create() : __i2c1_cmd_link_create()
#define _i2c_cmd_link_delete(i2c_num, cmd_handle) (i2c_num == 0) ? __i2c0_cmd_link_delete(cmd_handle) : __i2c1_cmd_link_delete(cmd_handle)

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------- Checksum calculation during transmission ---------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------- Initialize ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool initI2C(const i2c_port_t i2c_num, const int sda_io_num, const int scl_io_num, const bool pullup_enable, const uint32_t clk_speed)
{
  // Create lock objects
  #if I2C_LOCK_ENABLED
    if (_lockI2C[i2c_num] == nullptr) {
      #if CONFIG_I2C_LOCK_STATIC
        _lockI2C[i2c_num] = xSemaphoreCreateRecursiveMutexStatic(&_buffMutexI2C[i2c_num]);
      #else
        _lockI2C[i2c_num] = xSemaphoreCreateRecursiveMutex();
      #endif
      if (_lockI2C[i2c_num] == nullptr) {
        rlog_e(logTAG, ERROR_I2C_CREATE_MUTEX);
        return false;
      };
    };
  #endif // I2C_LOCK_ENABLED

  // Config I2C bus
  i2c_config_t confI2C;
  confI2C.mode = I2C_MODE_MASTER;
  confI2C.sda_io_num = sda_io_num;
  confI2C.sda_pullup_en = pullup_enable;
  confI2C.scl_io_num = scl_io_num;
  confI2C.scl_pullup_en = pullup_enable;
  confI2C.master.clk_speed = clk_speed;
  confI2C.clk_flags = 0;

  esp_err_t err = i2c_param_config(i2c_num, &confI2C);
  if (err != ESP_OK) {
    rlog_e(logTAG, "I2C bus #%d setup error: #%d (%s)!", i2c_num, err, esp_err_to_name);
    return false;
  };

  err = i2c_driver_install(i2c_num, confI2C.mode, 0, 0, 0);
  if (err == ESP_OK) {
    rlog_i(logTAG, "I2C bus #%d started successfully on GPIO SDA: %d, SCL: %d", i2c_num, sda_io_num, scl_io_num);
    return true;
  }
  else {
    rlog_e(logTAG, "I2C bus #%d initialization error: #%d (%s)!", i2c_num, err, esp_err_to_name);
    return false;
  };
}

void doneI2C(const i2c_port_t i2c_num)
{
  i2c_driver_delete(i2c_num); 
  #if I2C_LOCK_ENABLED
    if (_lockI2C[i2c_num] != nullptr) {
      vSemaphoreDelete(_lockI2C[i2c_num]);
      _lockI2C[i2c_num] = nullptr;
    };
  #endif // I2C_LOCK_ENABLED
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Macro-wrappers ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

i2c_cmd_handle_t prepareI2C(i2c_port_t i2c_num, const uint8_t i2c_address, const bool write)
{
  i2c_cmd_handle_t cmd = _i2c_cmd_link_create(i2c_num);
  if (cmd) {
    esp_err_t err = i2c_master_start(cmd);
    if (err != ESP_OK) goto error;
    err = i2c_master_write_byte(cmd, (i2c_address << 1) | (write ? I2C_MASTER_WRITE : I2C_MASTER_READ), ACK_CHECK_EN);
    if (err != ESP_OK) goto error;
  };
  return cmd;
error:
  _i2c_cmd_link_delete(i2c_num, cmd);
  return nullptr;
}

esp_err_t execI2C(i2c_port_t i2c_num, i2c_cmd_handle_t cmd, TickType_t timeout)
{
  esp_err_t err = i2c_master_stop(cmd);
  if (err == ESP_OK) {
    takeI2C(i2c_num);
    err = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(timeout));
    _i2c_cmd_link_delete(i2c_num, cmd);
    giveI2C(i2c_num);
  };
  return err;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------- Service functions --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

esp_err_t generalCallResetI2C(i2c_port_t i2c_num) 
{
  // Lock bus
  takeI2C(i2c_num);
  esp_err_t error_code = ESP_ERR_NO_MEM;
  // Create command link
  i2c_cmd_handle_t cmdLink = _i2c_cmd_link_create(i2c_num);
  if (cmdLink) {
    // Start bit
    error_code = i2c_master_start(cmdLink);
    if (error_code != ESP_OK) goto end;
    // Reset command using the general call address: 0x0006
    error_code = i2c_master_write_byte(cmdLink, 0x00, ACK_CHECK_EN);
    if (error_code != ESP_OK) goto end;
    error_code = i2c_master_write_byte(cmdLink, 0x06, ACK_CHECK_EN);
    if (error_code != ESP_OK) goto end;
    // No stop bit
    // error_code = i2c_master_stop(cmdLink);
    // if (error_code != ESP_OK) goto end;
    error_code = i2c_master_cmd_begin(i2c_num, cmdLink, pdMS_TO_TICKS(3000));
    if (error_code != ESP_OK) goto end;
  };
end:  
  // Unlock bus and release resources
  _i2c_cmd_link_delete(i2c_num, cmdLink);
  giveI2C(i2c_num);
  return error_code;
}

void scanI2C(i2c_port_t i2c_num)
{
  uint8_t cnt = 0;
  esp_err_t error_code;
  for (uint i = 1; i < 128; i++) {
    i2c_cmd_handle_t cmdLink = _i2c_cmd_link_create(i2c_num);
    i2c_master_start(cmdLink);
    i2c_master_write_byte(cmdLink, (i << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmdLink);
    error_code = i2c_master_cmd_begin(i2c_num, cmdLink, pdMS_TO_TICKS(1000));
    _i2c_cmd_link_delete(i2c_num, cmdLink);
    if (error_code == ESP_OK) {
      cnt++;
      rlog_i(logTAG, "Found device on bus %d at address 0x%.2X", i2c_num, i);
    };
    // vTaskDelay(1);
  };
  rlog_i(logTAG, "Found %d devices on bus %d", cnt, i2c_num);
}

esp_err_t wakeI2C(i2c_port_t i2c_num, const uint8_t i2c_address, TickType_t timeout) 
{
  takeI2C(i2c_num);
  esp_err_t error_code = ESP_ERR_NO_MEM;
  i2c_cmd_handle_t cmdLink = _i2c_cmd_link_create(i2c_num);
  if (cmdLink) {
    error_code = i2c_master_start(cmdLink);
    if (error_code != ESP_OK) goto end;
    error_code = i2c_master_write_byte(cmdLink, (i2c_address << 1) | I2C_MASTER_WRITE, true);
    if (error_code != ESP_OK) goto end;
    error_code = i2c_master_write_byte(cmdLink, 0x00, false);
    if (error_code != ESP_OK) goto end;
    error_code = i2c_master_stop(cmdLink);
    if (error_code != ESP_OK) goto end;
    error_code = i2c_master_cmd_begin(i2c_num, cmdLink, pdMS_TO_TICKS(timeout));
    if (error_code != ESP_OK) goto end;
  };
end:  
  // Unlock bus and release resources
  if (error_code != ESP_OK) {
    rlog_e(logTAG, ERROR_I2C_WRITE, i2c_num, i2c_address, error_code, esp_err_to_name(error_code));
  };
  _i2c_cmd_link_delete(i2c_num, cmdLink);
  giveI2C(i2c_num);
  return error_code;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ Read and write macros ------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

esp_err_t readI2C(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  const uint32_t wait_data_us, const TickType_t timeout)
{
  // Lock bus
  takeI2C(i2c_num);
  esp_err_t error_code = ESP_ERR_NO_MEM;
  i2c_cmd_handle_t cmdLink = nullptr;
  // Create commands link
  cmdLink = _i2c_cmd_link_create(i2c_num);
  if (cmdLink) {
    // Send commands, if needed
    if ((cmds) && (cmds_size > 0)) {
      // Add start bit
      error_code = i2c_master_start(cmdLink);
      if (error_code != ESP_OK) goto error_write;
      // Add address and mode
      error_code = i2c_master_write_byte(cmdLink, (i2c_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
      if (error_code != ESP_OK) goto error_write;
      // Add commands
      error_code = i2c_master_write(cmdLink, cmds, cmds_size, ACK_CHECK_EN);
      if (error_code != ESP_OK) goto error_write;
      // If there is a pause for waiting for data, we immediately send commands and release the bus
      if (wait_data_us > 0) {
        // Add stop bit
        error_code = i2c_master_stop(cmdLink);
        if (error_code != ESP_OK) goto error_write;
        // Execute packet
        error_code = i2c_master_cmd_begin(i2c_num, cmdLink, pdMS_TO_TICKS(timeout));
        if (error_code != ESP_OK) goto error_write;
        // Release resources
        _i2c_cmd_link_delete(i2c_num, cmdLink);
        cmdLink = nullptr;
        // Wait...
        ets_delay_us(wait_data_us);
        // Initializing a new packet of commands
        cmdLink = _i2c_cmd_link_create(i2c_num);
      };
    };
    // Reading data
    if (cmdLink) {
      // Add start bit
      error_code = i2c_master_start(cmdLink);
      if (error_code != ESP_OK) goto error_read;
      // Add address and mode
      error_code = i2c_master_write_byte(cmdLink, (i2c_address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
      if (error_code != ESP_OK) goto error_read;
      // Reading data
      error_code = i2c_master_read(cmdLink, data, data_size, I2C_MASTER_LAST_NACK);
      if (error_code != ESP_OK) goto error_read;
      // Add stop bit
      error_code = i2c_master_stop(cmdLink);
      if (error_code != ESP_OK) goto error_read;
      // Execute packet
      error_code = i2c_master_cmd_begin(i2c_num, cmdLink, pdMS_TO_TICKS(timeout));
      if (error_code != ESP_OK) goto error_read;
    }
    else error_code = ESP_ERR_NO_MEM;
  };
end:  
  // Unlock bus and release resources
  _i2c_cmd_link_delete(i2c_num, cmdLink);
  giveI2C(i2c_num);
  return error_code;
// Show write error log and exit
error_write:
  rlog_e(logTAG, ERROR_I2C_WRITE, i2c_num, i2c_address, error_code, esp_err_to_name(error_code));
  goto end;
// Show read error log and exit
error_read:
  rlog_e(logTAG, ERROR_I2C_READ, i2c_num, i2c_address, error_code, esp_err_to_name(error_code));
  goto end;
}

esp_err_t readI2C_CRC8(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  const uint32_t wait_data_us, const uint8_t crc_init, const TickType_t timeout)
{
  esp_err_t err = readI2C(i2c_num, i2c_address, cmds, cmds_size, data, data_size, wait_data_us, timeout);
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

esp_err_t writeI2C(i2c_port_t i2c_num, const uint8_t i2c_address, 
  uint8_t* cmds, const size_t cmds_size,
  uint8_t* data, const size_t data_size, 
  TickType_t timeout)
{
  takeI2C(i2c_num);
  esp_err_t error_code = ESP_ERR_NO_MEM;
  i2c_cmd_handle_t cmdLink = _i2c_cmd_link_create(i2c_num);
  if (cmdLink) {
    error_code = i2c_master_start(cmdLink);
    if (error_code != ESP_OK) goto end;
    error_code = i2c_master_write_byte(cmdLink, (i2c_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    if (error_code != ESP_OK) goto end;
    if ((cmds) && (cmds_size > 0)) {
      error_code = i2c_master_write(cmdLink, cmds, cmds_size, ACK_CHECK_EN);
      if (error_code != ESP_OK) goto end;
    };
    if ((data) && (data_size>0)) {
      error_code = i2c_master_write(cmdLink, data, data_size, ACK_CHECK_EN);
      if (error_code != ESP_OK) goto end;
    };
    error_code = i2c_master_stop(cmdLink);
    if (error_code != ESP_OK) goto end;
    error_code = i2c_master_cmd_begin(i2c_num, cmdLink, pdMS_TO_TICKS(timeout));
    if (error_code != ESP_OK) goto end;
  };
end:  
  // Unlock bus and release resources
  if (error_code != ESP_OK) {
    rlog_e(logTAG, ERROR_I2C_WRITE, i2c_num, i2c_address, error_code, esp_err_to_name(error_code));
  };
  _i2c_cmd_link_delete(i2c_num, cmdLink);
  giveI2C(i2c_num);
  return error_code;
}

