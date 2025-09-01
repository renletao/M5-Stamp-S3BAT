#ifndef __I2C_BUS_TOOLS_H__
#define __I2C_BUS_TOOLS_H__

#include "esp_idf_version.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
  #include <driver/i2c_master.h>
  #include <driver/i2c_types.h>
#else
  #include <driver/i2c.h>
#endif

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_bus.h"

// #define I2C_MASTER_SDA_IO GPIO_NUM_29
// #define I2C_MASTER_SCL_IO GPIO_NUM_24

#define I2C_MASTER_SDA_IO GPIO_NUM_48
#define I2C_MASTER_SCL_IO GPIO_NUM_47

#define I2C_MASTER_FREQ_HZ 400000
#define I2C_SCAN_ADDR_NUM 128

extern i2c_bus_handle_t g_i2c_bus; // I2C bus handle
extern i2c_bus_device_handle_t i2c_device_all[I2C_SCAN_ADDR_NUM];

void register_i2c_bus_device(void);
void register_i2c_bus_device(i2c_bus_handle_t bus, uint32_t speed , gpio_num_t i2c_sda_pin , gpio_num_t i2c_scl_pin);
void unregister_i2c_bus_device(i2c_bus_handle_t bus);
void scan_and_add_i2c_bus_devices(void);



#endif // __I2C_BUS_TOOLS_H__