#include "i2c_bus_tools.h"

static const char *TAG = "i2c_bus_tools";

// 定义全局变量
i2c_bus_handle_t g_i2c_bus = NULL;
i2c_bus_device_handle_t i2c_device_all[I2C_SCAN_ADDR_NUM] = {NULL};

void register_i2c_bus_device(i2c_bus_handle_t bus, uint32_t speed , gpio_num_t i2c_sda_pin , gpio_num_t i2c_scl_pin)
{
    // Initialize I2C bus    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_sda_pin,
        .scl_io_num = i2c_scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = speed,
        },
        .clk_flags = 0
    };
    g_i2c_bus = i2c_bus_create(I2C_NUM_0, &conf);
    if (g_i2c_bus == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C bus");
        return;
    }
    else {
        ESP_LOGI(TAG, "I2C bus created successfully");
    }

}

void register_i2c_bus_device(void)
{
    if (g_i2c_bus != NULL) {
        ESP_LOGE(TAG, "I2C bus already registered");
        return;
    }

    // Register I2C bus device
    register_i2c_bus_device(g_i2c_bus, I2C_MASTER_FREQ_HZ, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    
    // Initialize I2C devices
    for (int i = 0; i < I2C_SCAN_ADDR_NUM; i++) {
        i2c_device_all[i] = NULL;
    }
    
    ESP_LOGI(TAG, "I2C bus and devices initialized successfully");
}

void unregister_i2c_bus_device(i2c_bus_handle_t bus)
{
    if (bus == NULL) {
        ESP_LOGE(TAG, "I2C bus handle is NULL");
        return;
    }

    // Unregister I2C bus device
    i2c_bus_delete(&bus);
    g_i2c_bus = NULL;

    // Unregister all I2C devices
    for (int i = 0; i < I2C_SCAN_ADDR_NUM; i++) {
        if (i2c_device_all[i] != NULL) {
            i2c_bus_device_delete(&i2c_device_all[i]);
            i2c_device_all[i] = NULL;
        }
    }

    ESP_LOGI(TAG, "I2C bus and devices unregistered successfully");
}

void unregister_i2c_bus_device(void)
{
    if (g_i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not registered");
        return;
    }

    // Delete I2C bus
    i2c_bus_delete(&g_i2c_bus);
    g_i2c_bus = NULL;

    // Delete all I2C devices
    for (int i = 0; i < I2C_SCAN_ADDR_NUM; i++) {
        if (i2c_device_all[i] != NULL) {
            i2c_bus_device_delete(&i2c_device_all[i]);
            i2c_device_all[i] = NULL;
        }
    }

    ESP_LOGI(TAG, "I2C bus and devices unregistered successfully");
}

void scan_and_add_i2c_bus_devices(void)
{
    if (g_i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return;
    }


    // SCAN
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    // Scan I2C bus for devices
    uint8_t addrs[I2C_SCAN_ADDR_NUM] = {0};
    uint8_t i2c_device_num = 0;
    i2c_device_num = i2c_bus_scan(g_i2c_bus, addrs, I2C_SCAN_ADDR_NUM);

    // ADD
    for (uint8_t i = 0; i < i2c_device_num; i++) {
        uint8_t addr = addrs[i];
        if (addr < I2C_SCAN_ADDR_NUM && i2c_device_all[addr] == NULL) {
            i2c_device_all[addr] = i2c_bus_device_create(g_i2c_bus, addr, I2C_MASTER_FREQ_HZ);
            if (i2c_device_all[addr] != NULL) {
                ESP_LOGI(TAG, "I2C device found at address 0x%02X", addr);
            } else {
                ESP_LOGE(TAG, "Failed to create I2C device at address 0x%02X", addr);
            }
        }
    }

    ESP_LOGI(TAG, "I2C bus scan completed");
}