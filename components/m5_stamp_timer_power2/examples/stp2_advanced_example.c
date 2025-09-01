/**
 * @file stp2_advanced_example.c
 * @brief STP2 Advanced Usage Example
 * 
 * This example demonstrates advanced features of the STP2 component:
 * - Power management
 * - Watchdog timer
 * - Interrupt handling
 * - Timer and wake functions
 * - NeoPixel control
 * - RTC RAM storage
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "stp2_control.h"
#include "i2c_bus.h"

static const char *TAG = "STP2_ADVANCED";

void stp2_advanced_example_task(void)
{
    ESP_LOGI(TAG, "=== STP2 Advanced Features Example ===");

    // Initialize I2C and STP2 (same as basic example)
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_bus_handle_t i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_config);
    if (i2c_bus == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C bus");
        return;
    }

    esp_err_t ret = stp2_init(i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize STP2: %s", esp_err_to_name(ret));
        i2c_bus_delete(&i2c_bus);
        return;
    }

    // Example 1: Power Management
    ESP_LOGI(TAG, "=== Power Management Example ===");
    
    // Get current power configuration
    uint8_t current_pwr_cfg;
    ret = stp2_pwr_get_cfg(&current_pwr_cfg);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Current power config: 0x%02X", current_pwr_cfg);
    }

    // Enable charging
    uint8_t final_cfg;
    ret = stp2_pwr_set_cfg(STP2_PWR_CFG_CHG_EN, &final_cfg);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Charging enabled. Final config: 0x%02X", final_cfg);
    }

    // Example 2: Watchdog Timer
    ESP_LOGI(TAG, "=== Watchdog Timer Example ===");
    
    // Set watchdog timeout to 30 seconds
    ret = stp2_wdt_set(STP2_WDT_CTRL_ENABLE, 30);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Watchdog timer set to 30 seconds");
        
        // Feed the watchdog a few times
        for (int i = 0; i < 3; i++) {
            vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds
            ret = stp2_wdt_feed(0xA5);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Watchdog fed %d/3", i + 1);
            }
        }
        
        // Disable watchdog
        stp2_wdt_set(STP2_WDT_CTRL_DISABLE, 0);
        ESP_LOGI(TAG, "Watchdog disabled");
    }

    // Example 3: GPIO Interrupts and Wake
    ESP_LOGI(TAG, "=== GPIO Interrupt Example ===");
    
    // Configure GPIO4 for wake on rising edge
    stp2_gpio_set_func(STP2_GPIO_NUM_4, STP2_GPIO_FUNC_WAKE);
    stp2_gpio_set_wake_en(STP2_GPIO_NUM_4, STP2_GPIO_WAKE_ENABLE);
    stp2_gpio_set_wake_cfg(STP2_GPIO_NUM_4, STP2_GPIO_WAKE_RISING);
    ESP_LOGI(TAG, "GPIO4 configured for wake on rising edge");

    // Check for GPIO interrupts
    stp2_irq_gpio_t gpio_irq;
    ret = stp2_irq_get_status(&gpio_irq, STP2_ADDR_IRQ_GPIO_NOT_CLEAN);
    if (ret == ESP_OK) {
        if (gpio_irq != STP2_ADDR_IRQ_NULL) {
            ESP_LOGI(TAG, "GPIO interrupt detected on GPIO%d", gpio_irq);
        } else {
            ESP_LOGI(TAG, "No GPIO interrupts pending");
        }
    }

    // Check for system interrupts (power events)
    stp2_irq_sys_t sys_irq;
    ret = stp2_irq_get_sys_status(&sys_irq, STP2_ADDR_IRQ_SYS_NOT_CLEAN);
    if (ret == ESP_OK) {
        if (sys_irq != STP2_ADDR_IRQ_SYS_NULL) {
            const char *irq_name;
            switch (sys_irq) {
                case STP2_ADDR_IRQ_SYS_5VIN_INSERT:   irq_name = "5VIN Insert"; break;
                case STP2_ADDR_IRQ_SYS_5VIN_REMOVE:   irq_name = "5VIN Remove"; break;
                case STP2_ADDR_IRQ_SYS_5VINOUT_INSERT: irq_name = "5VINOUT Insert"; break;
                case STP2_ADDR_IRQ_SYS_5VINOUT_REMOVE: irq_name = "5VINOUT Remove"; break;
                case STP2_ADDR_IRQ_SYS_BAT_INSERT:    irq_name = "Battery Insert"; break;
                case STP2_ADDR_IRQ_SYS_BAT_REMOVE:    irq_name = "Battery Remove"; break;
                default:                              irq_name = "Unknown"; break;
            }
            ESP_LOGI(TAG, "System interrupt: %s", irq_name);
        } else {
            ESP_LOGI(TAG, "No system interrupts pending");
        }
    }

    // Example 4: Timer Wake Function
    ESP_LOGI(TAG, "=== Timer Wake Example ===");
    
    // Set timer to wake up in 60 seconds
    uint32_t wake_time = 60; // seconds
    ret = stp2_tim_set(STP2_ADDR_TIM_ENABLE, STP2_TIM_ACTION_001, wake_time);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Timer wake set for %lu seconds", wake_time);
        
        // Wait a bit then clear the timer
        vTaskDelay(pdMS_TO_TICKS(2000));
        ret = stp2_tim_clear(0xA5);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Timer cleared");
        }
    }

    // Example 5: NeoPixel Control
    ESP_LOGI(TAG, "=== NeoPixel Example ===");
    
    // Configure 3 NeoPixels with RGB565 data
    uint16_t neo_data[3] = {
        0xF800,  // Red (RGB565: 11111 000000 00000)
        0x07E0,  // Green (RGB565: 00000 111111 00000)
        0x001F   // Blue (RGB565: 00000 000000 11111)
    };
    
    ret = stp2_neo_set_cfg(3, neo_data, 1); // 3 pixels, auto-refresh
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NeoPixels set to Red, Green, Blue");
        
        vTaskDelay(pdMS_TO_TICKS(2000)); // Show for 2 seconds
        
        // Turn off all pixels
        uint16_t off_data[3] = {0x0000, 0x0000, 0x0000};
        stp2_neo_set_cfg(3, off_data, 1);
        ESP_LOGI(TAG, "NeoPixels turned off");
    }

    // Example 6: RTC RAM Storage
    ESP_LOGI(TAG, "=== RTC RAM Example ===");
    
    // Write data to RTC RAM
    uint8_t write_data[] = "Hello STP2!";
    uint8_t write_len = sizeof(write_data);
    
    ret = stp2_rtc_ram_write(0, write_len, write_data);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Data written to RTC RAM");
        
        // Read data back
        uint8_t read_data[32];
        ret = stp2_rtc_ram_read(0, write_len, read_data);
        if (ret == ESP_OK) {
            read_data[write_len-1] = '\\0'; // Ensure null termination
            ESP_LOGI(TAG, "Data read from RTC RAM: %s", (char*)read_data);
        }
    }

    // Example 7: Battery Monitoring with Low Voltage Protection
    ESP_LOGI(TAG, "=== Battery Protection Example ===");
    
    // Set low voltage protection to 3.2V (3200mV)
    ret = stp2_batt_set_lvp(3200);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Low voltage protection set to 3.2V");
    }

    // Monitor battery for a while
    ESP_LOGI(TAG, "Monitoring battery voltage...");
    for (int i = 0; i < 5; i++) {
        uint32_t vbat, vref;
        
        ret = stp2_vbat_read(&vbat);
        if (ret == ESP_OK) {
            stp2_vref_read(&vref);
            ESP_LOGI(TAG, "Battery: %lu mV, VRef: %lu mV", vbat, vref);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Example 8: Button Configuration
    ESP_LOGI(TAG, "=== Button Configuration Example ===");
    
    // Configure single click with 250ms delay
    ret = stp2_btn_set_cfg(STP2_ADDR_BTN_TYPE_CLICK, 
                           STP2_ADDR_BTN_CLICK_DELAY_250MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Button configured for single click (250ms)");
    }

    // Configure double click with 500ms delay
    ret = stp2_btn_set_cfg(STP2_ADDR_BTN_TYPE_DOUBLE_CLICK, 
                           STP2_ADDR_BTN_DOUBLE_CLICK_DELAY_500MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Button configured for double click (500ms)");
    }

    // Configure long press with 2000ms delay
    ret = stp2_btn_set_cfg(STP2_ADDR_BTN_TYPE_LONG_PRESS, 
                           STP2_ADDR_BTN_LONG_PRESS_DELAY_2000MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Button configured for long press (2000ms)");
    }

    ESP_LOGI(TAG, "Advanced example completed, cleaning up...");

    // Cleanup
    stp2_deinit();
    i2c_bus_delete(&i2c_bus);
    
    ESP_LOGI(TAG, "Advanced example finished");
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting STP2 Advanced Example");
    stp2_advanced_example_task();
}
