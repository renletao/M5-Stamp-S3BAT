/**
 * @file stp2_basic_example.c
 * @brief STP2 Basic Usage Example
 * 
 * This example demonstrates basic usage of the STP2 component:
 * - I2C initialization
 * - STP2 initialization
 * - GPIO control
 * - ADC reading
 * - PWM output
 * - Voltage monitoring
 * 
 * @note This is just an example code snippet, not a complete main application
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "stp2_control.h"
#include "i2c_bus.h"

static const char *TAG = "STP2_EXAMPLE";

void stp2_example_task(void)
{
    // I2C Configuration
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,      // Adjust according to your hardware
        .scl_io_num = GPIO_NUM_22,      // Adjust according to your hardware
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,     // 100KHz
    };

    // Create I2C bus
    i2c_bus_handle_t i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_config);
    if (i2c_bus == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C bus");
        return;
    }

    // Initialize STP2
    esp_err_t ret = stp2_init(i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize STP2: %s", esp_err_to_name(ret));
        i2c_bus_delete(&i2c_bus);
        return;
    }

    ESP_LOGI(TAG, "STP2 initialized successfully");

    // Example 1: GPIO Control
    ESP_LOGI(TAG, "=== GPIO Example ===");
    
    // Set GPIO0 as output and turn it on
    stp2_gpio_set_func(STP2_GPIO_NUM_0, STP2_GPIO_FUNC_GPIO);
    stp2_gpio_set(STP2_GPIO_NUM_0, STP2_GPIO_MODE_OUTPUT, 
                       STP2_GPIO_OUTPUT_HIGH, STP2_GPIO_PUPD_NC);
    ESP_LOGI(TAG, "GPIO0 set to output HIGH");

    // Set GPIO1 as input with pull-up
    stp2_gpio_set_func(STP2_GPIO_NUM_1, STP2_GPIO_FUNC_GPIO);
    stp2_gpio_set(STP2_GPIO_NUM_1, STP2_GPIO_MODE_INPUT, 
                       STP2_GPIO_INPUT_NC, STP2_GPIO_PUPD_PULLUP);
    ESP_LOGI(TAG, "GPIO1 set to input with pull-up");

    // Read GPIO1 state
    stp2_gpio_in_state_t gpio1_state;
    ret = stp2_gpio_get_in_state(STP2_GPIO_NUM_1, &gpio1_state);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "GPIO1 state: %s", 
                 gpio1_state == STP2_GPIO_IN_STATE_HIGH ? "HIGH" : "LOW");
    }

    // Example 2: ADC Reading
    ESP_LOGI(TAG, "=== ADC Example ===");
    
    // Read ADC Channel 1
    uint32_t adc_value;
    ret = stp2_adc_read(STP2_ADC_CHANNEL_1, &adc_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC Channel 1: %lu (0-4095)", adc_value);
    }

    // Read internal temperature
    uint32_t temperature;
    ret = stp2_adc_read(STP2_ADC_CHANNEL_TEMP, &temperature);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Internal Temperature: %lu°C", temperature);
    }

    // Example 3: PWM Output
    ESP_LOGI(TAG, "=== PWM Example ===");
    
    // Set GPIO2 to PWM function
    stp2_gpio_set_func(STP2_GPIO_NUM_2, STP2_GPIO_FUNC_OTHER);
    
    // Configure PWM0: 1KHz, 25% duty cycle
    uint32_t pwm_freq = 1000;      // 1KHz
    uint32_t duty_cycle = 1024;    // 25% of 4095
    
    ret = stp2_pwm_set(STP2_PWM_CHANNEL_0, STP2_PWM_CTRL_ENABLE, 
                       STP2_PWM_POLARITY_NORMAL, pwm_freq, duty_cycle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PWM0 set to %lu Hz, %lu%% duty cycle", 
                 pwm_freq, (duty_cycle * 100) / 4095);
    }

    // Example 4: Voltage Monitoring
    ESP_LOGI(TAG, "=== Voltage Monitoring Example ===");
    
    // Read battery voltage
    uint32_t battery_voltage;
    ret = stp2_vbat_read(&battery_voltage);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Battery Voltage: %lu mV", battery_voltage);
    }

    // Read input voltage
    uint32_t input_voltage;
    ret = stp2_vin_read(&input_voltage);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Input Voltage: %lu mV", input_voltage);
    }

    // Read VBus voltage
    uint32_t vbus_voltage;
    ret = stp2_vbus_read(&vbus_voltage);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "VBus Voltage: %lu mV", vbus_voltage);
    }

    // Read power source
    stp2_pwr_src_t power_source;
    ret = stp2_pwr_src_read(&power_source);
    if (ret == ESP_OK) {
        const char *pwr_src_str;
        switch (power_source) {
            case STP2_PWR_SRC_5VIN:    pwr_src_str = "5VIN"; break;
            case STP2_PWR_SRC_5VINOUT: pwr_src_str = "5VINOUT"; break;
            case STP2_PWR_SRC_BAT:     pwr_src_str = "Battery"; break;
            default:                   pwr_src_str = "Unknown"; break;
        }
        ESP_LOGI(TAG, "Power Source: %s", pwr_src_str);
    }

    // Example 5: Continuous monitoring loop
    ESP_LOGI(TAG, "=== Starting continuous monitoring ===");
    
    for (int i = 0; i < 10; i++) {
        // Toggle GPIO0
        static bool gpio0_state = false;
        gpio0_state = !gpio0_state;
        stp2_gpio_set(STP2_GPIO_NUM_0, STP2_GPIO_MODE_OUTPUT, 
                           gpio0_state ? STP2_GPIO_OUTPUT_HIGH : STP2_GPIO_OUTPUT_LOW, 
                           STP2_GPIO_PUPD_NC);
        
        // Read temperatures
        ret = stp2_adc_read(STP2_ADC_CHANNEL_TEMP, &temperature);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Loop %d: GPIO0=%s, Temp=%lu°C", 
                     i, gpio0_state ? "HIGH" : "LOW", temperature);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
    }

    ESP_LOGI(TAG, "Example completed, cleaning up...");

    // Cleanup
    stp2_deinit();
    i2c_bus_delete(&i2c_bus);
    
    ESP_LOGI(TAG, "Cleanup completed");
}

/**
 * @brief Example main function (you would adapt this to your main app)
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting STP2 Basic Example");
    stp2_example_task();
    ESP_LOGI(TAG, "STP2 Basic Example finished");
}
