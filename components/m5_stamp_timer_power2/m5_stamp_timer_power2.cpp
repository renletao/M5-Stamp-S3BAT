#include "m5_stamp_timer_power2.h"

#include "esp_log.h"
#include "esp_err.h"
// #include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 全局记录上次设置的PWM频率
static uint32_t stp2_last_pwm_freq = 0;

// Internal I2C handles - managed by the component
static i2c_bus_handle_t stp2_i2c_bus = NULL;
static i2c_bus_device_handle_t stp2_i2c_device = NULL;
static bool stp2_device_external = false;  // Flag to track if device was created externally
static const char *TAG = "stp2";

//记录引脚模式，状态
struct {
    stp2_gpio_func_t pin_mode; // 引脚模式
    stp2_gpio_state_t pin_state; // 引脚状态
    stp2_gpio_pupd_t pupd; // 上下拉配置
    stp2_gpio_wake_t wake_en; // 唤醒使能
    stp2_gpio_wake_edge_t wake_cfg; // 唤醒配置
} stp2_pin_status[5];

// stp2 I2C Init
esp_err_t stp2_init(i2c_bus_handle_t i2c_bus, i2c_bus_device_handle_t *i2c_device) 
{
    if (i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Store the I2C bus handle
    stp2_i2c_bus = i2c_bus;

    esp_err_t ret;

    // 配置 I2C频率
    uint32_t stp2_current_clk = i2c_bus_get_current_clk_speed(stp2_i2c_bus);
    if (stp2_current_clk != 100000) {
        // 销毁句柄
        if (i2c_device != NULL) {
            esp_err_t ret = i2c_bus_device_delete(i2c_device);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to delete existing stp2 I2C device: %s", esp_err_to_name(ret));
                return ret;
            }
            i2c_device = NULL;
        }
        // 创建新的 I2C 设备
        stp2_i2c_device = i2c_bus_device_create(stp2_i2c_bus, I2C_PY32F002_ADDR, 100000);
        if (stp2_i2c_device == NULL) {
            ESP_LOGE(TAG, "Failed to create stp2 I2C device");
            return ESP_ERR_NO_MEM;
        }
        i2c_device = &stp2_i2c_device;
    }
    
    // Mark device as internally created
    stp2_device_external = false;

    // Validate I2C frequency setting
#if I2C_PY32F002_FREQ_HZ != 100000 && I2C_PY32F002_FREQ_HZ != 400000
    ESP_LOGE(TAG, "Unsupported I2C frequency: %d Hz. Supported frequencies are 100000 Hz and 400000 Hz.", I2C_PY32F002_FREQ_HZ);
    i2c_bus_device_delete(&stp2_i2c_device);
    stp2_i2c_device = NULL;
    return ESP_ERR_INVALID_ARG;
#elif I2C_PY32F002_FREQ_HZ == 400000
    // Set clock speed to 400kHz if configured
    stp2_set_clk_speed(STP2_CLK_SPEED_400KHZ);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for the device to be ready
    
    // Recreate device with 400KHz frequency
    ret = i2c_bus_device_delete(&stp2_i2c_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete stp2 I2C device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    stp2_i2c_device = i2c_bus_device_create(stp2_i2c_bus, I2C_PY32F002_ADDR, 400000);
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "Failed to create stp2 I2C device at 400KHz");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "stp2 I2C device created at 400KHz");
    i2c_device = &stp2_i2c_device;
#endif

    // Read hardware version
    uint8_t hw_version = 0;
    ret = stp2_get_hw_version(&hw_version);
    if (ret != ESP_OK) {
        i2c_bus_device_delete(&stp2_i2c_device);
        stp2_i2c_device = NULL;
        return ret;
    }
    
    // Read software version
    uint8_t sw_version = 0;
    ret = stp2_get_sw_version(&sw_version);
    if (ret != ESP_OK) {
        i2c_bus_device_delete(&stp2_i2c_device);
        stp2_i2c_device = NULL;
        return ret;
    }
    
    // Read chip UID
    uint16_t chip_id = 0;
    ret = stp2_get_chip_id(&chip_id);
    if (ret != ESP_OK) {
        i2c_bus_device_delete(&stp2_i2c_device);
        stp2_i2c_device = NULL;
        return ret;
    }
    
    // Initialize pin status array
    for (int i = 0; i < 5; i++) {
        stp2_pin_status[i].pin_mode = STP2_GPIO_FUNC_GPIO;
        stp2_pin_status[i].pin_state = STP2_GPIO_INPUT_NC;
        stp2_pin_status[i].pupd = STP2_GPIO_PUPD_NC;
        stp2_pin_status[i].wake_en = STP2_GPIO_WAKE_DISABLE;
        stp2_pin_status[i].wake_cfg = STP2_GPIO_WAKE_FALLING;
    }
    
    ESP_LOGI(TAG, "STP2 Hardware Version: %d, Software Version: %d, Chip ID: 0x%04X", hw_version, sw_version, chip_id);
    ESP_LOGI(TAG, "stp2 initialized successfully");
    
    return ESP_OK;
}

// stp2 Deinit
esp_err_t stp2_deinit(void)
{
    esp_err_t ret = ESP_OK;
    
    if (stp2_i2c_device != NULL) {
        // Only delete device if it was created internally
        if (!stp2_device_external) {
            ret = i2c_bus_device_delete(&stp2_i2c_device);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to delete stp2 I2C device: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG, "stp2 I2C device deleted successfully");
            }
        } else {
            ESP_LOGI(TAG, "stp2 I2C device was external, not deleting");
        }
        stp2_i2c_device = NULL;
    }
    
    stp2_i2c_bus = NULL;
    stp2_device_external = false;
    stp2_last_pwm_freq = 0;
    
    // Clear pin status
    for (int i = 0; i < 5; i++) {
        stp2_pin_status[i].pin_mode = STP2_GPIO_FUNC_GPIO;
        stp2_pin_status[i].pin_state = STP2_GPIO_INPUT_NC;
        stp2_pin_status[i].pupd = STP2_GPIO_PUPD_NC;
        stp2_pin_status[i].wake_en = STP2_GPIO_WAKE_DISABLE;
        stp2_pin_status[i].wake_cfg = STP2_GPIO_WAKE_FALLING;
    }
    
    ESP_LOGI(TAG, "stp2 deinitialized successfully");
    return ret;
}


// [STP2_ADDR_HW_REV]
// 读取硬件版本号
// INPUT: uint8_t *hw_version (用来存储读取的硬件版本号)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t stp2_get_hw_version(uint8_t *hw_version)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_get_hw_version -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (hw_version == NULL) {
        ESP_LOGE(TAG, "Hardware version pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_HW_REV, hw_version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read STP2 hardware version: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "STP2 Hardware Version: %d", *hw_version);
    return ESP_OK;
}

// [STP2_ADDR_SW_REV]
// 读取软件版本号
// INPUT: uint8_t *sw_version (用来存储读取的软件版本号)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t stp2_get_sw_version(uint8_t *sw_version)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_get_sw_version -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sw_version == NULL) {
        ESP_LOGE(TAG, "Software version pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_SW_REV, sw_version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read STP2 software version: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "STP2 Software Version: %d", *sw_version);
    return ESP_OK;
}

// [STP2_ADDR_UID_L/H]
// 读取芯片唯一ID
// INPUT: uint16_t *chip_id (用来存储读取的芯片ID)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t stp2_get_chip_id(uint16_t *chip_id)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_get_chip_id -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (chip_id == NULL) {
        ESP_LOGE(TAG, "Chip ID pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t chip_id_l = 0;
    uint8_t chip_id_h = 0;
    esp_err_t ret;

    // 读取芯片ID低字节
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_UID_L, &chip_id_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read STP2 chip ID low byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取芯片ID高字节
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_UID_H, &chip_id_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read STP2 chip ID high byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 组合成16位芯片ID
    *chip_id = (chip_id_h << 8) | chip_id_l;
    
    ESP_LOGI(TAG, "STP2 Chip ID: 0x%04X", *chip_id);
    return ESP_OK;
}

// [STP2_ADDR_I2C_CFG]
// 配置I2C时钟速度
// INPUIT: stp2_clk_speed_t clk_speed_flag
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_set_clk_speed(stp2_clk_speed_t clk_speed_flag)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_set_clk_speed -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t read_buf = 0;
    esp_err_t ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_I2C_CFG, &read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read STP2 I2C configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    else
    {
        ESP_LOGI(TAG, "Current STP2 I2C configuration: 0x%02X:0x%02X", STP2_ADDR_I2C_CFG, read_buf);
    }

    // 配置第[4]位，0=100KHz, 1=400KHz
    if (clk_speed_flag == STP2_CLK_SPEED_400KHZ) {
        read_buf |= (1 << 4);
    } else {
        read_buf &= ~(1 << 4);
    }

    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_I2C_CFG, read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write STP2 I2C configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "STP2 I2C configuration set to %s successfully: 0x%02X:0x%02X",
             (clk_speed_flag == STP2_CLK_SPEED_400KHZ) ? "400K" : "100K",
             STP2_ADDR_I2C_CFG, read_buf);

    return ESP_OK;
}

// [STP2_ADDR_I2C_CFG]
// 配置I2C进入休眠的时长，休眠后，需要将SDA引脚拉低，才能唤醒
// INPUIT: uint8_t sleep_time_sec [0-15]
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_set_i2c_sleep_time(uint8_t sleep_time_sec)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_set_i2c_sleep_time -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 检查参数有效性
    if (sleep_time_sec > 15) {
        ESP_LOGE(TAG, "Invalid sleep time: %d. Valid range: 0-15 seconds", sleep_time_sec);
        return ESP_ERR_INVALID_ARG;
    }

    if(sleep_time_sec != 0)
    {
        // 警告不支持自动休眠唤醒，需要手动通过stp2_i2c_try_wake函数唤醒
        ESP_LOGW(TAG, "Automatic wake-up after sleep is not supported. Please use stp2_i2c_try_wake to wake up the stp2 device.");
    }

    uint8_t read_buf = 0;
    esp_err_t ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_I2C_CFG, &read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read STP2 I2C configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    else
    {
        ESP_LOGI(TAG, "Current STP2 I2C configuration: 0x%02X:0x%02X", STP2_ADDR_I2C_CFG, read_buf);
        ESP_LOGI(TAG, "Current sleep timeout: %d seconds", read_buf & 0x0F);
    }

    // 配置第[3-0]位，SLP_TO(4-bit 0-15 s，0=禁用)
    // 保持第[4]位SPD速度控制位不变
    read_buf &= 0xF0;  // 清除低4位，保持高4位
    read_buf |= (sleep_time_sec & 0x0F);  // 设置新的休眠时间

    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_I2C_CFG, read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write STP2 I2C configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    if (sleep_time_sec == 0) {
        ESP_LOGI(TAG, "STP2 I2C sleep disabled successfully: 0x%02X:0x%02X", STP2_ADDR_I2C_CFG, read_buf);
    } else {
        ESP_LOGI(TAG, "STP2 I2C sleep timeout set to %d seconds successfully: 0x%02X:0x%02X", 
                 sleep_time_sec, STP2_ADDR_I2C_CFG, read_buf);
    }

    return ESP_OK;
}

// [I2C_TOOL_FUNCTION]
// I2C工具函数，用于唤醒I2C总线
// INPUIT: stp2_i2c_ack_check_t ack_check_type uint32_t timeout_ms_or_try_times
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_i2c_try_wake(stp2_i2c_ack_check_t ack_check_type, uint32_t timeout_ms_or_try_times)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_i2c_try_wake -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_FAIL;
    uint8_t dummy_read = 0;
    uint32_t try_count = 0;

    if (ack_check_type == STP2_I2C_ACK_CHECK_UNTIL_WAKE) {
        // 模式0: 持续尝试直到唤醒成功，支持超时设置
        if (timeout_ms_or_try_times == 0) {
            ESP_LOGI(TAG, "Trying to wake I2C device until success (no timeout)...");
        } else {
            ESP_LOGI(TAG, "Trying to wake I2C device until success (timeout: %lu ms)...", timeout_ms_or_try_times);
        }
        
        uint32_t start_time = xTaskGetTickCount();
        uint32_t timeout_ticks = (timeout_ms_or_try_times == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms_or_try_times);
        
        while (1) {
            try_count++;
            // 尝试读取硬件版本寄存器来唤醒设备
            ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_HW_REV, &dummy_read);
            
            if (ret == ESP_OK) {
                uint32_t elapsed_time = pdTICKS_TO_MS(xTaskGetTickCount() - start_time);
                ESP_LOGI(TAG, "I2C device wake up successful after %lu attempts (%lu ms)", try_count, elapsed_time);
                break;
            }
            
            // 检查超时
            if (timeout_ms_or_try_times > 0) {
                uint32_t elapsed_time = xTaskGetTickCount() - start_time;
                if (elapsed_time >= timeout_ticks) {
                    ESP_LOGE(TAG, "I2C device wake up timeout after %lu attempts (%lu ms)", try_count, pdTICKS_TO_MS(elapsed_time));
                    ret = ESP_ERR_TIMEOUT;
                    break;
                }
            }
            
            // 延迟一段时间再尝试
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    } else if (ack_check_type == STP2_I2C_ACK_CHECK_TRY_TIMES) {
        // 模式1: 尝试指定次数，每次间隔500ms
        ESP_LOGI(TAG, "Trying to wake I2C device %lu times with 500ms interval...", timeout_ms_or_try_times);
        
        if (timeout_ms_or_try_times == 0) {
            ESP_LOGE(TAG, "Invalid try times: 0");
            return ESP_ERR_INVALID_ARG;
        }
        
        for (try_count = 1; try_count <= timeout_ms_or_try_times; try_count++) {
            // 尝试读取硬件版本寄存器来唤醒设备
            ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_HW_REV, &dummy_read);
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "I2C device wake up successful on attempt %lu/%lu", try_count, timeout_ms_or_try_times);
                break;
            }
            
            ESP_LOGW(TAG, "I2C wake attempt %lu/%lu failed: %s", try_count, timeout_ms_or_try_times, esp_err_to_name(ret));
            
            // 如果不是最后一次尝试，则延迟500ms
            if (try_count < timeout_ms_or_try_times) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C device wake up failed after %lu attempts", timeout_ms_or_try_times);
        }
    } else {
        ESP_LOGE(TAG, "Invalid ack_check_type: %d", ack_check_type);
        return ESP_ERR_INVALID_ARG;
    }

    return ret;
}

// [STP2_ADDR_JTAG_MODE]
// 设置STP2为JTAG模式
// INPUIT: stp2_jtag_mode_t jtag_mode
// OUTPUIT: ESP_OK on success, error code on failure
// esp_err_t stp2_set_jtag_mode(stp2_jtag_mode_t jtag_mode)
// {
//     if (stp2_i2c_device == NULL) {
//         ESP_LOGE(TAG, "stp2_set_jtag_mode -> stp2 I2C device not initialized");
//         return ESP_ERR_INVALID_STATE;
//     }

//     uint8_t mode_val = (jtag_mode == STP2_JTAG_MODE_JTAG) ? 0x01 : 0x00;
//     esp_err_t ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_JTAG_MODE, mode_val);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to set STP2 JTAG mode: %s", esp_err_to_name(ret));
//         return ret;
//     }
//     ESP_LOGI(TAG, "STP2 set to %s mode successfully", (jtag_mode == STP2_JTAG_MODE_JTAG) ? "JTAG" : "Normal");
//     return ESP_OK;
// }



// [STP2_ADDR_GPIO_FUNC]
// 配置GPIO模式
// INPUIT: stp2_gpio_num_t gpio_num
//        stp2_gpio_func_t func
// OUTPUIT: ESP_OK on success, error code on failure
// TIPS: stp2_pin_status 会记录变更
esp_err_t stp2_gpio_set_func(stp2_gpio_num_t gpio_num, stp2_gpio_func_t func)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_gpio_set_func -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > STP2_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_addr, read_buf;
    uint8_t shift_bits;
    esp_err_t ret;

    // 确定寄存器和位偏移
    if (gpio_num <= STP2_GPIO_NUM_3) {
        reg_addr = STP2_ADDR_GPIO_FUNC0;
        shift_bits = gpio_num * 2;
    } else {
        reg_addr = STP2_ADDR_GPIO_FUNC1;
        shift_bits = (gpio_num - STP2_GPIO_NUM_4) * 2;
    }

    // 读取当前寄存器值
    ret = i2c_bus_read_byte(stp2_i2c_device, reg_addr, &read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO function register (0x%02X): %s", reg_addr, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO%d function: 0x%02X (bits %d-%d: %d)", 
             gpio_num, read_buf, shift_bits+1, shift_bits, (read_buf >> shift_bits) & 0x03);

    // 修改对应的位
    read_buf &= ~(0x03 << shift_bits);  // 清除对应的两位
    read_buf |= (func << shift_bits);   // 设置新的功能值

    // 写入修改后的值
    ret = i2c_bus_write_byte(stp2_i2c_device, reg_addr, read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO function register (0x%02X): %s", reg_addr, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set GPIO%d function to %d successfully: 0x%02X", gpio_num, func, read_buf);

    // 更新引脚状态记录
    stp2_pin_status[gpio_num].pin_mode = func;
    
    ESP_LOGI(TAG, "GPIO%d status: function=%d, state=%d, pupd=%d", 
             gpio_num, stp2_pin_status[gpio_num].pin_mode, 
             stp2_pin_status[gpio_num].pin_state, stp2_pin_status[gpio_num].pupd);

    return ESP_OK;
}

// [STP2_ADDR_GPIO_MODE]
// 配置GPIO输出输入
// INPUIT: stp2_gpio_num_t gpio_num
//        stp2_gpio_mode_t mode
//        stp2_gpio_state_t state
//        stp2_gpio_pupd_t pupd
//        stp2_gpio_drv_t drv_mode
// OUTPUIT: ESP_OK on success, error code on failure
// TIPS: stp2_pin_status 会记录变更
esp_err_t stp2_gpio_set(stp2_gpio_num_t gpio_num, stp2_gpio_mode_t mode, stp2_gpio_state_t state, stp2_gpio_pupd_t pupd, stp2_gpio_drv_t drv_mode)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_gpio_set -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > STP2_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t read_mode, read_out;
    esp_err_t ret;

    // 读取当前GPIO模式
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_GPIO_MODE, &read_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO mode register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO%d mode: 0x%02X (bit %d: %s)", 
             gpio_num, read_mode, gpio_num, (read_mode & (1 << gpio_num)) ? "Output" : "Input");

    // 设置GPIO模式
    if (mode == STP2_GPIO_MODE_OUTPUT) {
        read_mode |= (1 << gpio_num);   // 设置为输出模式
        
        // 如果是输出模式，还需要设置输出状态
        ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_GPIO_OUT, &read_out);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read GPIO output register: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Current GPIO%d output: 0x%02X (bit %d: %s)", 
                 gpio_num, read_out, gpio_num, (read_out & (1 << gpio_num)) ? "High" : "Low");

        // 设置输出状态
        if (state == STP2_GPIO_OUTPUT_HIGH) {
            read_out |= (1 << gpio_num);  // 设置高电平
        } else {
            read_out &= ~(1 << gpio_num); // 设置低电平
        }
        
        // 写入输出状态
        ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_GPIO_OUT, read_out);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write GPIO output register: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Set GPIO%d output to %s: 0x%02X", 
                 gpio_num, (state == STP2_GPIO_OUTPUT_HIGH) ? "High" : "Low", read_out);
    } else {
        read_mode &= ~(1 << gpio_num);  // 设置为输入模式
    }

    // 写入GPIO模式
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_GPIO_MODE, read_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO mode register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set GPIO%d mode to %s: 0x%02X", 
             gpio_num, (mode == STP2_GPIO_MODE_OUTPUT) ? "Output" : "Input", read_mode);

    // 配置GPIO上下拉
    if (pupd != STP2_GPIO_PUPD_NC) {
        uint8_t pupd_reg_addr, read_pupd;
        uint8_t shift_bits;
        
        // 确定寄存器和位偏移
        if (gpio_num <= STP2_GPIO_NUM_3) {
            pupd_reg_addr = STP2_ADDR_GPIO_PUPD0;
            shift_bits = gpio_num * 2;
        } else {
            pupd_reg_addr = STP2_ADDR_GPIO_PUPD1;
            shift_bits = (gpio_num - STP2_GPIO_NUM_4) * 2;
        }
        
        // 读取当前上下拉配置
        ret = i2c_bus_read_byte(stp2_i2c_device, pupd_reg_addr, &read_pupd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read GPIO pull-up/pull-down register (0x%02X): %s", pupd_reg_addr, esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Current GPIO%d pull config: 0x%02X (bits %d-%d: %d)", 
                 gpio_num, read_pupd, shift_bits+1, shift_bits, (read_pupd >> shift_bits) & 0x03);
        
        // 修改对应的位
        read_pupd &= ~(0x03 << shift_bits);  // 清除对应的两位
        read_pupd |= (pupd << shift_bits);   // 设置新的上下拉配置
        
        // 写入修改后的值
        ret = i2c_bus_write_byte(stp2_i2c_device, pupd_reg_addr, read_pupd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write GPIO pull-up/pull-down register (0x%02X): %s", pupd_reg_addr, esp_err_to_name(ret));
            return ret;
        }
        
        const char *pupd_str = (pupd == STP2_GPIO_PUPD_PULLUP) ? "Pull-up" : 
                               (pupd == STP2_GPIO_PUPD_PULLDOWN) ? "Pull-down" : "No-pull";
        ESP_LOGI(TAG, "Set GPIO%d pull config to %s: 0x%02X", gpio_num, pupd_str, read_pupd);
    }

    // 配置GPIO驱动模式
    uint8_t drv_reg;
    
    // 读取当前GPIO驱动模式寄存器
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_GPIO_DRV, &drv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO drive register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO drive register: 0x%02X", drv_reg);

    // 修改对应GPIO的驱动模式位
    if (drv_mode == STP2_GPIO_DRV_OPEN_DRAIN) {
        drv_reg |= (1 << gpio_num);   // 设置为开漏模式
    } else {
        drv_reg &= ~(1 << gpio_num);  // 设置为推挽模式
    }

    // 写入修改后的值
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_GPIO_DRV, drv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO drive register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set GPIO%d drive mode to %s: 0x%02X", 
             gpio_num, 
             (drv_mode == STP2_GPIO_DRV_OPEN_DRAIN) ? "Open-Drain" : "Push-Pull", 
             drv_reg);

    // 更新引脚状态记录
    if (mode == STP2_GPIO_MODE_OUTPUT) {
        stp2_pin_status[gpio_num].pin_state = state;
    }
    stp2_pin_status[gpio_num].pupd = pupd;
    
    ESP_LOGI(TAG, "GPIO%d status updated: function=%d, state=%d, pupd=%d", 
             gpio_num, stp2_pin_status[gpio_num].pin_mode, 
             stp2_pin_status[gpio_num].pin_state, stp2_pin_status[gpio_num].pupd);

    return ESP_OK;
}

// [STP2_ADDR_GPIO_DRV]
// 配置GPIO驱动模式（推挽/开漏）
// INPUT: stp2_gpio_num_t gpio_num (GPIO编号)
//        stp2_gpio_drv_t drv_mode (驱动模式：推挽或开漏)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t stp2_gpio_set_drv(stp2_gpio_num_t gpio_num, stp2_gpio_drv_t drv_mode)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_gpio_set_drv -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > STP2_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t drv_reg;
    esp_err_t ret;

    // 读取当前GPIO驱动模式寄存器
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_GPIO_DRV, &drv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO drive register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO drive register: 0x%02X", drv_reg);

    // 修改对应GPIO的驱动模式位
    if (drv_mode == STP2_GPIO_DRV_OPEN_DRAIN) {
        drv_reg |= (1 << gpio_num);   // 设置为开漏模式
    } else {
        drv_reg &= ~(1 << gpio_num);  // 设置为推挽模式
    }

    // 写入修改后的值
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_GPIO_DRV, drv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO drive register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set GPIO%d drive mode to %s: 0x%02X", 
             gpio_num, 
             (drv_mode == STP2_GPIO_DRV_OPEN_DRAIN) ? "Open-Drain" : "Push-Pull", 
             drv_reg);

    return ESP_OK;
}

// [STP2_ADDR_GPIO_DRV]
// 配置LED_EN驱动模式（推挽/开漏）
// INPUT: stp2_gpio_drv_t drv_mode (驱动模式：推挽或开漏)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t stp2_led_en_set_drv(stp2_gpio_drv_t drv_mode)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_led_en_set_drv -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t drv_reg;
    esp_err_t ret;

    // 读取当前GPIO驱动模式寄存器
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_GPIO_DRV, &drv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO drive register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO drive register: 0x%02X", drv_reg);

    // 修改第5位（LED_EN_DRV位）
    if (drv_mode == STP2_GPIO_DRV_OPEN_DRAIN) {
        drv_reg |= (1 << 5);   // 设置第5位为1（开漏模式）
    } else {
        drv_reg &= ~(1 << 5);  // 清除第5位为0（推挽模式）
    }

    // 写入修改后的值
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_GPIO_DRV, drv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO drive register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set LED_EN drive mode to %s: 0x%02X", 
             (drv_mode == STP2_GPIO_DRV_OPEN_DRAIN) ? "Open-Drain" : "Push-Pull", 
             drv_reg);

    return ESP_OK;
}

// [STP2_ADDR_GPIO_MODE]
// 单独配置GPIO模式（输入/输出）
// INPUT: stp2_gpio_num_t gpio_num (GPIO编号)
//        stp2_gpio_mode_t mode (模式：输入或输出)
// OUTPUT: ESP_OK on success, error code on failure
// TIPS: stp2_pin_status 会记录变更
esp_err_t stp2_gpio_set_mode(stp2_gpio_num_t gpio_num, stp2_gpio_mode_t mode)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_gpio_set_mode -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > STP2_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t read_mode;
    esp_err_t ret;

    // 读取当前GPIO模式
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_GPIO_MODE, &read_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO mode register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO%d mode: 0x%02X (bit %d: %s)", 
             gpio_num, read_mode, gpio_num, (read_mode & (1 << gpio_num)) ? "Output" : "Input");

    // 设置GPIO模式
    if (mode == STP2_GPIO_MODE_OUTPUT) {
        read_mode |= (1 << gpio_num);   // 设置为输出模式
    } else {
        read_mode &= ~(1 << gpio_num);  // 设置为输入模式
    }

    // 写入GPIO模式
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_GPIO_MODE, read_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO mode register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set GPIO%d mode to %s: 0x%02X", 
             gpio_num, (mode == STP2_GPIO_MODE_OUTPUT) ? "Output" : "Input", read_mode);

    ESP_LOGI(TAG, "GPIO%d mode updated successfully", gpio_num);
    return ESP_OK;
}

// [STP2_ADDR_GPIO_OUT]
// 单独配置GPIO输出状态（仅在输出模式下有效）
// INPUT: stp2_gpio_num_t gpio_num (GPIO编号)
//        stp2_gpio_state_t state (输出状态：高电平或低电平)
// OUTPUT: ESP_OK on success, error code on failure
// TIPS: stp2_pin_status 会记录变更
esp_err_t stp2_gpio_set_state(stp2_gpio_num_t gpio_num, stp2_gpio_state_t state)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_gpio_set_state -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > STP2_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t read_mode, read_out;
    esp_err_t ret;

    // 检查GPIO是否配置为输出模式
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_GPIO_MODE, &read_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO mode register: %s", esp_err_to_name(ret));
        return ret;
    }

    if (!(read_mode & (1 << gpio_num))) {
        ESP_LOGW(TAG, "GPIO%d is not in output mode, state setting may not take effect", gpio_num);
    }

    // 读取当前输出状态
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_GPIO_OUT, &read_out);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO output register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO%d output: 0x%02X (bit %d: %s)", 
             gpio_num, read_out, gpio_num, (read_out & (1 << gpio_num)) ? "High" : "Low");

    // 设置输出状态
    if (state == STP2_GPIO_OUTPUT_HIGH) {
        read_out |= (1 << gpio_num);  // 设置高电平
    } else {
        read_out &= ~(1 << gpio_num); // 设置低电平
    }
    
    // 写入输出状态
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_GPIO_OUT, read_out);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO output register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set GPIO%d output to %s: 0x%02X", 
             gpio_num, (state == STP2_GPIO_OUTPUT_HIGH) ? "High" : "Low", read_out);

    // 更新引脚状态记录
    stp2_pin_status[gpio_num].pin_state = state;
    
    ESP_LOGI(TAG, "GPIO%d state updated successfully", gpio_num);
    return ESP_OK;
}

// [STP2_ADDR_GPIO_PUPD0/1]
// 单独配置GPIO上下拉
// INPUT: stp2_gpio_num_t gpio_num (GPIO编号)
//        stp2_gpio_pupd_t pupd (上下拉配置：无配置/上拉/下拉)
// OUTPUT: ESP_OK on success, error code on failure
// TIPS: stp2_pin_status 会记录变更
esp_err_t stp2_gpio_set_pupd(stp2_gpio_num_t gpio_num, stp2_gpio_pupd_t pupd)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_gpio_set_pupd -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > STP2_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t pupd_reg_addr, read_pupd;
    uint8_t shift_bits;
    esp_err_t ret;
    
    // 确定寄存器和位偏移
    if (gpio_num <= STP2_GPIO_NUM_3) {
        pupd_reg_addr = STP2_ADDR_GPIO_PUPD0;
        shift_bits = gpio_num * 2;
    } else {
        pupd_reg_addr = STP2_ADDR_GPIO_PUPD1;
        shift_bits = (gpio_num - STP2_GPIO_NUM_4) * 2;
    }
    
    // 读取当前上下拉配置
    ret = i2c_bus_read_byte(stp2_i2c_device, pupd_reg_addr, &read_pupd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO pull-up/pull-down register (0x%02X): %s", pupd_reg_addr, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO%d pull config: 0x%02X (bits %d-%d: %d)", 
             gpio_num, read_pupd, shift_bits+1, shift_bits, (read_pupd >> shift_bits) & 0x03);
    
    // 修改对应的位
    read_pupd &= ~(0x03 << shift_bits);  // 清除对应的两位
    read_pupd |= (pupd << shift_bits);   // 设置新的上下拉配置
    
    // 写入修改后的值
    ret = i2c_bus_write_byte(stp2_i2c_device, pupd_reg_addr, read_pupd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO pull-up/pull-down register (0x%02X): %s", pupd_reg_addr, esp_err_to_name(ret));
        return ret;
    }
    
    const char *pupd_str = (pupd == STP2_GPIO_PUPD_PULLUP) ? "Pull-up" : 
                           (pupd == STP2_GPIO_PUPD_PULLDOWN) ? "Pull-down" : "No-pull";
    ESP_LOGI(TAG, "Set GPIO%d pull config to %s: 0x%02X", gpio_num, pupd_str, read_pupd);

    // 更新引脚状态记录
    stp2_pin_status[gpio_num].pupd = pupd;
    
    ESP_LOGI(TAG, "GPIO%d pull configuration updated successfully", gpio_num);
    return ESP_OK;
}

// [STP2_ADDR_GPIO_WAKE_EN]
// 配置GPIO唤醒使能
// INPUIT: stp2_gpio_num_t gpio_num
//        stp2_gpio_wake_t wake_en
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_gpio_set_wake_en(stp2_gpio_num_t gpio_num, stp2_gpio_wake_t wake_en)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_gpio_set_wake_en -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > STP2_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    // 检查是否可设置唤醒: 根据CSV配置限制和stp2_pin_status状态
    if (wake_en == STP2_GPIO_WAKE_ENABLE) {
        // GPIO1不支持唤醒
        if (gpio_num == STP2_GPIO_NUM_1) {
            ESP_LOGE(TAG, "GPIO1 does not support WAKE functionality");
            return ESP_ERR_INVALID_ARG;
        }
        
        // 冲突检查：0<->2, 3<->4
        if ((gpio_num == STP2_GPIO_NUM_0 && stp2_pin_status[STP2_GPIO_NUM_2].wake_en == STP2_GPIO_WAKE_ENABLE) ||
            (gpio_num == STP2_GPIO_NUM_2 && stp2_pin_status[STP2_GPIO_NUM_0].wake_en == STP2_GPIO_WAKE_ENABLE)) {
            ESP_LOGE(TAG, "GPIO%d wake conflicts with GPIO% d", gpio_num, (gpio_num == STP2_GPIO_NUM_0) ? 2 : 0);
            return ESP_ERR_INVALID_ARG;
        }
        if ((gpio_num == STP2_GPIO_NUM_3 && stp2_pin_status[STP2_GPIO_NUM_4].wake_en == STP2_GPIO_WAKE_ENABLE) ||
            (gpio_num == STP2_GPIO_NUM_4 && stp2_pin_status[STP2_GPIO_NUM_3].wake_en == STP2_GPIO_WAKE_ENABLE)) {
            ESP_LOGE(TAG, "GPIO%d wake conflicts with GPIO% d", gpio_num, (gpio_num == STP2_GPIO_NUM_3) ? 4 : 3);
            return ESP_ERR_INVALID_ARG;
        }
    }
    uint8_t reg_val;
    esp_err_t ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_GPIO_WAKE_EN, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO wake enable register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置对应位
    if (wake_en == STP2_GPIO_WAKE_ENABLE) {
        reg_val |= (1 << gpio_num);
    } else {
        reg_val &= ~(1 << gpio_num);
    }

    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_GPIO_WAKE_EN, reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO wake enable register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set GPIO%d wake enable to %s: 0x%02X", gpio_num,
             (wake_en == STP2_GPIO_WAKE_ENABLE) ? "Enable" : "Disable", reg_val);
    // 更新唤醒使能状态
    stp2_pin_status[gpio_num].wake_en = wake_en;

    return ESP_OK;
}

// [STP2_ADDR_GPIO_WAKE_CFG]
// 配置GPIO唤醒边沿配置
// INPUIT: stp2_gpio_num_t gpio_num
//        stp2_gpio_wake_edge_t wake_cfg
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_gpio_set_wake_cfg(stp2_gpio_num_t gpio_num, stp2_gpio_wake_edge_t wake_cfg)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_gpio_set_wake_cfg -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > STP2_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    //不支持GPIO1的唤醒边沿配置
    if (gpio_num == STP2_GPIO_NUM_1) {
        ESP_LOGE(TAG, "GPIO1 does not support wake edge configuration");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_val;
    esp_err_t ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_GPIO_WAKE_CFG, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO wake config register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置对应位
    if (wake_cfg == STP2_GPIO_WAKE_RISING) {
        reg_val |= (1 << gpio_num);
    } else {
        reg_val &= ~(1 << gpio_num);
    }

    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_GPIO_WAKE_CFG, reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO wake config register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set GPIO%d wake edge to %s: 0x%02X", gpio_num,
             (wake_cfg == STP2_GPIO_WAKE_RISING) ? "Rising" : "Falling", reg_val);
    // 更新唤醒边沿配置状态
    stp2_pin_status[gpio_num].wake_cfg = wake_cfg;

    return ESP_OK;
}

// [STP2_ADDR_GPIO_IN]
// 读取GPIO输入状态
// INPUIT: stp2_gpio_num_t gpio_num
//          stp2_gpio_in_state_t *state (用来存储读取的状态)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_gpio_get_in_state(stp2_gpio_num_t gpio_num, stp2_gpio_in_state_t *state)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_gpio_get_in_state -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > STP2_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (state == NULL) {
        ESP_LOGE(TAG, "State pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t read_buf;
    esp_err_t ret;

    // 读取GPIO输入状态
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_GPIO_IN, &read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO input register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 提取指定GPIO的输入状态
    *state = (read_buf & (1 << gpio_num)) ? STP2_GPIO_IN_STATE_HIGH : STP2_GPIO_IN_STATE_LOW;
    
    ESP_LOGI(TAG, "GPIO%d input state: %s (0x%02X, bit %d: %d)", 
             gpio_num, (*state == STP2_GPIO_IN_STATE_HIGH) ? "High" : "Low", 
             read_buf, gpio_num, (*state == STP2_GPIO_IN_STATE_HIGH) ? 1 : 0);
    
    return ESP_OK;
}

// [STP2_ADDR_ADC_RES] Read Only
// 读取ADC值
// INPUIT: stp2_adc_channel_t channel
//          uint16_t *adc_12bit_value (用来存储读取的ADC值)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_adc_read(stp2_adc_channel_t channel, uint16_t *adc_value)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_adc_read -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (adc_value == NULL) {
        ESP_LOGE(TAG, "ADC value pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    // 若channel为1或2，检查对应GPIO模式
    if (channel == STP2_ADC_CHANNEL_1 || channel == STP2_ADC_CHANNEL_2) {
        stp2_gpio_num_t gpio = (channel == STP2_ADC_CHANNEL_1) ? STP2_GPIO_NUM_1 : STP2_GPIO_NUM_2;
        if (stp2_pin_status[gpio].pin_mode != STP2_GPIO_FUNC_OTHER) {
            ESP_LOGE(TAG, "GPIO%d not configured for ADC function", gpio);
            return ESP_ERR_INVALID_STATE;
        }
    }
    // 如果为0，3，4，5 或 大于6的通道，报错不支持
    if (channel == STP2_ADC_CHANNEL_0 || channel == STP2_ADC_CHANNEL_3 ||
        channel == STP2_ADC_CHANNEL_4 || channel == STP2_ADC_CHANNEL_5 ||
        channel > STP2_ADC_CHANNEL_TEMP) {
        ESP_LOGE(TAG, "ADC channel %d is not supported", channel);
        return ESP_ERR_NOT_SUPPORTED;
    }

    uint8_t read_buf;
    esp_err_t ret;

    // 读取当前ADC控制寄存器
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_ADC_CTRL, &read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC control register: %s", esp_err_to_name(ret));
        return ret;
    }
    // 设置ADC通道并启动转换
    read_buf = ((channel & 0x07) << 1) | 0x01;
    
    // 写入ADC控制寄存器以启动转换
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_ADC_CTRL, read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write ADC control register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Started ADC conversion on channel %d: 0x%02X", channel, read_buf);

    // 等待ADC转换完成（START位清零）
    int retry = 10;  // 最多等待10次
    while (retry--) {
        vTaskDelay(pdMS_TO_TICKS(20));  // 等待20ms
        ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_ADC_CTRL, &read_buf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ADC control register during wait: %s", esp_err_to_name(ret));
            return ret;
        }
        
        if ((read_buf & 0x01) == 0) {  // START位被清零，表示转换完成
            break;
        }
        
        if (retry == 0) {
            ESP_LOGE(TAG, "ADC conversion timeout");
            return ESP_ERR_TIMEOUT;
        }
    }

    // 读取ADC结果寄存器
    uint8_t adc_l, adc_h;
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_ADC_RES_L, &adc_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC result low byte: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_ADC_RES_H, &adc_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC result high byte: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 组合ADC结果（12位）
    *adc_value = ((adc_h & 0x0F) << 8) | adc_l;
    
    ESP_LOGI(TAG, "ADC channel %d result: %u (0x%03X)", channel, *adc_value, *adc_value);
    
    return ESP_OK;
}

// [STP2_ADDR_PWM]
// 配置PWM参数
// INPUIT: stp2_pwm_channel_t channel
//          stp2_pwm_ctrl_t ctrl (PWM使能控制)
//          stp2_pwm_polarity_t polarity (PWM极性)
//          uint32_t pwm_freq_16bit_value (16位频率值, 0-65535)
//          uint32_t duty_cycle_12bit_value (12位占空比值, 0-4095)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_pwm_set(stp2_pwm_channel_t channel, stp2_pwm_ctrl_t ctrl, stp2_pwm_polarity_t polarity, uint32_t pwm_freq_16bit_value, uint32_t duty_cycle_12bit_value)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_pwm_set -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    // 检查PWM对应的GPIO引脚模式：PWM0->GPIO3, PWM1->GPIO4
    if (channel == STP2_PWM_CHANNEL_0) {
        if (stp2_pin_status[STP2_GPIO_NUM_3].pin_mode != STP2_GPIO_FUNC_OTHER) {
            ESP_LOGW(TAG, "GPIO3 not configured for PWM function (OTHER mode)");
        }
    } else {
        if (stp2_pin_status[STP2_GPIO_NUM_4].pin_mode != STP2_GPIO_FUNC_OTHER) {
            ESP_LOGW(TAG, "GPIO4 not configured for PWM function (OTHER mode)");
        }
    }

    if (duty_cycle_12bit_value > 4095) {
        ESP_LOGE(TAG, "Invalid duty cycle value: %lu (max 4095)", duty_cycle_12bit_value);
        return ESP_ERR_INVALID_ARG;
    }

    if (pwm_freq_16bit_value > 0xFFFF) {
        ESP_LOGE(TAG, "Invalid PWM frequency value: %lu (max 65535)", pwm_freq_16bit_value);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t pwm_l_addr, pwm_hc_addr;
    uint8_t read_hc;
    esp_err_t ret;

    // 确定寄存器地址
    if (channel == STP2_PWM_CHANNEL_0) {
        pwm_l_addr = STP2_ADDR_PWM0_L;
        pwm_hc_addr = STP2_ADDR_PWM0_HC;
    } else {
        pwm_l_addr = STP2_ADDR_PWM1_L;
        pwm_hc_addr = STP2_ADDR_PWM1_HC;
    }

    // 读取当前PWM控制/高位寄存器
    ret = i2c_bus_read_byte(stp2_i2c_device, pwm_hc_addr, &read_hc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PWM%d control register: %s", channel, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current PWM%d control: 0x%02X (enable: %d, polarity: %d, duty high bits: 0x%X)", 
             channel, read_hc, (read_hc >> 4) & 0x01, (read_hc >> 5) & 0x01, read_hc & 0x0F);

    // 设置PWM占空比低8位
    uint8_t pwm_l = duty_cycle_12bit_value & 0xFF;
    ret = i2c_bus_write_byte(stp2_i2c_device, pwm_l_addr, pwm_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PWM%d duty cycle low byte: %s", channel, esp_err_to_name(ret));
        return ret;
    }
    
    // 设置PWM控制/高位寄存器
    uint8_t pwm_hc = (polarity << 5) | (ctrl << 4) | ((duty_cycle_12bit_value >> 8) & 0x0F);
    ret = i2c_bus_write_byte(stp2_i2c_device, pwm_hc_addr, pwm_hc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PWM%d control register: %s", channel, esp_err_to_name(ret));
        return ret;
    }
    
    // 检查频率改变对另一路PWM的影响
    if (stp2_last_pwm_freq != 0 && pwm_freq_16bit_value != stp2_last_pwm_freq) {
        ESP_LOGW(TAG, "PWM frequency changed from %lu to %lu, other channel will also be updated", stp2_last_pwm_freq, pwm_freq_16bit_value);
    }
    // 更新全局记录
    stp2_last_pwm_freq = pwm_freq_16bit_value;

    // 设置PWM频率寄存器
    uint8_t freq_l = pwm_freq_16bit_value & 0xFF;
    uint8_t freq_h = (pwm_freq_16bit_value >> 8) & 0xFF;
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_PWM_FREQ_L, freq_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PWM frequency low byte: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_PWM_FREQ_H, freq_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PWM frequency high byte: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set PWM%d: freq=%lu/65535, duty=%lu/4095, %s, polarity=%s", 
             channel, pwm_freq_16bit_value, duty_cycle_12bit_value, 
             (ctrl == STP2_PWM_CTRL_ENABLE) ? "enabled" : "disabled", 
             (polarity == STP2_PWM_POLARITY_NORMAL) ? "normal" : "inverted");
    
    return ESP_OK;
}

// [STP2_ADDR_WDT_SET]
// 配置WDT参数
// INPUIT: stp2_wdt_ctrl_t ctrl (WDT使能控制)
//          uint8_t timeout_s (WDT超时时间, 单位秒 0-255)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_wdt_set(stp2_wdt_ctrl_t ctrl, uint8_t timeout_s)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_wdt_set -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t read_buf;
    esp_err_t ret;

    // 读取当前WDT设置
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_WDT_CNT, &read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WDT counter register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current WDT counter: 0x%02X (%d seconds, %s)", 
             read_buf, read_buf, read_buf == 0 ? "disabled" : "enabled");

    // 设置WDT超时时间（0表示禁用，1-255表示超时秒数）
    uint8_t wdt_cnt = (ctrl == STP2_WDT_CTRL_ENABLE) ? timeout_s : 0;
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_WDT_CNT, wdt_cnt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write WDT counter register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set WDT to %s with timeout %u seconds", 
             (ctrl == STP2_WDT_CTRL_ENABLE) ? "enabled" : "disabled", 
             (ctrl == STP2_WDT_CTRL_ENABLE) ? timeout_s : 0);
    
    return ESP_OK;
}

// [STP2_ADDR_WDT_CLR] Write Only
// 喂狗
// INPUIT: key (喂狗密钥, 0xA5)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_wdt_feed(uint8_t key)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_wdt_feed -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (key != 0xA5) {
        ESP_LOGE(TAG, "Invalid WDT feed key: 0x%02X (should be 0xA5)", key);
        return ESP_ERR_INVALID_ARG;
    }

    // 写入喂狗密钥
    esp_err_t ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_WDT_KEY, key);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write WDT key register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WDT feed successful with key 0x%02X", key);
    
    return ESP_OK;
}

// [STP2_ADDR_WDT_CNT] Read Only
// 读取看门狗倒计时值
// INPUT: uint8_t *wdt_cnt (用来存储读取的看门狗倒计时值, 单位秒)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t stp2_wdt_get_wdt_cnt(uint8_t *wdt_cnt)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_wdt_get_wdt_cnt -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (wdt_cnt == NULL) {
        ESP_LOGE(TAG, "WDT count pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_WDT_CNT, wdt_cnt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WDT count register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WDT count: %u seconds (%s)", *wdt_cnt, 
             (*wdt_cnt == 0) ? "disabled" : "enabled");
    
    return ESP_OK;
}

// [STP2_ADDR_TIM]
// 配置定时器参数
// INPUIT: stp2_tim_ctrl_t ctrl (定时器使能控制)
//          stp2_tim_action_t action (定时器动作配置)
//          uint32_t tim_ct_31bit_value (31位定时器计数值, 0-0x7FFFFFFF,单位s)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_tim_set(stp2_tim_ctrl_t ctrl, stp2_tim_action_t action, uint32_t tim_ct_31bit_value)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_tim_set -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (tim_ct_31bit_value > 0x7FFFFFFF) {
        ESP_LOGE(TAG, "Invalid timer count: %lu (max 0x7FFFFFFF)", tim_ct_31bit_value);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t read_cfg;
    esp_err_t ret;

    // 读取当前定时器配置
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_TIM_CFG, &read_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read timer configuration register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current timer configuration: 0x%02X (ARM: %d, ACTION: %d)", 
             read_cfg, (read_cfg >> 3) & 0x01, read_cfg & 0x07);

    // 设置计数值 - 使用4个字节寄存器
    uint8_t tim_byte0 = tim_ct_31bit_value & 0xFF;
    uint8_t tim_byte1 = (tim_ct_31bit_value >> 8) & 0xFF;
    uint8_t tim_byte2 = (tim_ct_31bit_value >> 16) & 0xFF;
    uint8_t tim_byte3 = (tim_ct_31bit_value >> 24) & 0x7F; // 只有低7位有效

    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_TIM_CNT_BYTE_0, tim_byte0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timer counter byte 0: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_TIM_CNT_BYTE_1, tim_byte1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timer counter byte 1: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_TIM_CNT_BYTE_2, tim_byte2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timer counter byte 2: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_TIM_CNT_BYTE_3, tim_byte3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timer counter byte 3: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 设置定时器配置
    // ARM位[3]: 当ctrl为ENABLE时设置为1，表示启用定时器；为DISABLE时设置为0
    // ACTION位[2:0]: 定时器动作配置
    uint8_t tim_cfg;
    if (ctrl == STP2_ADDR_TIM_ENABLE) {
        tim_cfg = (1 << 3) | action;  // ARM=1(启用定时器), 设置ACTION
    } else {
        tim_cfg = 0;  // ARM=0(停止计数器), ACTION=000(停止计数器)
    }
    
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_TIM_CFG, tim_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timer configuration register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set timer: count=%lu, %s, ACTION=%s", 
             tim_ct_31bit_value, 
             (ctrl == STP2_ADDR_TIM_ENABLE) ? "enabled" : "disabled", 
             (action == STP2_TIM_ACTION_000) ? "stop" :
             (action == STP2_TIM_ACTION_001) ? "set WAKE flag" :
             (action == STP2_TIM_ACTION_010) ? "restart" :
             (action == STP2_TIM_ACTION_011) ? "power on" :
             (action == STP2_TIM_ACTION_100) ? "power off" : "unknown");

    return ESP_OK;
}

// [STP2_ADDR_TIM_CLR] Write Only
// 清除定时器并重载
// INPUIT: key (清除密钥, 0xA5)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_tim_clear(uint8_t key)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_tim_clear -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (key != 0xA5) {
        ESP_LOGE(TAG, "Invalid timer clear key: 0x%02X (should be 0xA5)", key);
        return ESP_ERR_INVALID_ARG;
    }

    // 写入清除密钥
    esp_err_t ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_TIM_KEY, key);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timer key register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Timer cleared and reloaded with key 0x%02X", key);
    
    return ESP_OK;
}

// [STP2_ADDR_BTN_DL_LOCK]
// 配置下载锁定模式(控制BTN_CFG寄存器的DL_LOCK位[7])
// INPUIT: stp2_download_enable_t enable (下载模式锁定控制)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_download_enable(stp2_download_enable_t enable)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_download_enable -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t btn_cfg;
    esp_err_t ret;

    // 读取当前按键配置寄存器
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_BTN_CFG, &btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button configuration register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current button configuration: 0x%02X (DL_LOCK: %d)", 
             btn_cfg, (btn_cfg >> 7) & 0x01);

    // 设置或清除DL_LOCK位[7]
    if (enable == STP2_ADDR_DOWNLOAD_DISABLE) {
        btn_cfg |= (1 << 7);  // 设置DL_LOCK位
    } else {
        btn_cfg &= ~(1 << 7); // 清除DL_LOCK位
    }
    
    // 写入更新后的配置
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_BTN_CFG, btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s download lock: %s", 
                 (enable == STP2_ADDR_DOWNLOAD_ENABLE) ? "enable" : "disable", 
                 esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "STP2 download lock %s successfully", 
             (enable == STP2_ADDR_DOWNLOAD_ENABLE) ? "enabled" : "disabled");
    
    return ESP_OK;
}

// [STP2_ADDR_BTN_CFG]
// 按键响应延迟配置
// INPUIT: stp2_btn_type_t btn_type (按键类型)
//          stp2_btn_delay_t delay (延迟配置)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_btn_set_cfg(stp2_btn_type_t btn_type, stp2_btn_delay_t delay)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_btn_set_cfg -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 验证按键类型参数
    if (btn_type > STP2_ADDR_BTN_TYPE_LONG_PRESS) {
        ESP_LOGE(TAG, "Invalid button type: %d", btn_type);
        return ESP_ERR_INVALID_ARG;
    }

    // 验证延迟配置参数（对应不同按键类型的有效范围）
    if ((btn_type == STP2_ADDR_BTN_TYPE_CLICK && delay > STP2_ADDR_BTN_CLICK_DELAY_1000MS) ||
        (btn_type == STP2_ADDR_BTN_TYPE_DOUBLE_CLICK && delay > STP2_ADDR_BTN_DOUBLE_CLICK_DELAY_1000MS) ||
        (btn_type == STP2_ADDR_BTN_TYPE_LONG_PRESS && delay > STP2_ADDR_BTN_LONG_PRESS_DELAY_4000MS)) {
        ESP_LOGE(TAG, "Invalid delay value %d for button type %d", delay, btn_type);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t btn_cfg;
    esp_err_t ret;

    // 读取当前按键配置
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_BTN_CFG, &btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button configuration register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current button configuration: 0x%02X", btn_cfg);

    // 根据按键类型修改相应的位
    switch (btn_type) {
        case STP2_ADDR_BTN_TYPE_CLICK:
            // 修改[2-1]位 - SINGLE配置
            btn_cfg &= ~(0x03 << 1);  // 清除原有配置
            btn_cfg |= (delay & 0x03) << 1;  // 设置新配置
            break;
        
        case STP2_ADDR_BTN_TYPE_DOUBLE_CLICK:
            // 修改[6-5]位 - DBL配置
            btn_cfg &= ~(0x03 << 5);  // 清除原有配置
            btn_cfg |= (delay & 0x03) << 5;  // 设置新配置
            break;
        
        case STP2_ADDR_BTN_TYPE_LONG_PRESS:
            // 修改[4-3]位 - LONG配置
            btn_cfg &= ~(0x03 << 3);  // 清除原有配置
            btn_cfg |= (delay & 0x03) << 3;  // 设置新配置
            break;
    }

    // 写入修改后的配置
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_BTN_CFG, btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write button configuration register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 记录按键类型和延迟的字符串描述，用于日志
    const char *type_str;
    const char *delay_str;

    // 按键类型描述
    switch (btn_type) {
        case STP2_ADDR_BTN_TYPE_CLICK:
            type_str = "Click";
            break;
        case STP2_ADDR_BTN_TYPE_DOUBLE_CLICK:
            type_str = "Double Click";
            break;
        case STP2_ADDR_BTN_TYPE_LONG_PRESS:
            type_str = "Long Press";
            break;
        default:
            type_str = "Unknown";
            break;
    }

    // 延迟配置描述
    if (btn_type == STP2_ADDR_BTN_TYPE_LONG_PRESS) {
        switch (delay) {
            case STP2_ADDR_BTN_LONG_PRESS_DELAY_1000MS:
                delay_str = "1000ms";
                break;
            case STP2_ADDR_BTN_LONG_PRESS_DELAY_2000MS:
                delay_str = "2000ms";
                break;
            case STP2_ADDR_BTN_LONG_PRESS_DELAY_3000MS:
                delay_str = "3000ms";
                break;
            case STP2_ADDR_BTN_LONG_PRESS_DELAY_4000MS:
                delay_str = "4000ms";
                break;
            default:
                delay_str = "Unknown";
                break;
        }
    } else {  // 点击和双击的延迟配置相同
        switch (delay) {
            case STP2_ADDR_BTN_CLICK_DELAY_125MS:
                delay_str = "125ms";
                break;
            case STP2_ADDR_BTN_CLICK_DELAY_250MS:
                delay_str = "250ms";
                break;
            case STP2_ADDR_BTN_CLICK_DELAY_500MS:
                delay_str = "500ms";
                break;
            case STP2_ADDR_BTN_CLICK_DELAY_1000MS:
                delay_str = "1000ms";
                break;
            default:
                delay_str = "Unknown";
                break;
        }
    }

    ESP_LOGI(TAG, "Button %s delay set to %s successfully (reg value: 0x%02X)", 
             type_str, delay_str, btn_cfg);
    
    return ESP_OK;
}

// [STP2_ADDR_IRQ_STATUS1]
// 读取GPIO中断状态
// INPUIT: stp2_irq_gpio_t *gpio_num (GPIO编号)
//         stp2_irq_gpio_clean_type_t clean_type (中断清除类型)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_irq_get_status(stp2_irq_gpio_t *gpio_num, stp2_irq_gpio_clean_type_t clean_type)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_irq_get_status -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num == NULL) {
        ESP_LOGE(TAG, "GPIO number pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t irq_status;
    esp_err_t ret;

    // 读取GPIO中断状态寄存器
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_IRQ_STATUS1, &irq_status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read IRQ status register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 检查哪个GPIO的中断状态位被设置
    *gpio_num = STP2_ADDR_IRQ_NULL;  // 初始化为无效值
    for (int i = 0; i < 5; i++) {
        if (irq_status & (1 << i)) {
            *gpio_num = (stp2_irq_gpio_t)i;
            break;
        }
    }

    ESP_LOGI(TAG, "IRQ status: 0x%02X, triggered GPIO: %d", 
             irq_status, (*gpio_num == STP2_ADDR_IRQ_NULL) ? -1 : (int)*gpio_num);
    // Clear interrupts based on clean_type
    switch (clean_type) {
        case STP2_ADDR_IRQ_GPIO_NOT_CLEAN:
            break;
        case STP2_ADDR_IRQ_GPIO_ONCE_CLEAN:
            if (*gpio_num != STP2_ADDR_IRQ_NULL) {
                uint8_t new_val = irq_status & ~(1 << *gpio_num);
                ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_IRQ_STATUS1, new_val);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to clear IRQ once: %s", esp_err_to_name(ret));
                    return ret;
                }
            }
            break;
        case STP2_ADDR_IRQ_GPIO_ALL_CLEAN:
            ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_IRQ_STATUS1, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to clear all IRQs: %s", esp_err_to_name(ret));
                return ret;
            }
            break;
        default:
            break;
    }

    return ESP_OK;
}

// [STP2_ADDR_IRQ_STATUS2]
// 读取系统中断状态
// INPUIT: stp2_irq_sys_t *sys_irq (系统中断类型)
//         stp2_irq_sys_clean_type_t clean_type (中断清除类型)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_irq_get_sys_status(stp2_irq_sys_t *sys_irq, stp2_irq_sys_clean_type_t clean_type)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_irq_get_sys_status -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sys_irq == NULL) {
        ESP_LOGE(TAG, "System IRQ pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t irq_status;
    esp_err_t ret;

    // 读取系统中断状态寄存器
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_IRQ_STATUS2, &irq_status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read system IRQ status register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 检查哪个系统中断状态位被设置
    *sys_irq = STP2_ADDR_IRQ_SYS_NULL;  // 初始化为无效值
    for (int i = 0; i <= 5; i++) {
        if (irq_status & (1 << i)) {
            *sys_irq = (stp2_irq_sys_t)i;
            break;
        }
    }

    ESP_LOGI(TAG, "System IRQ status: 0x%02X, event: %d", 
             irq_status, (*sys_irq == STP2_ADDR_IRQ_SYS_NULL) ? -1 : (int)*sys_irq);
    
    // Clear system IRQ based on clean_type
    switch (clean_type) {
        case STP2_ADDR_IRQ_SYS_NOT_CLEAN:
            // 不清除标志
            break;
        case STP2_ADDR_IRQ_SYS_ONCE_CLEAN:
            // 清除当前系统中断标志
            if (*sys_irq != STP2_ADDR_IRQ_SYS_NULL && *sys_irq <= STP2_ADDR_IRQ_SYS_BAT_REMOVE) {
                uint8_t new_val = irq_status & ~(1 << *sys_irq);
                ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_IRQ_STATUS2, new_val);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to clear system IRQ once: %s", esp_err_to_name(ret));
                    return ret;
                }
                ESP_LOGI(TAG, "Cleared system IRQ flag for event %d", *sys_irq);
            }
            break;
        case STP2_ADDR_IRQ_SYS_ALL_CLEAN:
            // 清除所有系统中断标志
            ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_IRQ_STATUS2, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to clear all system IRQs: %s", esp_err_to_name(ret));
                return ret;
            }
            ESP_LOGI(TAG, "Cleared all system IRQ flags");
            break;
        default:
            ESP_LOGW(TAG, "Unknown system IRQ clean type: %d", clean_type);
            break;
    }
    
    return ESP_OK;
}

// [STP2_ADDR_BATT_LVP]
// 配置电池低压保护阈值(2000-5000, 单位mV)
// INPUIT: uint16_t lvp_mv (低压保护阈值, 单位mV)
// OUTPUIT: ESP_OK on success, error code on failure
// 寄存器配置公式 lvp_mv = (2000mV + n * 7.81mV) ，n为写入寄存器的值
esp_err_t stp2_batt_set_lvp(uint16_t lvp_mv)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_batt_set_lvp -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 检查电压范围
    if (lvp_mv < 2000 || lvp_mv > 4000) {
        ESP_LOGE(TAG, "Invalid battery LVP value: %u mV (valid range: 2000-4000 mV)", lvp_mv);
        return ESP_ERR_INVALID_ARG;
    }

    // 计算寄存器值: n = (lvp_mv - 2000) / 7.81
    // 为了避免浮点运算，使用整数近似: n = (lvp_mv - 2000) * 100 / 781
    uint8_t reg_value = (uint8_t)((lvp_mv - 2000) * 100 / 781);
    
    // 写入LVP寄存器
    esp_err_t ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_BATT_LVP, reg_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set battery LVP: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Battery LVP set to %u mV (reg value: 0x%02X)", lvp_mv, reg_value);
    
    return ESP_OK;
}

// [STP2_ADDR_VREF] Read Only
// 读取参考电压值
// INPUIT: uint16_t *vref_mv (用来存储读取的参考电压值, 单位mV)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_vref_read(uint16_t *vref_mv)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_vref_read -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (vref_mv == NULL) {
        ESP_LOGE(TAG, "VREF pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t vref_l, vref_h;
    esp_err_t ret;

    // 读取VREF低8位
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_VREF_L, &vref_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VREF low byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取VREF高8位
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_VREF_H, &vref_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VREF high byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 计算电压值（单位mV）
    *vref_mv = ((uint16_t)vref_h << 8) | vref_l;
    
    ESP_LOGI(TAG, "VREF: %u mV (raw: 0x%04X)", *vref_mv, *vref_mv);
    
    return ESP_OK;
}

// [STP2_ADDR_VBAT] Read Only
// 读取电池电压值
// INPUIT: uint16_t *vbat_mv (用来存储读取的电池电压值, 单位mV)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_vbat_read(uint16_t *vbat_mv)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_vbat_read -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (vbat_mv == NULL) {
        ESP_LOGE(TAG, "VBAT pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t vbat_l, vbat_h;
    esp_err_t ret;

    // 读取电池电压低8位
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_VBAT_L, &vbat_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VBAT low byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取电池电压高4位
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_VBAT_H, &vbat_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VBAT high byte: %s", esp_err_to_name(ret));
        return ret;
    }
    // ESP_LOGI(TAG, "Battery voltage: %02x %02x", vbat_h, vbat_l);
    // 计算电池电压值（单位mV）
    *vbat_mv = ((uint16_t)vbat_h << 8) | vbat_l;
    
    ESP_LOGI(TAG, "Battery voltage: %u mV (raw: 0x%03X)", *vbat_mv, *vbat_mv);
    
    return ESP_OK;
}

// [STP2_ADDR_VIN] Read Only
// 读取VIN电压值
// INPUIT: uint16_t *vin_mv (用来存储读取的VIN电压值, 单位mV)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_vin_read(uint16_t *vin_mv)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_vin_read -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (vin_mv == NULL) {
        ESP_LOGE(TAG, "VIN pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t vin_l, vin_h;
    esp_err_t ret;

    // 读取VIN电压低8位
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_VIN_L, &vin_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VIN low byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取VIN电压高4位
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_VIN_H, &vin_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VIN high byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 计算VIN电压值（单位mV）
    *vin_mv = ((uint16_t)vin_h << 8) | vin_l;
    
    ESP_LOGI(TAG, "VIN voltage: %u mV (raw: 0x%03X)", *vin_mv, *vin_mv);
    
    return ESP_OK;
}

// [STP2_ADDR_VBUS] Read Only
// 读取VBUS电压值
// INPUIT: uint16_t *vbus_mv (用来存储读取的VBUS电压值, 单位mV)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_vbus_read(uint16_t *vbus_mv)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_vbus_read -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (vbus_mv == NULL) {
        ESP_LOGE(TAG, "VBUS pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t vbus_l, vbus_h;
    esp_err_t ret;

    // 读取VBUS电压低8位
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_VBUS_L, &vbus_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VBUS low byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取VBUS电压高4位
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_VBUS_H, &vbus_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VBUS high byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 计算VBUS电压值（单位mV）
    *vbus_mv = ((uint16_t)vbus_h << 8) | vbus_l;
    
    ESP_LOGI(TAG, "VBUS voltage: %u mV (raw: 0x%03X)", *vbus_mv, *vbus_mv);
    
    return ESP_OK;
}

// [STP2_ADDR_PWR_SRC] Read Only
// 读取电源来源状态
// INPUIT: stp2_pwr_src_t *pwr_src (用来存储读取的电源来源状态)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_pwr_src_read(stp2_pwr_src_t *pwr_src)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_pwr_src_read -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (pwr_src == NULL) {
        ESP_LOGE(TAG, "Power source pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t pwr_src_reg;
    esp_err_t ret;

    // 读取电源来源寄存器
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_PWR_SRC, &pwr_src_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read power source register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 解析电源来源 (低3位)
    uint8_t src_bits = pwr_src_reg & 0x07;
    
    if (src_bits & (1 << 0)) {
        *pwr_src = STP2_PWR_SRC_5VIN;
    } else if (src_bits & (1 << 1)) {
        *pwr_src = STP2_PWR_SRC_5VINOUT;
    } else if (src_bits & (1 << 2)) {
        *pwr_src = STP2_PWR_SRC_BAT;
    } else {
        *pwr_src = STP2_PWR_SRC_UNKNOWN;
    }
    
    ESP_LOGI(TAG, "Power source: %d (reg value: 0x%02X, bits: %d)", 
             *pwr_src, pwr_src_reg, src_bits);
    
    return ESP_OK;
}

// [STP2_ADDR_WAKE_SRC] Read Only
// [STP2_ADDR_WAKE_SRC] Read Only
// 读取唤醒来源状态
// INPUIT: stp2_wake_src_t *wake_src (用来存储读取的唤醒来源状态)
//         stp2_wake_flag_clean_type_t clean_type (唤醒来源清除类型)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_wake_src_read(stp2_wake_src_t *wake_src, stp2_wake_flag_clean_type_t clean_type)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_wake_src_read -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (wake_src == NULL) {
        ESP_LOGE(TAG, "Wake source pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t wake_src_reg;
    esp_err_t ret;

    // 读取唤醒源寄存器
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_WAKE_SRC, &wake_src_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read wake source register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 解析唤醒源 (低5位)
    uint8_t src_bits = wake_src_reg & 0x3F;
    
    // 初始化为未知唤醒源
    *wake_src = STP2_ADDR_WAKE_SRC_UNKNOWN;
    
    // 找到第一个置1的位
    for (int i = 0; i <= 5; i++) {
        if (src_bits & (1 << i)) {
            // 检查是否在有效范围内 (0-5)
            if (i >= STP2_ADDR_WAKE_SRC_TIM && i <= STP2_ADDR_WAKE_SRC_EXT_WAKE) {
                *wake_src = (stp2_wake_src_t)i;
            } else {
                *wake_src = STP2_ADDR_WAKE_SRC_UNKNOWN;
            }
            break;
        }
    }
    
    ESP_LOGI(TAG, "Wake source: %d (reg value: 0x%02X, bits: 0x%02X)", 
             *wake_src, wake_src_reg, src_bits);
    
    // Clear wake flags based on clean_type
    switch (clean_type) {
        case STP2_ADDR_WAKE_FLAG_NOT_CLEAN:
            // 不清除标志
            break;
        case STP2_ADDR_WAKE_FLAG_ONCE_CLEAN:
            // 清除当前唤醒源标志
            if (*wake_src != STP2_ADDR_WAKE_SRC_UNKNOWN && *wake_src <= STP2_ADDR_WAKE_SRC_EXT_WAKE) {
                uint8_t new_val = wake_src_reg & ~(1 << *wake_src);
                ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_WAKE_SRC, new_val);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to clear wake flag once: %s", esp_err_to_name(ret));
                    return ret;
                }
                ESP_LOGI(TAG, "Cleared wake flag for source %d", *wake_src);
            }
            break;
        case STP2_ADDR_WAKE_FLAG_ALL_CLEAN:
            // 清除所有唤醒源标志
            ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_WAKE_SRC, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to clear all wake flags: %s", esp_err_to_name(ret));
                return ret;
            }
            ESP_LOGI(TAG, "Cleared all wake flags");
            break;
        default:
            ESP_LOGW(TAG, "Unknown clean type: %d", clean_type);
            break;
    }
    
    return ESP_OK;
}

// [STP2_ADDR_SYS_CMD] Write Only
// 操作系统命令寄存器
// INPUIT: stp2_sys_cmd_t cmd (系统命令)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_sys_cmd(stp2_sys_cmd_t cmd)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_sys_cmd -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (cmd > STP2_SYS_CMD_JTAG) {
        ESP_LOGE(TAG, "Invalid system command: %d", cmd);
        return ESP_ERR_INVALID_ARG;
    }

    // 命令格式: [7-4] KEY(0xA) | [3-2] Reserved | [1-0] CMD
    uint8_t cmd_value = (0xA << 4) | (cmd & 0x03);
    
    esp_err_t ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_SYS_CMD, cmd_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send system command: %s", esp_err_to_name(ret));
        return ret;
    }
    
    const char *cmd_str;
    switch (cmd) {
        case STP2_SYS_CMD_NULL:
            cmd_str = "NULL";
            break;
        case STP2_SYS_CMD_SHUTDOWN:
            cmd_str = "SHUTDOWN";
            break;
        case STP2_SYS_CMD_REBOOT:
            cmd_str = "REBOOT";
            break;
        case STP2_SYS_CMD_JTAG:
            cmd_str = "JTAG";
            break;
        default:
            cmd_str = "UNKNOWN";
            break;
    }
    ESP_LOGI(TAG, "System command '%s' sent successfully", cmd_str);
    
    return ESP_OK;
}

// [STP2_ADDR_PWR_CFG]
// 配置电源管理参数（精确控制模式）
// INPUIT: uint8_t mask (要修改的位掩码，1表示该位需要修改)
//         uint8_t value (对应掩码位的目标值，1表示设置为true，0表示设置为false)
//         uint8_t *final_cfg (用于返回最终配置值的指针，可以为NULL)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_pwr_set_cfg(uint8_t mask, uint8_t value, uint8_t *final_cfg)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_pwr_set_cfg -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t current_cfg;
    esp_err_t ret;

    // 先读取当前配置
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_PWR_CFG, &current_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read current power configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    // 确保只有有效位被设置 (bit 0-4)
    uint8_t valid_mask = mask & 0x1F;  // 只保留低5位的掩码
    uint8_t valid_value = value & 0x1F; // 只保留低5位的值
    
    // 精确控制配置：
    // 1. 清除掩码指定的位
    // 2. 设置掩码指定位的新值
    uint8_t new_cfg = (current_cfg & ~valid_mask) | (valid_value & valid_mask);
    
    // 写入新的配置
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_PWR_CFG, new_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write power configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 如果用户提供了返回配置的指针，则返回最终配置
    if (final_cfg != NULL) {
        *final_cfg = new_cfg;
    }
    
    // 记录配置变化
    ESP_LOGI(TAG, "Power configuration updated:");
    ESP_LOGI(TAG, "  Previous: 0x%02X, Mask: 0x%02X, Value: 0x%02X, Final: 0x%02X", 
             current_cfg, valid_mask, valid_value, new_cfg);
    ESP_LOGI(TAG, "  CHG_EN: %s", (new_cfg & STP2_PWR_CFG_CHG_EN) ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  DCDC_EN: %s", (new_cfg & STP2_PWR_CFG_DCDC_EN) ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  LDO_EN: %s", (new_cfg & STP2_PWR_CFG_LDO_EN) ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  5V_INOUT: %s", (new_cfg & STP2_PWR_CFG_5V_INOUT) ? "output mode" : "input mode");
    ESP_LOGI(TAG, "  LED_CONTROL: %s", (new_cfg & STP2_PWR_CFG_LED_CONTROL) ? "enabled" : "disabled");
    
    return ESP_OK;
}

// [STP2_ADDR_PWR_CFG]
// 清除电源管理参数中的特定位
// INPUIT: uint8_t cfg (要清除的电源配置位掩码)
//         uint8_t *final_cfg (用于返回最终配置值的指针，可以为NULL)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_pwr_clear_cfg(uint8_t cfg, uint8_t *final_cfg)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_pwr_clear_cfg -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t current_cfg;
    esp_err_t ret;

    // 先读取当前配置
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_PWR_CFG, &current_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read current power configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    // 确保只有有效位被清除 (bit 0-4)
    uint8_t valid_cfg = cfg & 0x1F;  // 只保留低5位
    
    // 清除配置：将指定位从当前配置中清除
    uint8_t cleared_cfg = current_cfg & (~valid_cfg);
    
    // 写入清除后的配置
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_PWR_CFG, cleared_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write power configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 如果用户提供了返回配置的指针，则返回最终配置
    if (final_cfg != NULL) {
        *final_cfg = cleared_cfg;
    }
    
    ESP_LOGI(TAG, "Power configuration cleared:");
    ESP_LOGI(TAG, "  Previous: 0x%02X, Cleared: 0x%02X, Final: 0x%02X", current_cfg, valid_cfg, cleared_cfg);
    
    return ESP_OK;
}

// [STP2_ADDR_PWR_CFG]
// 读取当前电源管理配置
// INPUIT: uint8_t *current_cfg (用于返回当前配置值的指针)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_pwr_get_cfg(uint8_t *current_cfg)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_pwr_get_cfg -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (current_cfg == NULL) {
        ESP_LOGE(TAG, "Configuration pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_PWR_CFG, current_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read power configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Current power configuration: 0x%02X", *current_cfg);
    ESP_LOGI(TAG, "  CHG_EN: %s", (*current_cfg & STP2_PWR_CFG_CHG_EN) ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  BOOST_EN: %s", (*current_cfg & STP2_PWR_CFG_DCDC_EN) ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  LDO_HOLD: %s", (*current_cfg & STP2_PWR_CFG_LDO_EN) ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  5V_VIN: %s", (*current_cfg & STP2_PWR_CFG_5V_INOUT) ? "output mode" : "input mode");
    ESP_LOGI(TAG, "  LED_CONTROL: %s", (*current_cfg & STP2_PWR_CFG_LED_CONTROL) ? "enabled" : "disabled");
    
    return ESP_OK;
}

// [STP2_ADDR_NEO_REFRESH]
// 刷新NeoPixel数据
// INPUIT: NULL
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_neo_refresh(void)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_neo_refresh -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    // 检查GPIO0是否配置为NeoPixel复用功能
    if (stp2_pin_status[STP2_GPIO_NUM_0].pin_mode != STP2_GPIO_FUNC_OTHER) {
        ESP_LOGE(TAG, "GPIO0 not configured for NeoPixel function");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t neo_cfg;
    esp_err_t ret;

    // 读取当前NeoPixel配置
    ret = i2c_bus_read_byte(stp2_i2c_device, STP2_ADDR_NEO_CFG, &neo_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NeoPixel configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 设置REFRESH位(bit6)
    neo_cfg |= (1 << 6);
    
    // 写入配置以触发刷新
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_NEO_CFG, neo_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger NeoPixel refresh: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "NeoPixel refresh triggered successfully");
    
    return ESP_OK;
}

// [STP2_ADDR_NEO_CFG]
// 配置NeoPixel参数
// INPUIT: uint8_t neo_num (NeoPixel数量, 1-32)
//          uint16_t *neo_data (NeoPixel数据, RGB565格式, 每个像素16位)
//          uint8_t refresh_flag (刷新标志, 1表示立刻刷新，0表示不刷新)
esp_err_t stp2_neo_set_cfg(uint8_t neo_num, uint16_t *neo_data, uint8_t refresh_flag)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_neo_set_cfg -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (neo_num > 32) {
        ESP_LOGE(TAG, "Invalid NeoPixel count: %d (max 32)", neo_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (neo_num > 0 && neo_data == NULL) {
        ESP_LOGE(TAG, "NeoPixel data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // 设置NeoPixel数量
    uint8_t neo_cfg = neo_num & 0x1F;  // 仅使用低5位
    ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_NEO_CFG, neo_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set NeoPixel count: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 如果有像素数据，写入数据
    if (neo_num > 0) {
        // 每个像素占用2字节(RGB565格式)
        for (int i = 0; i < neo_num; i++) {
            uint8_t data_l = neo_data[i] & 0xFF;          // 低字节
            uint8_t data_h = (neo_data[i] >> 8) & 0xFF;   // 高字节
            
            // 写入低字节
            ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_NEO_PIXn_ADDR_START + (i * 2), data_l);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to write NeoPixel %d low byte: %s", i, esp_err_to_name(ret));
                return ret;
            }
            
            // 写入高字节
            ret = i2c_bus_write_byte(stp2_i2c_device, STP2_ADDR_NEO_PIXn_ADDR_START + (i * 2) + 1, data_h);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to write NeoPixel %d high byte: %s", i, esp_err_to_name(ret));
                return ret;
            }
        }
    }
    
    ESP_LOGI(TAG, "NeoPixel configuration set: %d LEDs", neo_num);
    
    // 如果需要立即刷新
    if (refresh_flag) {
        ret = stp2_neo_refresh();
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    return ESP_OK;
}

// [STP2_ADDR_RTC_RAM_WRITE]
// 写RTC RAM
// INPUIT: uin8_t rtc_ram_addr_start (RTC RAM起始地址, 0-31, 对应STP2_ADDR_RTC_MEM_ADDR_START + rtc_ram_addr_start) 
//          uint8_t len (数据长度, 1-32)
//          uint8_t *data (数据缓冲区, 长度为len)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_rtc_ram_write(uint8_t rtc_ram_addr_start, uint8_t len, uint8_t *data)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_rtc_ram_write -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (rtc_ram_addr_start > 31) {
        ESP_LOGE(TAG, "Invalid RTC RAM start address: %d (max 31)", rtc_ram_addr_start);
        return ESP_ERR_INVALID_ARG;
    }

    if (len == 0 || len > 32 || rtc_ram_addr_start + len > 32) {
        ESP_LOGE(TAG, "Invalid data length: %d (1-32, and must not exceed address space)", len);
        return ESP_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    
    // 写入RTC RAM
    for (int i = 0; i < len; i++) {
        uint8_t addr = STP2_ADDR_RTC_MEM_ADDR_START + rtc_ram_addr_start + i;
        ret = i2c_bus_write_byte(stp2_i2c_device, addr, data[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write RTC RAM at address 0x%02X: %s", addr, esp_err_to_name(ret));
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "Wrote %d bytes to RTC RAM starting at address %d (0x%02X)", 
             len, rtc_ram_addr_start, STP2_ADDR_RTC_MEM_ADDR_START + rtc_ram_addr_start);
    
    return ESP_OK;
}

// [STP2_ADDR_RTC_RAM_READ]
// 读RTC RAM
// INPUIT: uin8_t rtc_ram_addr_start (RTC RAM起始地址, 0-31, 对应STP2_ADDR_RTC_MEM_ADDR_START + rtc_ram_addr_start)
//          uint8_t len (数据长度, 1-32)
//          uint8_t *data (数据缓冲区, 长度为len)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t stp2_rtc_ram_read(uint8_t rtc_ram_addr_start, uint8_t len, uint8_t *data)
{
    if (stp2_i2c_device == NULL) {
        ESP_LOGE(TAG, "stp2_rtc_ram_read -> stp2 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (rtc_ram_addr_start > 31) {
        ESP_LOGE(TAG, "Invalid RTC RAM start address: %d (max 31)", rtc_ram_addr_start);
        return ESP_ERR_INVALID_ARG;
    }

    if (len == 0 || len > 32 || rtc_ram_addr_start + len > 32) {
        ESP_LOGE(TAG, "Invalid data length: %d (1-32, and must not exceed address space)", len);
        return ESP_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    
    // 读取RTC RAM
    for (int i = 0; i < len; i++) {
        uint8_t addr = STP2_ADDR_RTC_MEM_ADDR_START + rtc_ram_addr_start + i;
        ret = i2c_bus_read_byte(stp2_i2c_device, addr, &data[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read RTC RAM at address 0x%02X: %s", addr, esp_err_to_name(ret));
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "Read %d bytes from RTC RAM starting at address %d (0x%02X)", 
             len, rtc_ram_addr_start, STP2_ADDR_RTC_MEM_ADDR_START + rtc_ram_addr_start);
    
    return ESP_OK;
}