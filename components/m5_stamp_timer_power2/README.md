# M5 Stamp Timer Power2 (STP2) Component

这是一个用于 ESP-IDF 5.3+ 的 M5 Stamp Timer Power2 (STP2) 控制库组件。

## 功能特性

- GPIO 控制和配置
- ADC 读取 (包括内部温度传感器)
- PWM 输出控制
- 看门狗定时器
- 定时器和唤醒功能
- 电源管理
- 电池和电压监测
- 中断状态查询
- NeoPixel LED 控制
- RTC RAM 存储

## 安装

### 方法 1: 作为项目组件 (推荐)

将此组件文件夹复制到您的 ESP-IDF 项目的 `components/` 目录下：

```
your_project/
├── components/
│   └── m5_stamp_timer_power2/
├── main/
├── CMakeLists.txt
└── ...
```

### 方法 2: 作为管理组件

如果您想将此组件发布到组件注册表，可以使用 ESP 组件管理器。

## 依赖

- ESP-IDF 5.3 或更高版本
- `espressif/i2c_bus` 组件

## 配置

可以通过 `idf.py menuconfig` 配置以下选项：

- STP2 I2C 地址 (默认: 0x6E)
- STP2 I2C 频率 (默认: 100000 Hz)
- 调试日志开关
- 看门狗默认超时时间
- GPIO 默认上拉配置

## 基本用法

### 1. 包含头文件

```c
#include "stp2_control.h"
#include "i2c_bus.h"
```

### 2. 初始化

```c
// 首先初始化 I2C 总线
i2c_config_t i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = GPIO_NUM_21,
    .scl_io_num = GPIO_NUM_22,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,
};

i2c_bus_handle_t i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_config);

// 初始化 STP2
esp_err_t ret = stp2_init(i2c_bus);
if (ret != ESP_OK) {
    ESP_LOGE("APP", "Failed to initialize STP2: %s", esp_err_to_name(ret));
    return;
}
```

### 3. GPIO 控制

```c
// 设置 GPIO0 为输出模式
stp2_gpio_set_func(STP2_GPIO_NUM_0, STP2_GPIO_FUNC_GPIO);
stp2_gpio_set_mode(STP2_GPIO_NUM_0, STP2_GPIO_MODE_OUTPUT, 
                   STP2_GPIO_OUTPUT_HIGH, STP2_GPIO_PUPD_NC);

// 读取 GPIO1 输入状态
stp2_gpio_in_state_t state;
stp2_gpio_get_in_state(STP2_GPIO_NUM_1, &state);
```

### 4. ADC 读取

```c
// 读取 ADC 通道1
uint32_t adc_value;
esp_err_t ret = stp2_adc_read(STP2_ADC_CHANNEL_1, &adc_value);
if (ret == ESP_OK) {
    ESP_LOGI("APP", "ADC Channel 1: %lu", adc_value);
}

// 读取内部温度
uint32_t temperature;
ret = stp2_adc_read(STP2_ADC_CHANNEL_TEMP, &temperature);
if (ret == ESP_OK) {
    ESP_LOGI("APP", "Internal Temperature: %lu°C", temperature);
}
```

### 5. PWM 控制

```c
// 设置 PWM0: 1KHz, 50% 占空比
uint32_t pwm_freq = 1000;      // 1KHz
uint32_t duty_cycle = 2048;    // 50% (0-4095)

stp2_pwm_set(STP2_PWM_CHANNEL_0, STP2_PWM_CTRL_ENABLE, 
             STP2_PWM_POLARITY_NORMAL, pwm_freq, duty_cycle);
```

### 6. 电压监测

```c
// 读取电池电压
uint32_t battery_voltage;
stp2_vbat_read(&battery_voltage);
ESP_LOGI("APP", "Battery Voltage: %lu mV", battery_voltage);

// 读取输入电压
uint32_t input_voltage;
stp2_vin_read(&input_voltage);
ESP_LOGI("APP", "Input Voltage: %lu mV", input_voltage);
```

### 7. 清理

```c
// 程序结束时清理资源
stp2_deinit();
i2c_bus_delete(&i2c_bus);
```

## API 参考

详细的 API 参考请查看 `include/stp2_control.h` 头文件中的函数声明和注释。

主要函数分类：

- **初始化**: `stp2_init()`, `stp2_deinit()`
- **GPIO**: `stp2_gpio_set_func()`, `stp2_gpio_set_mode()`, `stp2_gpio_get_in_state()`
- **ADC**: `stp2_adc_read()`
- **PWM**: `stp2_pwm_set()`
- **定时器**: `stp2_tim_set()`, `stp2_tim_clear()`
- **看门狗**: `stp2_wdt_set()`, `stp2_wdt_feed()`
- **电源管理**: `stp2_pwr_set_cfg()`, `stp2_pwr_get_cfg()`
- **电压监测**: `stp2_vbat_read()`, `stp2_vin_read()`, `stp2_vbus_read()`
- **中断**: `stp2_irq_get_status()`, `stp2_irq_get_sys_status()`
- **NeoPixel**: `stp2_neo_set_cfg()`, `stp2_neo_refresh()`
- **RTC RAM**: `stp2_rtc_ram_read()`, `stp2_rtc_ram_write()`

## 注意事项

1. 在调用任何其他 STP2 函数之前，必须先调用 `stp2_init()`
2. 确保 I2C 总线已正确初始化并传递给 `stp2_init()`
3. STP2 支持的 I2C 频率为 100KHz 和 400KHz
4. 部分 ADC 通道在当前项目中可能不被支持
5. PWM 频率范围和占空比范围请参考硬件文档

## 许可证

MIT License

## 支持

如有问题，请联系 M5Stack 技术支持。
