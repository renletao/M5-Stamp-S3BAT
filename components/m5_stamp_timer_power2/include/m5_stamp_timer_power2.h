#ifndef __STP2_CONTROL_H__
#define __STP2_CONTROL_H__

#include "esp_err.h"
#include <stdint.h>

#include "i2c_bus.h"
#define STP2_DEV_VERSION "0.0.1" // STP2驱动版本号

// Configuration from Kconfig or defaults
#ifndef CONFIG_STP2_I2C_ADDRESS
#define CONFIG_STP2_I2C_ADDRESS 0x6E
#endif

#ifndef CONFIG_STP2_I2C_FREQUENCY  
#define CONFIG_STP2_I2C_FREQUENCY 400000
#endif

#ifndef I2C_PY32F002_ADDR
#define I2C_PY32F002_ADDR CONFIG_STP2_I2C_ADDRESS // STP2 I2C address
#endif

#ifndef I2C_PY32F002_FREQ_HZ
#define I2C_PY32F002_FREQ_HZ CONFIG_STP2_I2C_FREQUENCY // STP2 I2C frequency
#endif

#define STP2_ADDR_UID_L 0x00            // [7-0] UID_L
#define STP2_ADDR_UID_H 0x01            // [7-0] UID_H
#define STP2_ADDR_HW_REV 0x02           // [7-0] 硬件版本号
#define STP2_ADDR_SW_REV 0x03           // [7-0] 软件/固件版本号
#define STP2_ADDR_GPIO_MODE 0x04        // [7-5] Reserved | [4-0] GPIO模式寄存器 1:输出 | 0:输入
#define STP2_ADDR_GPIO_OUT 0x05         // [7-5] Reserved | [4-0] GPIO输出寄存器 1:高 | 0:低
#define STP2_ADDR_GPIO_IN 0x06          // [7-5] Reserved | [4-0] GPIO输入寄存器 实时输入的值
#define STP2_ADDR_GPIO_DRV 0x07         // [7-5] Reserved | [5] LED_EN_DRV | [4-0] GPIO驱动寄存器 1:开漏 | 0:推挽
#define STP2_ADDR_ADC_RES_L 0x08        // [7-0] ADC结果低8位
#define STP2_ADDR_ADC_RES_H 0x09        // [7-4] Reserved | [3-0] ADC结果高4位
#define STP2_ADDR_ADC_CTRL 0x0A         // [7-4] Reserved | [3-1] 通道选择(1[gpio]\2[gpio]\6[temp]) | [0] 开始转换 注：FUNC必须为11
// #define STP2_ADDR_
//   占空比 0-4096
#define STP2_ADDR_PWM0_L 0x0C           // [7-0] PWM0占空比低8位
#define STP2_ADDR_PWM0_HC 0x0D          // [7-6] Reserved | [5] 极性 | [4] 使能 | [3-0] 占空比高4位
#define STP2_ADDR_PWM1_L 0x0E           // [7-0] PWM1占空比低8位
#define STP2_ADDR_PWM1_HC 0x0F          // [7-6] Reserved | [5] 极性 | [4] 使能 | [3-0] 占空比高4位
//   00:GPIO | 01:IRQ | 10:WAKE | 11:LED/PWM/ADC
#define STP2_ADDR_GPIO_FUNC0 0x10       // [7-6] GPIO3 | [5-4] GPIO2 | [3-2] GPIO1 | [1-0] GPIO0
#define STP2_ADDR_GPIO_FUNC1 0x11       // [1-0] GPIO4 | 其余位保留
#define STP2_ADDR_GPIO_PUPD0 0x12       // [7-6] GPIO3 | [5-4] GPIO2 | [3-2] GPIO1 | [1-0] GPIO0 上下拉配置
#define STP2_ADDR_GPIO_PUPD1 0x13       // [1-0] GPIO4 上下拉配置 | 其余位保留
#define STP2_ADDR_WDT_CNT 0x14          // [7-0] 看门狗倒计时(秒) 0:不启用看门狗, 1-255s:看门狗时限
#define STP2_ADDR_WDT_KEY 0x15          // [7-0] 在该寄存器写0xA5以喂狗
#define STP2_ADDR_GPIO_WAKE_EN 0x16     // [7-5] Reserved | [4-0] GPIO唤醒使能
#define STP2_ADDR_GPIO_WAKE_CFG 0x17    // [7-5] Reserved | [4-0] GPIO唤醒边沿配置
//   定时唤醒计数器 寄存器 范围 0-0x7FFFFFFF 单位为秒，约合最长68.1年。
#define STP2_ADDR_TIM_CNT_BYTE_0 0x18             // [7-0] 定时唤醒计数器 Byte0
#define STP2_ADDR_TIM_CNT_BYTE_1 0x19             // [7-0] 定时唤醒计数器 Byte1
#define STP2_ADDR_TIM_CNT_BYTE_2 0x1A             // [7-4] 定时唤醒计数器 Byte2
#define STP2_ADDR_TIM_CNT_BYTE_3 0x1B             // [7] Reserved | [6-0] 定时唤醒计数器 Byte3
#define STP2_ADDR_PWR_CFG 0x1C          // [7-5] Reserved [4] LED_CONTROL | [3] 5V_INOUT | [2] LDO_EN | [1] DCDC_EN | [0] CHG_EN
//   ACTION: 000：停止计数器 | 001：仅置位 WAKE 标志 | 010：系统重启 | 011：系统上电 | 100：系统关机
#define STP2_ADDR_TIM_CFG 0x1D          // [7-4] Reserved | [3] ARM(是否自动清零) | [2-0] ACTION
#define STP2_ADDR_PWM_FREQ_L 0x1E       // [7-0] PWM频率低8位
#define STP2_ADDR_PWM_FREQ_H 0x1F       // [7-0] PWM频率高8位
#define STP2_ADDR_BTN_CFG 0x20          // [7] DL_LOCK | [6-5] DBL | [4-3] LONG | [2-1] SINGLE | [0] Reserved
#define STP2_ADDR_IRQ_STATUS1 0x21      // [7-5] Reserved | [4-0] GPIO中断状态 [4:GPIO4 3:GPIO3 2:GPIO2 1:GPIO1 0:GPIO0]
// 备注：电池拔插仅在充电使能为打开时有效，5VINOUT插拔仅在设置为输入时有效
#define STP2_ADDR_IRQ_STATUS2 0x22      // [7-6] Reserved | [5] 电池移除 | [4] 电池插入 | [3] 5VINOUT移除 | [2] 5VINOUT插入 | [1] 5V IN移除 | [0] 5V IN插入
// #define STP2_ADDR_
#define STP2_ADDR_BATT_LVP 0x24         // [7-0] 低压阈值:2000mV+n×7.81mV
#define STP2_ADDR_TIM_KEY 0x25          // [7-0] 写0xA5清零并重载
#define STP2_ADDR_VREF_L 0x26           // [7-0] VREF低8位
#define STP2_ADDR_VREF_H 0x27           // [7-0] VREF高8位
#define STP2_ADDR_VBAT_L 0x28           // [7-0] 电池电压低8位
#define STP2_ADDR_VBAT_H 0x29           // [7-4] Reserved | [3-0] 电池电压高4位
#define STP2_ADDR_VIN_L 0x2A            // [7-0] VIN电压低8位
#define STP2_ADDR_VIN_H 0x2B            // [7-4] Reserved | [3-0] VIN电压高4位
#define STP2_ADDR_VBUS_L 0x2C           // [7-0] BUS电压低8位
#define STP2_ADDR_VBUS_H 0x2D           // [7-4] Reserved | [3-0] BUS电压高4位
#define STP2_ADDR_PWR_SRC 0x2E          // [7-3] Reserved | [2-0] VALID 0:5VIN 1:5VINOUT 2:BAT
#define STP2_ADDR_WAKE_SRC 0x2F         // [7-5] Reserved | [4-0] 唤醒源标志 0:TIM 1:VIN 2:PWRBTN 3:RSTBTN 4:CMD_RST 5:EXT_WAKE
#define STP2_ADDR_I2C_CFG 0x30          // [7-5] Reserved | [4] SPD 0->100K 1:400K | [3-0] SLP_TO
// #define STP2_ADDR_
#define STP2_ADDR_SYS_CMD 0x32          // [7-4] KEY(0xA) | [3-2] Reserved | [1-0] CMD 00:无 01:关机 10:重启 11:下载
// #define STP2_ADDR_
#define STP2_ADDR_NEO_CFG 0x34          // [7-6] Reserved | [5] REFRESH | [4-0] LED_CNT
// #define STP2_ADDR_
// 例如 rgb1_red:36H[4:0] rgb1_green:36H[5:3]&37H[2:0] rgb1_blue:37H[4:0] ...
#define STP2_ADDR_NEO_PIXn_ADDR_START 0x36 // [多个地址] NeoPixel像素RGB565数据起始地址
#define STP2_ADDR_NEO_PIXn_ADDR_END 0x75   // NeoPixel像素数据结束地址
// #define STP2_ADDR_
#define STP2_ADDR_RTC_MEM_ADDR_START 0x80  // [多个地址] 32字节RTC备份RAM起始地址
#define STP2_ADDR_RTC_MEM_ADDR_END 0x9F    // RTC备份RAM结束地址
// #define STP2_ADDR_
// #define STP2_ADDR_JTAG_MODE 0xF0         // [7-0]写0x01进入JTAG模式 弃用、不支持

// GPIO Function Types
typedef enum  {
    STP2_GPIO_FUNC_GPIO = 0b00,
    STP2_GPIO_FUNC_IRQ = 0b01,
    STP2_GPIO_FUNC_RES = 0b10,      //保留 reserve
    STP2_GPIO_FUNC_OTHER = 0b11     //LED:频率固定为24Mhz/PWM/ADC
} stp2_gpio_func_t;

// GPIO Number Types
typedef enum  {
    STP2_GPIO_NUM_0 = 0,
    STP2_GPIO_NUM_1 = 1,
    STP2_GPIO_NUM_2 = 2,
    STP2_GPIO_NUM_3 = 3,
    STP2_GPIO_NUM_4 = 4,
} stp2_gpio_num_t;

// GPIO Mode Types
typedef enum {
    STP2_GPIO_MODE_INPUT = 0b00,   // 输入模式
    STP2_GPIO_MODE_OUTPUT = 0b01,  // 输出模式
} stp2_gpio_mode_t;

// GPIO State Types
typedef enum {
    STP2_GPIO_OUTPUT_LOW = 0b0,     // 低电平
    STP2_GPIO_OUTPUT_HIGH = 0b1,    // 高电平
    STP2_GPIO_INPUT_NC = 0b0,    // 输入模式下不关心状态
} stp2_gpio_state_t;

// GPIO Pull-up/Pull-down Types
typedef enum {
    STP2_GPIO_PUPD_NC = 0b00,       // 不配置上下拉
    STP2_GPIO_PUPD_PULLUP = 0b01,   // 上拉
    STP2_GPIO_PUPD_PULLDOWN = 0b10, // 下拉
} stp2_gpio_pupd_t;

// GPIO Input State Types
typedef enum {
    STP2_GPIO_IN_STATE_LOW = 0b0,   // 输入状态低
    STP2_GPIO_IN_STATE_HIGH = 0b1,  // 输入状态高
} stp2_gpio_in_state_t;

// GPIO Drive Mode Types
typedef enum {
    STP2_GPIO_DRV_PUSH_PULL = 0b0,   // 推挽输出
    STP2_GPIO_DRV_OPEN_DRAIN = 0b1,  // 开漏输出
} stp2_gpio_drv_t;

// ADC Channel Types
typedef enum {
    STP2_ADC_CHANNEL_0 = 0,      // ADC通道0  0-4095 [本项目不支持]
    STP2_ADC_CHANNEL_1 = 1,      // ADC通道1  0-4095
    STP2_ADC_CHANNEL_2 = 2,      // ADC通道2  0-4095
    STP2_ADC_CHANNEL_3 = 3,      // ADC通道3  0-4095 [本项目不支持]
    STP2_ADC_CHANNEL_4 = 4,      // ADC通道4  0-4095 [本项目不支持]
    STP2_ADC_CHANNEL_5 = 5,      // ADC通道5  0-4095 [本项目不支持]
    STP2_ADC_CHANNEL_TEMP = 6,   // ADC通道6  MCU内部温度探针 单位：摄氏度
} stp2_adc_channel_t;

// ADC Control Types
typedef enum {
    STP2_ADC_CTRL_DISABLE = 0b0, // ADC禁用
    STP2_ADC_CTRL_ENABLE = 0b1,  // ADC使能
} stp2_adc_ctrl_t;

// PWM Channel Types
typedef enum {
    STP2_PWM_CHANNEL_0 = 0,      // PWM通道0
    STP2_PWM_CHANNEL_1 = 1,      // PWM通道1
} stp2_pwm_channel_t;

// PWM Control Types
typedef enum {
    STP2_PWM_CTRL_DISABLE = 0b0, // PWM禁用
    STP2_PWM_CTRL_ENABLE = 0b1,  // PWM使能
} stp2_pwm_ctrl_t;

// PWM Polarity Types
typedef enum {
    STP2_PWM_POLARITY_NORMAL = 0b0, // 正常极性
    STP2_PWM_POLARITY_INVERTED = 0b1, // 反向极性
} stp2_pwm_polarity_t;

// WDT Control Types
typedef enum {
    STP2_WDT_CTRL_DISABLE = 0b0, // WDT禁用
    STP2_WDT_CTRL_ENABLE = 0b1,  // WDT使能
} stp2_wdt_ctrl_t;

// Timer Control Types
typedef enum {
    STP2_ADDR_TIM_DISABLE = 0b0, // 定时器禁用
    STP2_ADDR_TIM_ENABLE = 0b1,  // 定时器使能
} stp2_tim_ctrl_t;

// Timer Action Types
typedef enum {
    STP2_TIM_ACTION_000 = 0b000, // 停止计数器
    STP2_TIM_ACTION_001 = 0b001, // 仅置位 WAKE 标志
    STP2_TIM_ACTION_010 = 0b010, // 系统重启
    STP2_TIM_ACTION_011 = 0b011, // 系统上电
    STP2_TIM_ACTION_100 = 0b100, // 系统关机
} stp2_tim_action_t;

// I2C Clock Speed Types
typedef enum {
    STP2_CLK_SPEED_100KHZ = 0, // 100KHz MCU_CLK:3MHz
    STP2_CLK_SPEED_400KHZ = 1, // 400KHz MCU_CLK:12MHz （功耗增加）
} stp2_clk_speed_t;

// I2C ACK Check Types
// [0:直到唤醒为止，第二个参数为超时参数，单位为毫秒，0表示不超时]
// [1:尝试唤醒一定次数，第二个参数为唤醒次数，每次唤醒间隔为500ms]
typedef enum {
    STP2_I2C_ACK_CHECK_UNTIL_WAKE = 0, // I2C尝试唤醒直到成功
    STP2_I2C_ACK_CHECK_TRY_TIMES = 1,   // I2C尝试唤醒指定次数
} stp2_i2c_ack_check_t;


// JTAG Mode Types
typedef enum {
    STP2_JTAG_MODE_NORMAL = 0, // Normal Mode
    STP2_JTAG_MODE_JTAG = 1,   // JTAG Mode
} stp2_jtag_mode_t;

// GPIO Pull-up/Pull-down Types
typedef enum {
    STP2_GPIO_PULL_NO = 0b00,   // 无上拉下拉
    STP2_GPIO_PULL_UP = 0b01,   // 上拉
    STP2_GPIO_PULL_DOWN = 0b11, // 下拉
} stp2_gpio_pull_t;

// GPIO Wake Types
typedef enum {
    STP2_GPIO_WAKE_DISABLE = 0b0, // 禁用唤醒
    STP2_GPIO_WAKE_ENABLE = 0b1,  // 使能唤醒
} stp2_gpio_wake_t;

// GPIO Wake Edge Types
typedef enum {
    STP2_GPIO_WAKE_FALLING = 0b0, // 下降沿唤醒
    STP2_GPIO_WAKE_RISING = 0b1,  // 上升沿唤醒
} stp2_gpio_wake_edge_t;

// Download Enable Types
typedef enum {
    STP2_ADDR_DOWNLOAD_DISABLE = 0x00, // 禁用下载模式
    STP2_ADDR_DOWNLOAD_ENABLE = 0x01   // 启用下载模式
} stp2_download_enable_t;

// Button Types
typedef enum {
    STP2_ADDR_BTN_TYPE_CLICK = 0x00,        // 点击
    STP2_ADDR_BTN_TYPE_DOUBLE_CLICK = 0x01, // 双击
    STP2_ADDR_BTN_TYPE_LONG_PRESS = 0x02    // 长按
} stp2_btn_type_t;

// Button Delay Types
typedef enum {
    STP2_ADDR_BTN_CLICK_DELAY_125MS = 0x00,          // 125毫秒
    STP2_ADDR_BTN_CLICK_DELAY_250MS = 0x01,          // 250毫秒
    STP2_ADDR_BTN_CLICK_DELAY_500MS = 0x02,          // 500毫秒
    STP2_ADDR_BTN_CLICK_DELAY_1000MS = 0x03,         // 1000毫秒
    STP2_ADDR_BTN_DOUBLE_CLICK_DELAY_125MS = 0x00,   // 双击125毫秒
    STP2_ADDR_BTN_DOUBLE_CLICK_DELAY_250MS = 0x01,   // 双击250毫秒
    STP2_ADDR_BTN_DOUBLE_CLICK_DELAY_500MS = 0x02,   // 双击500毫秒
    STP2_ADDR_BTN_DOUBLE_CLICK_DELAY_1000MS = 0x03,  // 双击1000毫秒
    STP2_ADDR_BTN_LONG_PRESS_DELAY_1000MS = 0x00,    // 长按1000毫秒
    STP2_ADDR_BTN_LONG_PRESS_DELAY_2000MS = 0x01,    // 长按2000毫秒
    STP2_ADDR_BTN_LONG_PRESS_DELAY_3000MS = 0x02,    // 长按3000毫秒
    STP2_ADDR_BTN_LONG_PRESS_DELAY_4000MS = 0x03     // 长按4000毫秒
} stp2_btn_delay_t;

// GPIO IRQ Types
typedef enum {
    STP2_ADDR_IRQ_GPIO0 = 0x00,
    STP2_ADDR_IRQ_GPIO1 = 0x01,
    STP2_ADDR_IRQ_GPIO2 = 0x02,
    STP2_ADDR_IRQ_GPIO3 = 0x03,
    STP2_ADDR_IRQ_GPIO4 = 0x04,
    STP2_ADDR_IRQ_NULL  = 0xFF,
} stp2_irq_gpio_t;

// GPIO IRQ Clean Types
typedef enum {
    STP2_ADDR_IRQ_GPIO_NOT_CLEAN = 0x00, // 不清除中断
    STP2_ADDR_IRQ_GPIO_ONCE_CLEAN = 0x01,    // 清除一次中断
    STP2_ADDR_IRQ_GPIO_ALL_CLEAN = 0x02      // 清除所有中断
} stp2_irq_gpio_clean_type_t;

// System IRQ Types
typedef enum {
    STP2_ADDR_IRQ_SYS_BAT_REMOVE = 0x05,    // 电池移除
    STP2_ADDR_IRQ_SYS_BAT_INSERT = 0x04,    // 电池插入
    STP2_ADDR_IRQ_SYS_5VINOUT_REMOVE = 0x03,  // 5VINOUT移除
    STP2_ADDR_IRQ_SYS_5VINOUT_INSERT = 0x02,  // 5VINOUT插入
    STP2_ADDR_IRQ_SYS_5VIN_REMOVE = 0x01,   // 5VIN移除
    STP2_ADDR_IRQ_SYS_5VIN_INSERT = 0x00,   // 5VIN插入
    STP2_ADDR_IRQ_SYS_NULL = 0xFF
} stp2_irq_sys_t;

// System IRQ Clean Types
typedef enum {
    STP2_ADDR_IRQ_SYS_NOT_CLEAN = 0x00, // 不清除中断
    STP2_ADDR_IRQ_SYS_ONCE_CLEAN = 0x01,    // 清除一次中断
    STP2_ADDR_IRQ_SYS_ALL_CLEAN = 0x02      // 清除所有中断
} stp2_irq_sys_clean_type_t;

// Power Source Types
typedef enum {
    STP2_PWR_SRC_5VIN = 0x00,    // 5VIN供电
    STP2_PWR_SRC_5VINOUT = 0x01,   // 5VINOUT供电
    STP2_PWR_SRC_BAT = 0x02,     // 电池供电
    STP2_PWR_SRC_UNKNOWN = 0x03  // 未知电源
} stp2_pwr_src_t;

// Wake Source Types
typedef enum {
    STP2_ADDR_WAKE_SRC_TIM = 0x00,     // 定时器唤醒
    STP2_ADDR_WAKE_SRC_VIN = 0x01,     // VIN唤醒
    STP2_ADDR_WAKE_SRC_PWRBTN = 0x02,  // 电源按键唤醒
    STP2_ADDR_WAKE_SRC_RSTBTN = 0x03,  // 复位按键唤醒
    STP2_ADDR_WAKE_SRC_CMD_RST = 0x04, // 命令复位唤醒
    STP2_ADDR_WAKE_SRC_EXT_WAKE = 0x05, // 外部WAKE引脚唤醒
    STP2_ADDR_WAKE_SRC_UNKNOWN = 0xFF   // 未知唤醒源
} stp2_wake_src_t;

// WAKE Flag Clean Types
typedef enum {
    STP2_ADDR_WAKE_FLAG_NOT_CLEAN = 0x00, // 不清除中断
    STP2_ADDR_WAKE_FLAG_ONCE_CLEAN = 0x01,    // 清除一次中断
    STP2_ADDR_WAKE_FLAG_ALL_CLEAN = 0x02      // 清除所有中断
} stp2_wake_flag_clean_type_t;

// System Command Types
typedef enum {
    STP2_SYS_CMD_NULL = 0x00,     // 无命令
    STP2_SYS_CMD_SHUTDOWN = 0x01, // 关机
    STP2_SYS_CMD_REBOOT = 0x02,   // 重启
    STP2_SYS_CMD_JTAG = 0x03,     // JTAG下载
} stp2_sys_cmd_t;

// PWR CFG Types
typedef enum {
    STP2_PWR_CFG_CHG_EN      = 0x01, // [0] CHG_EN 充电使能
    STP2_PWR_CFG_DCDC_EN     = 0x02, // [1] DCDC_EN 升压使能
    STP2_PWR_CFG_LDO_EN      = 0x04, // [2] LDO_EN LDO保持
    STP2_PWR_CFG_5V_INOUT    = 0x08, // [3] 5V_INOUT 5V输入输出
    STP2_PWR_CFG_LED_CONTROL = 0x10  // [4] LED_CONTROL LED控制
} stp2_pwr_cfg_t;

// Function Declarations
esp_err_t stp2_init(i2c_bus_handle_t i2c_bus,i2c_bus_device_handle_t *i2c_device);
esp_err_t stp2_deinit(void);
esp_err_t stp2_set_clk_speed(stp2_clk_speed_t clk_speed_flag);
esp_err_t stp2_set_i2c_sleep_time(uint8_t sleep_time_sec);
esp_err_t stp2_i2c_try_wake(stp2_i2c_ack_check_t ack_check_type, uint32_t timeout_or_try_times);
esp_err_t stp2_set_jtag_mode(stp2_jtag_mode_t jtag_mode);

// Device Information Functions
esp_err_t stp2_get_hw_version(uint8_t *hw_version);
esp_err_t stp2_get_sw_version(uint8_t *sw_version);
esp_err_t stp2_get_chip_id(uint16_t *chip_id);

// GPIO Functions
esp_err_t stp2_gpio_set_func(stp2_gpio_num_t gpio_num, stp2_gpio_func_t func);
esp_err_t stp2_gpio_set(stp2_gpio_num_t gpio_num, stp2_gpio_mode_t mode, stp2_gpio_state_t state, stp2_gpio_pupd_t pupd, stp2_gpio_drv_t drv_mode);
esp_err_t stp2_gpio_set_mode(stp2_gpio_num_t gpio_num, stp2_gpio_mode_t mode);
esp_err_t stp2_gpio_set_state(stp2_gpio_num_t gpio_num, stp2_gpio_state_t state);
esp_err_t stp2_gpio_set_pupd(stp2_gpio_num_t gpio_num, stp2_gpio_pupd_t pupd);
esp_err_t stp2_gpio_set_drv(stp2_gpio_num_t gpio_num, stp2_gpio_drv_t drv_mode);
esp_err_t stp2_led_en_set_drv(stp2_gpio_drv_t drv_mode);
esp_err_t stp2_gpio_set_wake_en(stp2_gpio_num_t gpio_num, stp2_gpio_wake_t wake_en);
esp_err_t stp2_gpio_set_wake_cfg(stp2_gpio_num_t gpio_num, stp2_gpio_wake_edge_t wake_cfg);
esp_err_t stp2_gpio_get_in_state(stp2_gpio_num_t gpio_num, stp2_gpio_in_state_t *state);

// ADC Functions
esp_err_t stp2_adc_read(stp2_adc_channel_t channel, uint16_t *adc_value);

// PWM Functions
esp_err_t stp2_pwm_set(stp2_pwm_channel_t channel, stp2_pwm_ctrl_t ctrl, stp2_pwm_polarity_t polarity, uint32_t pwm_freq_16bit_value, uint32_t duty_cycle_12bit_value);

// WDT Functions
esp_err_t stp2_wdt_set(stp2_wdt_ctrl_t ctrl, uint8_t timeout_s);
esp_err_t stp2_wdt_feed(uint8_t key);
esp_err_t stp2_wdt_get_wdt_cnt(uint8_t *wdt_cnt);

// Timer Functions
esp_err_t stp2_tim_set(stp2_tim_ctrl_t ctrl, stp2_tim_action_t action, uint32_t tim_ct_31bit_value);
esp_err_t stp2_tim_clear(uint8_t key);

// Download Functions
esp_err_t stp2_download_enable(stp2_download_enable_t enable);

// Button Functions
esp_err_t stp2_btn_set_cfg(stp2_btn_type_t btn_type, stp2_btn_delay_t delay);

// IRQ Functions
esp_err_t stp2_irq_get_status(stp2_irq_gpio_t *gpio_num, stp2_irq_gpio_clean_type_t clean_type);
esp_err_t stp2_irq_get_sys_status(stp2_irq_sys_t *sys_irq, stp2_irq_sys_clean_type_t clean_type);

// Battery and Voltage Functions
esp_err_t stp2_batt_set_lvp(uint16_t lvp_mv);
esp_err_t stp2_vref_read(uint16_t *vref_mv);
esp_err_t stp2_vbat_read(uint16_t *vbat_mv);
esp_err_t stp2_vin_read(uint16_t *vin_mv);
esp_err_t stp2_vbus_read(uint16_t *vbus_mv);

// Power and Wake Functions
esp_err_t stp2_pwr_src_read(stp2_pwr_src_t *pwr_src);
esp_err_t stp2_pwr_set_cfg(uint8_t mask, uint8_t value, uint8_t *final_cfg);
esp_err_t stp2_pwr_clear_cfg(uint8_t cfg, uint8_t *final_cfg);
esp_err_t stp2_pwr_get_cfg(uint8_t *current_cfg);
esp_err_t stp2_wake_src_read(stp2_wake_src_t *wake_src, stp2_wake_flag_clean_type_t clean_type);

// System Command Functions
esp_err_t stp2_sys_cmd(stp2_sys_cmd_t cmd);

// NeoPixel Functions
esp_err_t stp2_neo_refresh(void);
esp_err_t stp2_neo_set_cfg(uint8_t neo_num, uint16_t *neo_data, uint8_t refresh_flag);

// RTC RAM Functions
esp_err_t stp2_rtc_ram_write(uint8_t rtc_ram_addr_start, uint8_t len, uint8_t *data);
esp_err_t stp2_rtc_ram_read(uint8_t rtc_ram_addr_start, uint8_t len, uint8_t *data);

#endif // __STP2_CONTROL_H__