#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "i2c_bus.h"
#include "m5_stamp_timer_power2.h"
#include "i2c_bus_tools.h"
#include "stamp_case.h"
#include <stdio.h>
#include <string.h>

#define TAG "M5-STAMP-S3BAT-Test"

#define I2C_ADDR_STP2 0x6E

extern "C" void app_main(void)
{

    register_i2c_bus_device();
    scan_and_add_i2c_bus_devices();

    // STP2 控制器初始化
    esp_err_t ret = stp2_init(g_i2c_bus, &i2c_device_all[I2C_ADDR_STP2]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize STP2: %s", esp_err_to_name(ret));
        return;
    }

    // vTaskDelay(3000 / portTICK_PERIOD_MS);

    stamp_py32_test_case_1();  // VBAT_ADC采集测试

    // stamp_py32_test_case_2();  // VIN_ADC采集测试

    // stamp_py32_test_case_3();  // VIN_DET检测测试

    // stamp_py32_test_case_4();  // boost输出测试

    // stamp_py32_test_case_5();  // WS2812测试

    // stamp_py32_test_case_6();  // 充电测试

    // stamp_py32_test_case_7();  // 调整充电电流

    // stamp_py32_test_case_8();  // 读取充电状态

    // stamp_py32_test_case_9();  // 外部中断唤醒PY32

    // stamp_py32_test_case_10(); // WIFI测试

    // stamp_py32_test_case_11(); // gpio测试



}
