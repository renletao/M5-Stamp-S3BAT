#include "stamp_case.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"
#include "nvs_flash.h"
#include "soc/rtc.h"
#include "string.h"
#include <math.h>
#include "m5_stamp_timer_power2.h"

#define TAG "TEST_CASE"

#define DEFAULT_SCAN_LIST_SIZE 15
#define GPIO_INPUT_IO_1        (1)
#define TIMEOUT_PERIOD         (1000000)

esp_timer_handle_t periodic_timer;

esp_timer_create_args_t timer_args = {.callback = NULL, .arg = NULL, .name = "32k_timer"};

void stamp_py32_test_case_1(void)
{
    uint16_t vbat_voltage = 0;
    while (1) {
        if (stp2_vbat_read(&vbat_voltage) != ESP_OK) {
            ESP_LOGE(TAG, "get vbat voltage failed");
        }
        stp2_wdt_feed(0xA5);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void stamp_py32_test_case_2(void)
{
    uint16_t vbus_voltage = 0;
    while (1) {
        if (stp2_vbus_read(&vbus_voltage) != ESP_OK) {
            ESP_LOGE(TAG, "get vbus voltage failed");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void stamp_py32_test_case_3(void)
{
    uint16_t vin_det_voltage = 0;
    while (1) {
        if (stp2_vin_read(&vin_det_voltage) != ESP_OK) {
            ESP_LOGE(TAG, "get vin det failed");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void stamp_py32_test_case_4(void)

{
    if (stp2_pwr_set_cfg(STP2_PWR_CFG_5V_INOUT, STP2_PWR_CFG_5V_INOUT, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "set pwr cfg failed");
    } else {
        ESP_LOGI(TAG, "set pwr cfg success");
    }

    while (1) {
        // if (stp2_pwr_set_cfg(STP2_PWR_CFG_5V_INOUT,STP2_PWR_CFG_5V_INOUT,NULL) != ESP_OK) {
        //     ESP_LOGE(TAG, "set pwr cfg failed");
        // } else {
        //     ESP_LOGI(TAG, "set pwr cfg success");
        // }

        // vTaskDelay(10000 / portTICK_PERIOD_MS);

        // if (stp2_pwr_set_cfg(STP2_PWR_CFG_5V_INOUT,STP2_PWR_CFG_5V_INOUT,NULL) != ESP_OK) {
        //     ESP_LOGE(TAG, "set pwr cfg failed");
        // } else {
        //     ESP_LOGI(TAG, "set pwr cfg success");
        // }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void stamp_py32_test_case_5()
{
    stp2_gpio_set_func(STP2_GPIO_NUM_0, STP2_GPIO_FUNC_OTHER);
    stp2_gpio_set_drv(STP2_GPIO_NUM_0, STP2_GPIO_DRV_PUSH_PULL);  // 设置为推挽输出
    uint16_t color[5] = {0xF800, 0x07E0, 0x001F, 0xFFFF, 0x0000};

    while (1) {
        for (int i = 0; i < 5; i++) {
            stp2_neo_set_cfg(1, &color[i], 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void stamp_py32_test_case_6()
{
    // stp2_pwr_set_cfg(STP2_PWR_CFG_CHG_EN, STP2_PWR_CFG_CHG_EN, NULL);  // 开启
    while (1) {
        stp2_pwr_set_cfg(STP2_PWR_CFG_CHG_EN, STP2_PWR_CFG_CHG_EN, NULL);  // 开启
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        stp2_pwr_set_cfg(STP2_PWR_CFG_CHG_EN, 0, NULL);  // 关闭
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void stamp_py32_test_case_7()
{
    stp2_pwr_set_cfg(STP2_PWR_CFG_CHG_EN, STP2_PWR_CFG_CHG_EN, NULL);  // 开启

    // stp2_gpio_set_func(STP2_GPIO_NUM_1, STP2_GPIO_FUNC_GPIO);
    // stp2_gpio_set_func(STP2_GPIO_NUM_3, STP2_GPIO_FUNC_GPIO);

    while (1) {
        // // 200mA EN1 输入（GPIO3）  EN2输入（GPIO1）
        // stp2_gpio_set_mode(STP2_GPIO_NUM_1, STP2_GPIO_MODE_INPUT);
        // stp2_gpio_set_mode(STP2_GPIO_NUM_3, STP2_GPIO_MODE_INPUT);

        // vTaskDelay(pdMS_TO_TICKS(DELAY_MS));

        // // 700mA EN1 输入（GPIO3）  EN2输出（GPIO1）
        // stp2_gpio_set_mode(STP2_GPIO_NUM_1, STP2_GPIO_MODE_OUTPUT);
        // stp2_gpio_set_mode(STP2_GPIO_NUM_3, STP2_GPIO_MODE_INPUT);
        // vTaskDelay(10000 / portTICK_PERIOD_MS);

        // 950mA EN1 输出（GPIO3）  EN2输出（GPIO1）
        // stp2_gpio_set_mode(STP2_GPIO_NUM_1, STP2_GPIO_MODE_OUTPUT);
        // stp2_gpio_set_mode(STP2_GPIO_NUM_3, STP2_GPIO_MODE_OUTPUT);
        // vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void stamp_py32_test_case_8()
{
    stp2_gpio_in_state_t in;

    while (1) {
        stp2_pwr_set_cfg(STP2_PWR_CFG_CHG_EN, STP2_PWR_CFG_CHG_EN, NULL);  // 开启
        vTaskDelay(10 / portTICK_PERIOD_MS);
        stp2_gpio_get_in_state(STP2_GPIO_NUM_2, &in);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        stp2_pwr_set_cfg(STP2_PWR_CFG_CHG_EN, 0, NULL);  // 关闭
        vTaskDelay(10 / portTICK_PERIOD_MS);
        stp2_gpio_get_in_state(STP2_GPIO_NUM_2, &in);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void stamp_py32_test_case_9()
{
    uint8_t hw_version;
    uint8_t sw_version;
    uint16_t chip_id;

    esp_err_t ret = ESP_OK;

    if (ret == ESP_OK) {
        ret = stp2_gpio_set_mode(STP2_GPIO_NUM_4, STP2_GPIO_MODE_INPUT);
        ret |= stp2_gpio_set_pupd(STP2_GPIO_NUM_4, STP2_GPIO_PUPD_PULLUP);
        ret |= stp2_gpio_set_wake_en(STP2_GPIO_NUM_4, STP2_GPIO_WAKE_ENABLE);
        ret |= stp2_gpio_set_wake_cfg(STP2_GPIO_NUM_4, STP2_GPIO_WAKE_FALLING);  // 下降沿唤醒

        ESP_LOGI(TAG, "GPIO4 configured for wake (fall edge)");
    } else {
        ESP_LOGE(TAG, "Failed to configure GPIO4 for wake: %s", esp_err_to_name(ret));
    }

    if (stp2_pwr_set_cfg(STP2_PWR_CFG_5V_INOUT, STP2_PWR_CFG_5V_INOUT, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "set pwr cfg failed");
    } else {
        ESP_LOGI(TAG, "set pwr cfg success");
    }

    while (1) {
        stp2_get_hw_version(&hw_version);
        stp2_get_sw_version(&sw_version);
        stp2_get_chip_id(&chip_id);

        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

static void print_auth_mode(int authmode)
{
    switch (authmode) {
        case WIFI_AUTH_OPEN:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OPEN");
            break;
        case WIFI_AUTH_OWE:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OWE");
            break;
        case WIFI_AUTH_WEP:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WEP");
            break;
        case WIFI_AUTH_WPA_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_PSK");
            break;
        case WIFI_AUTH_WPA2_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_PSK");
            break;
        case WIFI_AUTH_WPA_WPA2_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_WPA2_PSK");
            break;
        case WIFI_AUTH_ENTERPRISE:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_ENTERPRISE");
            break;
        case WIFI_AUTH_WPA3_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_PSK");
            break;
        case WIFI_AUTH_WPA2_WPA3_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_WPA3_PSK");
            break;
        case WIFI_AUTH_WPA3_ENT_192:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_ENT_192");
            break;
        default:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_UNKNOWN");
            break;
    }
}

static void print_cipher_type(int pairwise_cipher, int group_cipher)
{
    switch (pairwise_cipher) {
        case WIFI_CIPHER_TYPE_NONE:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_NONE");
            break;
        case WIFI_CIPHER_TYPE_WEP40:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP40");
            break;
        case WIFI_CIPHER_TYPE_WEP104:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP104");
            break;
        case WIFI_CIPHER_TYPE_TKIP:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP");
            break;
        case WIFI_CIPHER_TYPE_CCMP:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_CCMP");
            break;
        case WIFI_CIPHER_TYPE_TKIP_CCMP:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
            break;
        case WIFI_CIPHER_TYPE_AES_CMAC128:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_AES_CMAC128");
            break;
        case WIFI_CIPHER_TYPE_SMS4:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_SMS4");
            break;
        case WIFI_CIPHER_TYPE_GCMP:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_GCMP");
            break;
        case WIFI_CIPHER_TYPE_GCMP256:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_GCMP256");
            break;
        default:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
            break;
    }

    switch (group_cipher) {
        case WIFI_CIPHER_TYPE_NONE:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_NONE");
            break;
        case WIFI_CIPHER_TYPE_WEP40:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP40");
            break;
        case WIFI_CIPHER_TYPE_WEP104:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP104");
            break;
        case WIFI_CIPHER_TYPE_TKIP:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP");
            break;
        case WIFI_CIPHER_TYPE_CCMP:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_CCMP");
            break;
        case WIFI_CIPHER_TYPE_TKIP_CCMP:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
            break;
        case WIFI_CIPHER_TYPE_SMS4:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_SMS4");
            break;
        case WIFI_CIPHER_TYPE_GCMP:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_GCMP");
            break;
        case WIFI_CIPHER_TYPE_GCMP256:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_GCMP256");
            break;
        default:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
            break;
    }
}

/* Initialize Wi-Fi as sta and set scan method */
void wifi_scan_init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void wifi_scan(void)
{
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    esp_wifi_scan_start(NULL, true);
    ESP_LOGI(TAG, "Max AP number ap_info can hold = %u", number);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_LOGI(TAG,
             "=============================================================="
             "=========");
    ESP_LOGI(TAG, "Total APs scanned = %u, actual AP number ap_info holds = %u", ap_count, number);
    ESP_LOGI(TAG, "The best is %s(%d)", ap_info[0].ssid, ap_info[0].rssi);
    for (int i = 0; i < number; i++) {
        ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
        ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
        print_auth_mode(ap_info[i].authmode);
        if (ap_info[i].authmode != WIFI_AUTH_WEP) {
            print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
        }
        ESP_LOGI(TAG, "Channel \t\t%d\n", ap_info[i].primary);
    }
    ESP_LOGI(TAG,
             "=============================================================="
             "=========");
}

void stamp_py32_test_case_10()
{
    wifi_scan_init();
    wifi_scan();
}

void stamp_py32_test_case_11()
{
    // 所有变量声明放在函数开始处
    // const char* TAG = "GPIO_CONTROL";
    int DELAY_MS          = 4000;
    int current_pin_index = 0;
    int GPIO_COUNT        = 28;  // 直接填充数组数量
    int current_state     = 0;   // 0 = LOW, 1 = HIGH
    int i;
    gpio_config_t io_conf = {};

    // 定义引脚数组，按照您指定的顺序
    const gpio_num_t gpio_pins[] = {GPIO_NUM_1,  GPIO_NUM_2,  GPIO_NUM_3,  GPIO_NUM_4,  GPIO_NUM_5,  GPIO_NUM_6,
                                    GPIO_NUM_7,  GPIO_NUM_8,  GPIO_NUM_9,  GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_46,
                                    GPIO_NUM_45, GPIO_NUM_41, GPIO_NUM_43, GPIO_NUM_42, GPIO_NUM_39, GPIO_NUM_17,
                                    GPIO_NUM_44, GPIO_NUM_21, GPIO_NUM_16, GPIO_NUM_18, GPIO_NUM_14, GPIO_NUM_15,
                                    GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_40, GPIO_NUM_38};

    ESP_LOGI(TAG, "ESP32-S3 GPIO Control Test Case 11 Starting...");

    // 打印引脚列表
    ESP_LOGI(TAG, "GPIO pins to control:");
    for (i = 0; i < GPIO_COUNT; i++) {
        printf("GPIO%d ", (int)gpio_pins[i]);
        if ((i + 1) % 10 == 0) printf("\n");  // 每10个换行
    }
    printf("\n");

    // 初始化所有GPIO引脚
    for (i = 0; i < GPIO_COUNT; i++) {
        io_conf.intr_type    = GPIO_INTR_DISABLE;       // 禁用中断
        io_conf.mode         = GPIO_MODE_OUTPUT;        // 输出模式
        io_conf.pin_bit_mask = (1ULL << gpio_pins[i]);  // 设置引脚掩码
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;   // 禁用下拉
        io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;     // 禁用上拉

        gpio_config(&io_conf);

        // 初始化所有引脚为低电平
        gpio_set_level(gpio_pins[i], 0);

        ESP_LOGI(TAG, "GPIO%d initialized as output, set to LOW", (int)gpio_pins[i]);
    }

    ESP_LOGI(TAG, "Total %d GPIO pins initialized", GPIO_COUNT);

    // GPIO控制主循环
    ESP_LOGI(TAG, "Starting GPIO control loop");
    ESP_LOGI(TAG, "Cycle period: %d seconds", DELAY_MS / 1000);

    // gpio_set_level(gpio_pins[1], 1);
    // current_pin_index = 9;
    while(1){
        for (uint8_t i = 0; i < GPIO_COUNT; i++) {
            gpio_set_level(gpio_pins[i], 1);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        for (uint8_t i = 0; i < GPIO_COUNT; i++) {
            gpio_set_level(gpio_pins[i], 0);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    // while (1) {
    //     // 如果当前状态是LOW，切换到HIGH
    //     if (current_state == 0) {
    //         gpio_set_level(gpio_pins[current_pin_index], 1);
    //         current_state = 1;
    //         ESP_LOGI(TAG, "[%d/%d] GPIO%d -> HIGH",
    //                  current_pin_index + 1, GPIO_COUNT, (int)gpio_pins[current_pin_index]);
    //     }
    //     // 如果当前状态是HIGH，切换到LOW并移动到下一个引脚
    //     else {
    //         gpio_set_level(gpio_pins[current_pin_index], 0);
    //         ESP_LOGI(TAG, "[%d/%d] GPIO%d -> LOW",
    //                  current_pin_index + 1, GPIO_COUNT, (int)gpio_pins[current_pin_index]);

    //         // 移动到下一个引脚
    //         current_pin_index++;
    //         if (current_pin_index >= GPIO_COUNT) {
    //             current_pin_index = 0;
    //             ESP_LOGI(TAG, "=== Completed one full cycle, restarting ===");
    //         }

    //         current_state = 0;
    //     }

    //     // 延时3秒
    //     vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    // }
}