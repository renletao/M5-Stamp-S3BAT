/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __STAMP_CASE_H_
#define __STAMP_CASE_H_

#ifdef __cplusplus

extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

void stamp_py32_test_case_1(void);  // VBAT_ADC采集测试

void stamp_py32_test_case_2(void);  // VIN_ADC采集测试

void stamp_py32_test_case_3(void);  // VIN_DET检测测试

void stamp_py32_test_case_4(void);  //  boost输出测试

void stamp_py32_test_case_5(void);  // WS2812测试

void stamp_py32_test_case_6(void);  // 充电测试

void stamp_py32_test_case_7(void);  // 调整充电电流

void stamp_py32_test_case_8(void);  // 读取充电状态

void stamp_py32_test_case_9(void);  // 按键唤醒PY32

void stamp_py32_test_case_10(void);  // wifi测试

void stamp_py32_test_case_11(void);  // IO测试


#ifdef __cplusplus
}
#endif

#endif /* __STAMP_CASE_H */