#ifndef CODE_XUNJI_H_
#define CODE_XUNJI_H_

#include "zf_common_headfile.h"

// ==================== 巡线参数 ====================

#define XUNJI_SENSOR_NUM                (10)

// 左右 ADC 总和差值在 ±300 内，认为不用修正
#define XUNJI_LINE_DEAD_ZONE            (300)

// 差值转成目标速度差
// 速度从 0.3 提到 1.0 m/s，转向强度需要大幅加大
#define XUNJI_LINE_TURN_KP              (0.03f)

// 最大左右目标差，单位：20ms 编码器计数
#define XUNJI_LINE_TURN_LIMIT           (35.0f)

// 防止某一边目标速度太低
#define XUNJI_MIN_TARGET_COUNT          (20.0f)


// ==================== 巡线结果 ====================

typedef struct
{
    int32 line_error;
    float turn_count;
    float left_target_count;
    float right_target_count;
} xunji_result_struct;


// ==================== 对外接口 ====================

int32 xunji_get_line_error_simple(const uint16 adc_value[]);

void xunji_update(const uint16 adc_value[],
                  float base_target_count,
                  xunji_result_struct *result);

#endif
