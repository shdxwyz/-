#ifndef CODE_XUNJI_H_
#define CODE_XUNJI_H_

#include "zf_common_headfile.h"

// ==================== 巡线参数 ====================

// 总传感器数量（A0~A8, A10~A13, A16, A17）
#define XUNJI_SENSOR_TOTAL             (15)

// 巡线用的传感器数量（A2~A8, A10~A13，跳过两边的 A0,A1 和 A16,A17）
#define XUNJI_SENSOR_NUM               (11)

// 巡线传感器在 adc_value 数组中的起始索引（跳过 A0, A1）
#define XUNJI_LINE_START_IDX           (2)

// 左右 ADC 总和差值在正负 500 以内时认为居中，不做修正。
#define XUNJI_LINE_DEAD_ZONE            (500)

// ADC 差值转换成左右轮目标编码器计数差的比例。
#define XUNJI_LINE_TURN_KP              (0.05f)

// 左右轮目标计数的最大修正量，单位：count/20ms。
#define XUNJI_LINE_TURN_LIMIT           (800.0f)

// 防止某一侧目标速度过低导致电机不稳定。
#define XUNJI_MIN_TARGET_COUNT          (200.0f)

// 所有巡线 ADC 同时大于该阈值时停止电机。
#define XUNJI_STOP_ADC_THRESHOLD        (8000)


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
