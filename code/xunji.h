#ifndef CODE_XUNJI_H_
#define CODE_XUNJI_H_

#include "zf_common_headfile.h"

// 巡线模块：根据 15 路 ADC 中的巡线区域，计算左右轮目标编码器计数。
// 本模块不直接输出 PWM，PWM 由速度 PID 根据这里的目标值产生。

// ==================== 巡线参数 ====================

// 完整 adc_value 共 15 项，数组下标为 [0]~[14]。
// 实际 ADC 引脚依次为 A0~A8、A10~A13、A16、A17。
#define XUNJI_SENSOR_TOTAL             (15)

// 巡线使用 adc_value[2]~adc_value[12]，共 11 路。
#define XUNJI_SENSOR_NUM               (11)

// 巡线传感器在完整 adc_value 数组中的首尾下标。
#define XUNJI_LINE_START_IDX           (2)
#define XUNJI_LINE_END_IDX             (12)

// 中间光电管为 adc_value[7]；严格大于 2000 时将左右差值放大 2 倍。
#define XUNJI_CENTER_SENSOR_IDX        (7)
#define XUNJI_CENTER_ADC_THRESHOLD     (2000)
#define XUNJI_CENTER_ERROR_GAIN        (2.0f)

// line_error 的死区：差值绝对值小于 500 时不产生左右差速。
#define XUNJI_LINE_DEAD_ZONE            (500)

// 死区外 ADC 差值到单轮目标修正量的比例。
// turn_count = (去掉死区后的 line_error) * XUNJI_LINE_TURN_KP。
// PID 周期由 20ms 缩短为 5ms，目标计数按 1/4 缩放以保持原差速。
#define XUNJI_LINE_TURN_KP              (0.008f)

// 左右轮目标计数的最大修正量，单位：count/5ms。
#define XUNJI_LINE_TURN_LIMIT           (2000.0f)

// 防止某一侧目标速度过低导致电机不稳定。
#define XUNJI_MIN_TARGET_COUNT          (50.0f)

// 所有巡线 ADC 同时大于该阈值时，左右目标设为 0。
// 注意：12 位 ADC 最大只有 4095，当前 18000 会使这个停止条件无法成立。
#define XUNJI_STOP_ADC_THRESHOLD        (18000)


// ==================== 巡线结果 ====================

typedef struct
{
    int32 line_error;           // 左侧 ADC 和 - 右侧 ADC 和，正值表示黑线偏左。
    float turn_count;           // 单轮目标修正量，单位 count/PID 周期。
    float left_target_count;    // 左轮巡线目标计数。
    float right_target_count;   // 右轮巡线目标计数。
} xunji_result_struct;


// ==================== 对外接口 ====================

// 根据完整 adc_value 计算左右 ADC 差值。
int32 xunji_get_line_error_simple(const uint16 adc_value[]);

// 根据 ADC 差值、基础目标和巡线参数，生成左右轮目标。
void xunji_update(const uint16 adc_value[],
                  float base_target_count,
                  xunji_result_struct *result);

#endif
