#include "xunji.h"


// ==================== 内部限幅函数 ====================

// 将 value 限制在 [min, max] 范围内。
static float xunji_limit_float(float value, float min, float max)
{
    if(value > max)
    {
        return max;
    }
    else if(value < min)
    {
        return min;
    }
    else
    {
        return value;
    }
}


// ==================== 巡线偏差计算 ====================

// 参数是完整的 15 路 adc_value，本函数直接使用真实数组下标。
int32 xunji_get_line_error_simple(const uint16 adc_value[])
{
    float left_sum = 0.0f;
    float right_sum = 0.0f;
    float line_error = 0.0f;

    // 左侧为 [2]~[6]，中间为 [7]，右侧为 [8]~[12]。
    // 当前左右五路的权重均为 1。
    left_sum = (float)adc_value[2] * 3.0f +
               (float)adc_value[3] * 2.0f +
               (float)adc_value[4] * 1.5f +
               (float)adc_value[5] * 1.0f +
               (float)adc_value[6] * 1.0f;

    right_sum = (float)adc_value[8] * 1.0f +
                (float)adc_value[9] * 1.0f +
                (float)adc_value[10] * 1.5f +
                (float)adc_value[11] * 2.0f +
                (float)adc_value[12] * 3.0f;

    // 正值表示左侧信号更强，负值表示右侧信号更强。
    line_error = left_sum - right_sum;

    // 中间光电管大于 2000 时，在死区和 KP 计算前先将差值翻倍。
    if(adc_value[XUNJI_CENTER_SENSOR_IDX] > XUNJI_CENTER_ADC_THRESHOLD)
    {
        line_error *= XUNJI_CENTER_ERROR_GAIN;
    }

    // line_error > 0：黑线偏左，需要右转。
    // line_error < 0：黑线偏右，需要左转。
    return (int32)line_error;
}

// 检查 11 路巡线 ADC 是否全部超过停止阈值。
static uint8 xunji_all_adc_over_stop_threshold(const uint16 adc_value[])
{
    uint8 i;

    // 只检查巡线区间 [2]~[12]，不使用两侧特殊触发通道。
    for(i = XUNJI_LINE_START_IDX; i <= XUNJI_LINE_END_IDX; i++)
    {
        if(adc_value[i] <= XUNJI_STOP_ADC_THRESHOLD)
        {
            return 0;
        }
    }

    return 1;
}


// ==================== 巡线目标更新 ====================

void xunji_update(const uint16 adc_value[],
                  float base_target_count,
                  xunji_result_struct *result)
{
    // 当 11 路巡线 ADC 全部超过停止阈值时，两轮目标清零。
    if(xunji_all_adc_over_stop_threshold(adc_value))
    {
        result->line_error = 0;
        result->turn_count = 0;
        result->left_target_count = 0;
        result->right_target_count = 0;
        return;
    }

    // 计算左右差值，再经过死区和比例系数换算为 turn_count。
    result->line_error = xunji_get_line_error_simple(adc_value);

    // 死区内不做差速，减少 ADC 小波动导致的左右抖动。
    if(result->line_error > -XUNJI_LINE_DEAD_ZONE && result->line_error < XUNJI_LINE_DEAD_ZONE)
    {
        result->turn_count = 0;
    }
    else if(result->line_error >= XUNJI_LINE_DEAD_ZONE)
    {
        result->turn_count = (float)(result->line_error - XUNJI_LINE_DEAD_ZONE) * XUNJI_LINE_TURN_KP;
    }
    else
    {
        result->turn_count = (float)(result->line_error + XUNJI_LINE_DEAD_ZONE) * XUNJI_LINE_TURN_KP;
    }

    // 限制最大巡线修正量，防止单次异常信号导致过激转向。
    result->turn_count = xunji_limit_float(result->turn_count,
                                           -XUNJI_LINE_TURN_LIMIT,
                                           XUNJI_LINE_TURN_LIMIT);

    // line_error > 0：左轮加速、右轮减速，小车向右修正。
    // line_error < 0：右轮加速、左轮减速，小车向左修正。
    result->left_target_count = base_target_count + result->turn_count;
    result->right_target_count = base_target_count - result->turn_count;

    // 目标下限防止低速不稳定，上限是基础目标加最大修正量。
    result->left_target_count = xunji_limit_float(result->left_target_count,
                                                  XUNJI_MIN_TARGET_COUNT,
                                                  base_target_count + XUNJI_LINE_TURN_LIMIT);

    result->right_target_count = xunji_limit_float(result->right_target_count,
                                                   XUNJI_MIN_TARGET_COUNT,
                                                   base_target_count + XUNJI_LINE_TURN_LIMIT);
}
