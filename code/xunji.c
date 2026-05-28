#include "xunji.h"


// ==================== 内部工具函数 ====================

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


// ==================== 不加权巡线误差 ====================

int32 xunji_get_line_error_simple(const uint16 adc_value[])
{
    int32 left_sum = 0;
    int32 right_sum = 0;

    // 左 5 路：A1 A2 A3 A4 A5
    for(uint8 i = 0; i < 5; i++)
    {
        left_sum += adc_value[i];
    }

    // 右 5 路：A6 A7 A8 A10 A11
    for(uint8 i = 5; i < XUNJI_SENSOR_NUM; i++)
    {
        right_sum += adc_value[i];
    }

    // 白线 ADC 小
    // left_sum - right_sum > 0：右边更白，线偏右
    // left_sum - right_sum < 0：左边更白，线偏左
    return left_sum - right_sum;
}


// ==================== 巡线目标计算 ====================

void xunji_update(const uint16 adc_value[],
                  float base_target_count,
                  xunji_result_struct *result)
{
    result->line_error = xunji_get_line_error_simple(adc_value);

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

    result->turn_count = xunji_limit_float(result->turn_count,
                                           -XUNJI_LINE_TURN_LIMIT,
                                           XUNJI_LINE_TURN_LIMIT);

    // 线偏右：line_error > 0，左轮目标增大，右轮目标减小
    // 线偏左：line_error < 0，左轮目标减小，右轮目标增大
    result->left_target_count = base_target_count + result->turn_count;
    result->right_target_count = base_target_count - result->turn_count;

    result->left_target_count = xunji_limit_float(result->left_target_count,
                                                  XUNJI_MIN_TARGET_COUNT,
                                                  base_target_count + XUNJI_LINE_TURN_LIMIT);

    result->right_target_count = xunji_limit_float(result->right_target_count,
                                                   XUNJI_MIN_TARGET_COUNT,
                                                   base_target_count + XUNJI_LINE_TURN_LIMIT);
}