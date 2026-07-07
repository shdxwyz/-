#include "xunji.h"


// ==================== 限幅函数 ====================

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

int32 xunji_get_line_error_simple(const uint16 adc_value[])
{
    float left_sum = 0.0f;
    float right_sum = 0.0f;

    // 11 路巡线传感器（A2~A8, A10~A13）
    // 以 A7（索引 5）为中心，左右对称加权
    // 左侧：idx 0~4，权重从边缘向中心递减 5,4,3,2,1
    // 右侧：idx 6~10，权重从边缘向中心递减 1,2,3,4,5
    // A7（idx 5）是中心，不参与左右加权计算

    left_sum = (float)adc_value[0] * 2.0f +
               (float)adc_value[1] * 2.0f +
               (float)adc_value[2] * 1.0f +
               (float)adc_value[3] * 1.0f +
               (float)adc_value[4] * 1.0f;

    right_sum = (float)adc_value[6] * 1.0f +
                (float)adc_value[7] * 1.0f +
                (float)adc_value[8] * 1.0f +
                (float)adc_value[9] * 2.0f +
                (float)adc_value[10] * 2.0f;

    // left_sum - right_sum > 0：黑线偏左，需要右转
    // left_sum - right_sum < 0：黑线偏右，需要左转
    return (int32)(left_sum - right_sum);
}

static uint8 xunji_all_adc_over_stop_threshold(const uint16 adc_value[])
{
    uint8 i;

    for(i = 0; i < XUNJI_SENSOR_NUM; i++)
    {
        if(adc_value[i] <= XUNJI_STOP_ADC_THRESHOLD)
        {
            return 0;
        }
    }

    return 1;
}


// ==================== 巡线更新 ====================

void xunji_update(const uint16 adc_value[],
                  float base_target_count,
                  xunji_result_struct *result)
{
    if(xunji_all_adc_over_stop_threshold(adc_value))
    {
        result->line_error = 0;
        result->turn_count = 0;
        result->left_target_count = 0;
        result->right_target_count = 0;
        return;
    }

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

    // line_error > 0：黑线偏左，左轮加速、右轮减速 → 右转
    // line_error < 0：黑线偏右，右轮加速、左轮减速 → 左转
    result->left_target_count = base_target_count + result->turn_count;
    result->right_target_count = base_target_count - result->turn_count;

    result->left_target_count = xunji_limit_float(result->left_target_count,
                                                  XUNJI_MIN_TARGET_COUNT,
                                                  base_target_count + XUNJI_LINE_TURN_LIMIT);

    result->right_target_count = xunji_limit_float(result->right_target_count,
                                                   XUNJI_MIN_TARGET_COUNT,
                                                   base_target_count + XUNJI_LINE_TURN_LIMIT);
}
