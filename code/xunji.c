#include "xunji.h"


// ==================== ?????????? ====================

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


// ==================== ????????? ====================

int32 xunji_get_line_error_simple(const uint16 adc_value[])
{
    float left_sum = 0.0f;
    float right_sum = 0.0f;

    left_sum = (float)adc_value[0] * 4.0f +
               (float)adc_value[1] * 3.0f +
               (float)adc_value[2] * 2.0f +
               (float)adc_value[3] * 1.0f +
               (float)adc_value[4] * 0.5f;

    right_sum = (float)adc_value[5] * 0.5f +
                (float)adc_value[6] * 1.0f +
                (float)adc_value[7] * 2.0f +
                (float)adc_value[8] * 3.0f +
                (float)adc_value[9] * 4.0f;

    // ???? ADC §³
    // left_sum - right_sum > 0??????????????
    // left_sum - right_sum < 0??????????????
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


// ==================== ????????? ====================

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

    // ??????line_error > 0?????????????????????§³
    // ?????line_error < 0??????????§³?????????????
    result->left_target_count = base_target_count + result->turn_count;
    result->right_target_count = base_target_count - result->turn_count;

    result->left_target_count = xunji_limit_float(result->left_target_count,
                                                  XUNJI_MIN_TARGET_COUNT,
                                                  base_target_count + XUNJI_LINE_TURN_LIMIT);

    result->right_target_count = xunji_limit_float(result->right_target_count,
                                                   XUNJI_MIN_TARGET_COUNT,
                                                   base_target_count + XUNJI_LINE_TURN_LIMIT);
}