#include "xunji.h"


// ==================== 闄愬箙鍑芥暟 ====================

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


// ==================== 宸＄嚎鍋忓樊璁＄畻 ====================

int32 xunji_get_line_error_simple(const uint16 adc_value[])
{
    float left_sum = 0.0f;
    float right_sum = 0.0f;

    // 11 璺贰绾夸紶鎰熷櫒锛圓2~A8, A10~A13锛�
    // 浠� A7锛堢储寮� 5锛変负涓績锛屽乏鍙冲绉板姞鏉�
    // 宸︿晶锛歩dx 0~4锛屾潈閲嶄粠杈圭紭鍚戜腑蹇冮�掑噺 5,4,3,2,1
    // 鍙充晶锛歩dx 6~10锛屾潈閲嶄粠杈圭紭鍚戜腑蹇冮�掑噺 1,2,3,4,5
    // A7锛坕dx 5锛夋槸涓績锛屼笉鍙備笌宸﹀彸鍔犳潈璁＄畻

    left_sum = (float)adc_value[0] * 3.0f +
               (float)adc_value[1] * 3.0f +
               (float)adc_value[2] * 2.0f +
               (float)adc_value[3] * 1.5f +
               (float)adc_value[4] * 1.0f;

    right_sum = (float)adc_value[6] * 1.0f +
                (float)adc_value[7] * 1.5f +
                (float)adc_value[8] * 2.0f +
                (float)adc_value[9] * 3.0f +
                (float)adc_value[10] * 3.0f;

    // left_sum - right_sum > 0锛氶粦绾垮亸宸︼紝闇�瑕佸彸杞�
    // left_sum - right_sum < 0锛氶粦绾垮亸鍙筹紝闇�瑕佸乏杞�
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


// ==================== 宸＄嚎鏇存柊 ====================

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

    // line_error > 0锛氶粦绾垮亸宸︼紝宸﹁疆鍔犻�熴�佸彸杞噺閫� 鈫� 鍙宠浆
    // line_error < 0锛氶粦绾垮亸鍙筹紝鍙宠疆鍔犻�熴�佸乏杞噺閫� 鈫� 宸﹁浆
    result->left_target_count = base_target_count + result->turn_count;
    result->right_target_count = base_target_count - result->turn_count;

    result->left_target_count = xunji_limit_float(result->left_target_count,
                                                  XUNJI_MIN_TARGET_COUNT,
                                                  base_target_count + XUNJI_LINE_TURN_LIMIT);

    result->right_target_count = xunji_limit_float(result->right_target_count,
                                                   XUNJI_MIN_TARGET_COUNT,
                                                   base_target_count + XUNJI_LINE_TURN_LIMIT);
}
