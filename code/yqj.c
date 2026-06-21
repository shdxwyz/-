#include "yqj.h"
#include "device.h"

#define YQJ_MS_TO_10NS(ms) ((uint32)((ms) * 100000UL))

uint16 yqj_flag = 1;
yqj_state_enum yqj_state = YQJ_STATE_LINE;
uint8 yqj_action_trigger = 0;
uint32 yqj_state_start_time = 0;
int32 yqj_lock_start_count = 0;

static float yqj_pid_period_s = 0.02f;
static float yqj_encoder_count_per_meter = 12106.0f;

// 判断左转触发条件：左边传感器检测到白线（低于阈值），
uint8 yqj_left_turn_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 判断右转触发条件：右边传感器检测到白线（低于阈值），
uint8 yqj_right_turn_trigger(const uint16 adc_value[])
{
    return (adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 电阻
uint8 yqj_dianzu_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 电源
uint8 yqj_dianyuan_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 线圈电阻
uint8 yqj_xianquandianzu_trigger(const uint16 adc_value[])
{
    return (adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 开关 1_0
uint8 yqj_kaiguang1_0trigger(const uint16 adc_value[])
{
    return (adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 开关 0_1
uint8 yqj_kaiguang0_1trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

uint8 yqj_kaiguang_trigger(const uint16 adc_value[])
{
    return (adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 二极管
uint8 yqj_erjiguan_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 三极管 1_2
uint8 yqj_sanjiguan1_2trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 三极管 2_1
uint8 yqj_sanjiguan2_1trigger(const uint16 adc_value[])
{
    return (adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 三极管 2_0
uint8 yqj_sanjiguan2_0trigger(const uint16 adc_value[])
{
    return (adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}
// 三极管 0_1
uint8 yqj_sanjiguan0_1trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}
// 三极管 1_0
uint8 yqj_sanjiguan1_0trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE);
}
// 三极管 0_2
uint8 yqj_sanjiguan0_2trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}
// 双侧触发
uint8 yqj_double_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 左电感
uint8 yqj_ldiangan_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE||
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 右电感
uint8 yqj_rdiangan_trigger(const uint16 adc_value[])
{
    return (adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE||
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 线圈
uint8 yqj_xianquan_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 二极管
uint8 yqj_erji_trigger(const uint16 adc_value[])
{
    return (adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 电池
uint8 yqj_dianchi_trigger(const uint16 adc_value[])
{
    return (
        adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
        adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
        adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
        adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE);
}
// 非门

uint8 yqj_feimen_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}
// 电容

uint8 yqj_dianrong_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}
// ==================== 内部工具函数 ====================

// 将速度 m/s 换算成一个 PID 周期内的编码器目标计数。
static float yqj_speed_to_target_count(float speed_mps)
{
    return speed_mps * yqj_pid_period_s * yqj_encoder_count_per_meter;
}

// 将自锁距离 m 换算成左右轮里程和需要增加的编码器计数。
static int32 yqj_meter_to_count(float distance_m)
{
    if (distance_m <= 0.0f)
    {
        return 0;
    }

    return (int32)(distance_m * yqj_encoder_count_per_meter);
}

// 切换状态，并记录进入该状态的时间。
static void yqj_enter_state(yqj_state_enum state)
{
    yqj_state = state;
    yqj_state_start_time = system_getval();
}

// 判断动作后的自锁距离是否已经走够。
static uint8 yqj_lock_distance_reached(int32 encoder_total_sum, float lock_distance_m)
{
    int32 need_count = yqj_meter_to_count(lock_distance_m);

    if (need_count <= 0)
    {
        return 1;
    }

    return ((encoder_total_sum - yqj_lock_start_count) >= need_count);
}

// ==================== 对外工具函数 ====================

// 初始化元器件顺序框架，把 flag、状态、动作都复位到起点。
void yqj_init(float pid_period_s, float encoder_count_per_meter)
{
    yqj_pid_period_s = pid_period_s;
    yqj_encoder_count_per_meter = encoder_count_per_meter;
    yqj_flag = 1;
    yqj_state = YQJ_STATE_LINE;
    yqj_action_trigger = 0;
    yqj_state_start_time = 0;
    yqj_lock_start_count = 0;

    system_start();
}

// 判断从 start_time 开始是否已经达到指定毫秒数；duration_ms 为 0 时表示不用等待。
uint8 yqj_time_reached(uint32 start_time, uint32 duration_ms)
{
    if (0 == duration_ms)
    {
        return 1;
    }

    return ((uint32)(system_getval() - start_time) >= YQJ_MS_TO_10NS(duration_ms));
}

// 当前 case 的条件成立后，记录是否执行动作，并进入触发延时。
void yqj_start_case(uint8 action_trigger)
{
    yqj_action_trigger = action_trigger;
    yqj_enter_state(YQJ_STATE_DELAY);
}

// 动作执行完成后，记录左右轮里程和，并进入自锁状态。
void yqj_start_lock(int32 encoder_total_sum)
{
    yqj_lock_start_count = encoder_total_sum;
    yqj_enter_state(YQJ_STATE_LOCK);
}

// 当前 case 完全结束后，flag 加 1，开始等待下一个元器件条件。
void yqj_finish_case(void)
{
    if (yqj_flag < 65535u)
    {
        yqj_flag++;
    }
    yqj_action_trigger = 0;
    yqj_enter_state(YQJ_STATE_LINE);

    // motor_stop();
    // system_delay_ms(20000);
}

// 判断当前 case 的自锁是否结束；时间和距离两个条件都满足才解锁。
uint8 yqj_lock_done(int32 encoder_total_sum, uint32 lock_ms, float lock_distance_m)
{
    return (yqj_time_reached(yqj_state_start_time, lock_ms) &&
            yqj_lock_distance_reached(encoder_total_sum, lock_distance_m));
}

// 根据当前 case 给出的左右轮速度覆盖目标；触发标志为 0 时不覆盖，继续巡线。
// 转弯时在动作速度基础上叠加巡线修正量，让小车边转边巡线。
void yqj_apply_action(float left_speed_mps,
                      float right_speed_mps,
                      float *left_target_count,
                      float *right_target_count)
{
    if (yqj_action_trigger)
    {
        float base_left = yqj_speed_to_target_count(left_speed_mps);
        float base_right = yqj_speed_to_target_count(right_speed_mps);

        // 计算巡线修正量：当前巡线目标与动作基础速度的差值。
        float line_correction_left = *left_target_count - base_left;
        float line_correction_right = *right_target_count - base_right;

        // 叠加修正量，限幅到正负 50%，防止修正过大。
        if (line_correction_left > 0)
        {
            line_correction_left = line_correction_left > base_left * 0.5f ? base_left * 0.5f : line_correction_left;
        }
        else
        {
            line_correction_left = line_correction_left < -base_left * 0.5f ? -base_left * 0.5f : line_correction_left;
        }

        if (line_correction_right > 0)
        {
            line_correction_right = line_correction_right > base_right * 0.5f ? base_right * 0.5f : line_correction_right;
        }
        else
        {
            line_correction_right = line_correction_right < -base_right * 0.5f ? -base_right * 0.5f : line_correction_right;
        }

        *left_target_count = base_left + line_correction_left;
        *right_target_count = base_right + line_correction_right;
    }
}

// 获取当前正在等待或执行的元器件编号，串口调试时看这个值。
uint16 yqj_get_flag(void)
{
    return yqj_flag;
}

// 手动设置当前元器件编号，方便从某一个 case 开始调车。
void yqj_set_flag(uint16 flag)
{
    yqj_flag = flag;
    yqj_action_trigger = 0;
    yqj_enter_state(YQJ_STATE_LINE);
}

// 获取当前状态：巡线、延时、执行动作、自锁。
yqj_state_enum yqj_get_state(void)
{
    return yqj_state;
}

// 获取当前是否正在执行触发动作：0 表示不执行，1 表示执行。
uint8 yqj_get_action_trigger(void)
{
    return yqj_action_trigger;
}
