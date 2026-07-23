#include "yqj.h"
#include "device.h"

#define YQJ_MS_TO_10NS(ms) ((uint32)((ms) * 100000UL))

uint16 yqj_flag = 1;
yqj_state_enum yqj_state = YQJ_STATE_LINE;
uint8 yqj_action_trigger = 0;
uint32 yqj_state_start_time = 0;
int32 yqj_lock_start_count = 0;

static float yqj_encoder_count_per_meter = 12106.0f;
static int32 yqj_delay_start_count = 0;

// ==================== 传感器索引说明 ====================
// 15 路传感器：A0 A1 A2 A3 A4 A5 A6 A7 A8 A10 A11 A12 A13 A16 A17
// 索引：        0  1  2  3  4  5  6  7  8  9  10  11  12  13  14
// A0(0), A1(1) 用于左转弯检测
// A16(13), A17(14) 用于右转弯检测
// A2~A13(2~12) 用于巡线和元器件识别

// 左转触发：A0 或 A1 检测到白线
uint8 yqj_left_turn_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE ||
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 右转触发：A16 或 A17 检测到白线
uint8 yqj_right_turn_trigger(const uint16 adc_value[])
{
    return (adc_value[13] < YQJ_TURN_TRIGGER_ADC_VALUE ||
            adc_value[14] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 电阻：A2 和 A10 同时检测到白线
uint8 yqj_dianzu_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[13] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 电源：A3~A8 同时检测到白线
uint8 yqj_dianyuan_trigger(const uint16 adc_value[])
{
    return (adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[10] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 线圈电阻：A8 和 A10 同时检测到白线
uint8 yqj_xianquandianzu0_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 开关 1_0：A8 和 A10 同时检测到白线
uint8 yqj_kaiguang1_0trigger(const uint16 adc_value[])
{
    return (adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 开关 0_1：A2 和 A3 同时检测到白线
uint8 yqj_kaiguang0_1trigger(const uint16 adc_value[])
{
    return (adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 开关：A5~A8 同时检测到白线
uint8 yqj_kaiguang_trigger(const uint16 adc_value[])
{
    return (adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 二极管：A2 和 A10 同时检测到白线
uint8 yqj_erjiguan_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[13] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 特别二极管：A14检测到白线
uint8 yqj_tberjiguan_trigger(const uint16 adc_value[])
{   return (adc_value[14] < YQJ_TURN_TRIGGER_ADC_VALUE);}

// 三极管 1_2：A1 和 A2 同时检测到白线
uint8 yqj_sanjiguan1_2trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 三极管 2_1：A8 和 A10 同时检测到白线
uint8 yqj_sanjiguan2_1trigger(const uint16 adc_value[])
{
    return (adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}


uint8 yqj_sanjiguan2_0trigger(const uint16 adc_value[])
{
    return (adc_value[14] < YQJ_TURN_TRIGGER_ADC_VALUE ||
            adc_value[13] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 三极管 0_1：A2~A8 同时检测到白线
uint8 yqj_sanjiguan0_1trigger(const uint16 adc_value[])
{
    return (adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 三极管 1_0：A1 和 A2 同时检测到白线
uint8 yqj_sanjiguan1_0trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 三极管 0_2：A2~A8 同时检测到白线
uint8 yqj_sanjiguan0_2trigger(const uint16 adc_value[])
{
    return (adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 双侧触发：A0 和 A16 同时检测到白线
uint8 yqj_double_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[13] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 左电感：A0 或 A1 检测到白线
uint8 yqj_ldiangan_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE ||
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 右电感：A16 或 A17 检测到白线
uint8 yqj_rdiangan_trigger(const uint16 adc_value[])
{
    return (adc_value[13] < YQJ_TURN_TRIGGER_ADC_VALUE ||
            adc_value[14] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 线圈：A2~A8 同时检测到白线
uint8 yqj_xianquan_trigger(const uint16 adc_value[])
{
    return (adc_value[10] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[11] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 二极管：A0 和 A16 同时检测到白线
uint8 yqj_erji_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[13] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 电池：A0 和 A16 同时检测到白线
uint8 yqj_dianchi_trigger(const uint16 adc_value[])
{
    return (adc_value[10] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[11] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 非门：A0 和 A16 同时检测到白线
uint8 yqj_feimen_trigger(const uint16 adc_value[])
{
    return (adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE&&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE );
}
uint8 yqj_feimen0_1_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[13] < YQJ_TURN_TRIGGER_ADC_VALUE);
}


// 电容：A0 和 A16 同时检测到白线
uint8 yqj_dianrong_trigger(const uint16 adc_value[])
{
    return (adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[10] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// ==================== 内部工具函数 ====================

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

// RUN 距离按单轮/车身前进距离配置，编码器判断使用左右轮总计数。
static uint8 yqj_run_distance_reached(int32 encoder_total_sum,
                                      float run_distance_m)
{
    int32 need_count = 2 * yqj_meter_to_count(run_distance_m);

    if (need_count <= 0)
    {
        return 1;
    }

    return ((encoder_total_sum - yqj_lock_start_count) >= need_count);
}

// DELAY 距离按单轮/车身前进距离配置，编码器判断使用左右轮总计数。
static uint8 yqj_delay_distance_reached(int32 encoder_total_sum,
                                        float delay_distance_m)
{
    int32 need_count = 2 * yqj_meter_to_count(delay_distance_m);

    if (need_count <= 0)
    {
        return 1;
    }

    return ((encoder_total_sum - yqj_delay_start_count) >= need_count);
}

// ==================== 对外工具函数 ====================

// 初始化元器件顺序框架，把 flag、状态、动作都复位到起点。
void yqj_init(float encoder_count_per_meter)
{
    yqj_encoder_count_per_meter = encoder_count_per_meter;
    yqj_flag = 1;
    yqj_state = YQJ_STATE_LINE;
    yqj_action_trigger = 0;
    yqj_state_start_time = 0;
    yqj_lock_start_count = 0;
    yqj_delay_start_count = 0;
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

// 当前 case 的条件成立后，记录延迟起点并进入不巡线延迟阶段。
void yqj_start_case(uint8 action_trigger, int32 encoder_total_sum)
{
    yqj_action_trigger = action_trigger;
    yqj_delay_start_count = encoder_total_sum;
    yqj_enter_state(YQJ_STATE_DELAY);
}

// 延时结束后开始不巡线执行，并同时记录时间和左右轮里程起点。
void yqj_start_run(int32 encoder_total_sum)
{
    yqj_lock_start_count = encoder_total_sum;
    yqj_enter_state(YQJ_STATE_RUN);
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
}

// DELAY 的时间和距离从触发时同时开始计算，两项都满足才开始动作。
// 某项为 0 时，该项立即视为满足。
uint8 yqj_delay_done(int32 encoder_total_sum,
                     uint32 delay_ms,
                     float delay_distance_m)
{
    return (yqj_time_reached(yqj_state_start_time, delay_ms) &&
            yqj_delay_distance_reached(encoder_total_sum,
                                       delay_distance_m));
}

// RUN 的时间和距离从同一时刻开始计算，两项都满足才结束执行。
// 某项为 0 时，该项立即视为满足。
uint8 yqj_run_done(int32 encoder_total_sum,
                   uint32 run_ms,
                   float run_distance_m)
{
    return (yqj_time_reached(yqj_state_start_time, run_ms) &&
            yqj_run_distance_reached(encoder_total_sum, run_distance_m));
}

// 判断当前 case 的自锁是否结束；时间和距离两个条件都满足才解锁。
uint8 yqj_lock_done(int32 encoder_total_sum, uint32 lock_ms, float lock_distance_m)
{
    return (yqj_time_reached(yqj_state_start_time, lock_ms) &&
            yqj_lock_distance_reached(encoder_total_sum, lock_distance_m));
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
