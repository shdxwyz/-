#include "yqj.h"

// system_getval() 的计时单位为 10ns，1ms 对应 100000 个计数。
#define YQJ_MS_TO_10NS(ms) ((uint32)((ms) * 100000UL))

// ==================== 顺序状态机 ====================

uint16 yqj_flag = 1;                         // 上电后从 case 1 开始。
yqj_state_enum yqj_state = YQJ_STATE_LINE;   // 初始在普通巡线等待状态。
uint8 yqj_action_trigger = 0;                // 初始不覆盖巡线目标。
uint32 yqj_state_start_time = 0;             // 当前状态的起始计时值。
int32 yqj_delay_start_count = 0;             // DELAY 开始时的左右总计数。
int32 yqj_lock_start_count = 0;              // LOCK 开始时的左右总计数。

// yqj_init() 会用主程序的实际参数覆盖下面的默认值。
static float yqj_pid_period_s = 0.02f;               // 速度 PID 周期，单位 s。
static float yqj_encoder_count_per_meter = 12106.0f; // 单轮行驶 1m 的编码器计数。

// ==================== 传感器索引说明 ====================
// 数组索引：       0  1  2  3  4  5  6  7  8   9  10  11  12  13  14
// 实际 ADC 引脚：  A0 A1 A2 A3 A4 A5 A6 A7 A8 A10 A11 A12 A13 A16 A17
// A0(0), A1(1) 用于左转弯检测
// A16(13), A17(14) 用于右转弯检测
// 数组 [2]~[12]（引脚 A2~A8、A10~A13）用于巡线和元器件识别

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
    return (adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 电源：A3~A8 同时检测到白线
uint8 yqj_dianyuan_trigger(const uint16 adc_value[])
{
    return (adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 线圈电阻：A8 和 A10 同时检测到白线
uint8 yqj_xianquandianzu_trigger(const uint16 adc_value[])
{
    return (adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
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
    return (adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 三极管 1_2：A1 和 A2 同时检测到白线
uint8 yqj_sanjiguan1_2trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 三极管 2_1：A8 和 A10 同时检测到白线
uint8 yqj_sanjiguan2_1trigger(const uint16 adc_value[])
{
    return (adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 三极管 2_0：A8 和 A10 同时检测到白线
uint8 yqj_sanjiguan2_0trigger(const uint16 adc_value[])
{
    return (adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 三极管 0_1：数组 [2]~[9]（A2~A8 和 A10）同时检测到白线
uint8 yqj_sanjiguan0_1trigger(const uint16 adc_value[])
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

// 三极管 1_0：A1 和 A2 同时检测到白线
uint8 yqj_sanjiguan1_0trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 三极管 0_2：数组 [2]~[9]（A2~A8 和 A10）同时检测到白线
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

// 线圈：数组 [2]~[9]（A2~A8 和 A10）同时检测到白线
uint8 yqj_xianquan_trigger(const uint16 adc_value[])
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

// 二极管：A0 和 A16 同时检测到白线
uint8 yqj_erji_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[13] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 电池：A0 和 A16 同时检测到白线
uint8 yqj_dianchi_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[13] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 非门：A0 和 A16 同时检测到白线
uint8 yqj_feimen_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[13] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 电容：A0 和 A16 同时检测到白线
uint8 yqj_dianrong_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[13] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// ==================== 内部工具函数 ====================

// 将速度 m/s 换算成一个 PID 周期内的编码器目标计数。
static float yqj_speed_to_target_count(float speed_mps)
{
    // count/周期 = m/s * s/周期 * count/m。
    return speed_mps * yqj_pid_period_s * yqj_encoder_count_per_meter;
}

// 将距离 m 换算成左右轮里程和需要增加的编码器计数。
static int32 yqj_meter_to_count(float distance_m)
{
    // 延迟距离和自锁距离都定义为左右轮距离之和，因此这里不再额外乘 2。
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

// 判断从指定起点开始，左右轮累计距离之和是否已经达到要求。
static uint8 yqj_distance_reached(int32 encoder_total_sum,
                                  int32 start_count,
                                  float distance_m)
{
    int32 need_count = yqj_meter_to_count(distance_m);

    if (need_count <= 0)
    {
        return 1;
    }

    return ((encoder_total_sum - start_count) >= need_count);
}

// ==================== 对外工具函数 ====================

// 初始化元器件顺序框架，把 flag、状态、动作都复位到起点。
void yqj_init(float pid_period_s, float encoder_count_per_meter)
{
    // 保存主程序的实际标定值，确保 m/s、距离和编码器计数换算一致。
    yqj_pid_period_s = pid_period_s;
    yqj_encoder_count_per_meter = encoder_count_per_meter;
    yqj_flag = 1;
    yqj_state = YQJ_STATE_LINE;
    yqj_action_trigger = 0;
    yqj_state_start_time = 0;
    yqj_delay_start_count = 0;
    yqj_lock_start_count = 0;

    // 启动 system_getval() 使用的高精度计时器。
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

// 当前 case 的条件成立后，记录延迟起点并进入触发延时。
void yqj_start_case(uint8 action_trigger, int32 encoder_total_sum)
{
    yqj_action_trigger = action_trigger;
    yqj_delay_start_count = encoder_total_sum;
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
}

// 判断动作前延迟是否结束；时间和距离两个条件都满足才进入 RUN。
uint8 yqj_delay_done(int32 encoder_total_sum,
                     uint32 delay_ms,
                     float delay_distance_m)
{
    return (yqj_time_reached(yqj_state_start_time, delay_ms) &&
            yqj_distance_reached(encoder_total_sum,
                                 yqj_delay_start_count,
                                 delay_distance_m));
}

// 判断当前 case 的自锁是否结束；时间和距离两个条件都满足才解锁。
uint8 yqj_lock_done(int32 encoder_total_sum, uint32 lock_ms, float lock_distance_m)
{
    // 时间和距离两项必须同时达标才解锁。
    return (yqj_time_reached(yqj_state_start_time, lock_ms) &&
            yqj_distance_reached(encoder_total_sum,
                                 yqj_lock_start_count,
                                 lock_distance_m));
}

// 将当前 case 的左右轮目标速度换算成速度 PID 使用的目标计数。
void yqj_apply_action(float left_speed_mps,
                      float right_speed_mps,
                      float *left_target_count,
                      float *right_target_count)
{
    if (yqj_action_trigger)
    {
        *left_target_count = yqj_speed_to_target_count(left_speed_mps);
        *right_target_count = yqj_speed_to_target_count(right_speed_mps);
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
