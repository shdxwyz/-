#include "yqj.h"
#include "device.h"
#include <math.h>

#define YQJ_MS_TO_10NS(ms) ((uint32)((ms) * 100000UL))

uint16 yqj_flag = 1;
yqj_state_enum yqj_state = YQJ_STATE_LINE;
uint8 yqj_action_trigger = 0;
uint32 yqj_state_start_time = 0;
int32 yqj_lock_start_count = 0;

static float yqj_pid_period_s = 0.02f;
static float yqj_encoder_count_per_meter = 12106.0f;

// 角度环相关变量
volatile float yqj_integrated_angle = 0.0f; // 三轴角速度模长积分得到的相对转角（度）
volatile float yqj_gyro_rate_dps = 0.0f;    // 三轴角速度合成值（度/秒）
PidTypeDef yqj_angle_pid;                // 角度环 PID
float yqj_angle_pid_output = 0.0f;       // 角度环 PID 输出（速度差，m/s）

// 转弯方向：1=左转，2=右转
static uint8 yqj_turn_direction = 0;

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
            adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE);
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
    return (adc_value[12] < YQJ_TURN_TRIGGER_ADC_VALUE &&
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

// ==================== 角度环相关函数 ====================

// 初始化角度环 PID
void yqj_angle_pid_init(void)
{
    PID_Init(&yqj_angle_pid,
             PID_POSITION,
             ANGLE_PID_MAX_OUT,
             ANGLE_PID_MAX_IOUT,
             ANGLE_KP,
             ANGLE_KI,
             ANGLE_KD);
}

// 重置角度环 PID 和积分角度
void yqj_angle_pid_reset(void)
{
    PID_clear(&yqj_angle_pid);
    yqj_integrated_angle = 0.0f;
    yqj_angle_pid_output = 0.0f;
}

// 角度环 PID 计算：输入目标角度和当前角度，输出速度差（m/s）
float yqj_angle_pid_calc(float target_angle, float current_angle)
{
    yqj_angle_pid_output = PID_Calc(&yqj_angle_pid, current_angle, target_angle);
    return yqj_angle_pid_output;
}

// 在 120Hz IMU 数据就绪中断中调用，每个原始角速度样本只积分一次。
// 平面转弯只关心角速度大小；使用三轴向量模长可避免模块安装方向选错轴。
void yqj_integrate_gyro(float gyro_x_dps,
                        float gyro_y_dps,
                        float gyro_z_dps,
                        float dt_s)
{
    yqj_gyro_rate_dps = sqrtf(gyro_x_dps * gyro_x_dps +
                              gyro_y_dps * gyro_y_dps +
                              gyro_z_dps * gyro_z_dps);
    yqj_integrated_angle += yqj_gyro_rate_dps * dt_s;
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
    yqj_turn_direction = 0;

    // 初始化角度环 PID
    yqj_angle_pid_init();
    yqj_angle_pid_reset();

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
    yqj_turn_direction = 0;  // 转弯结束，清除方向
    yqj_enter_state(YQJ_STATE_LOCK);
}

// 转弯完成后进入不循线直行阶段，并记录起始里程。
void yqj_start_blind(int32 encoder_total_sum)
{
    yqj_lock_start_count = encoder_total_sum;
    yqj_turn_direction = 0;
    yqj_enter_state(YQJ_STATE_BLIND);
}

// 当前 case 完全结束后，flag 加 1，开始等待下一个元器件条件。
void yqj_finish_case(void)
{
    if (yqj_flag < 65535u)
    {
        yqj_flag++;
    }
    yqj_action_trigger = 0;
    yqj_turn_direction = 0;
    yqj_enter_state(YQJ_STATE_LINE);
}

// 判断当前 case 的自锁是否结束；时间和距离两个条件都满足才解锁。
uint8 yqj_lock_done(int32 encoder_total_sum, uint32 lock_ms, float lock_distance_m)
{
    return (yqj_time_reached(yqj_state_start_time, lock_ms) &&
            yqj_lock_distance_reached(encoder_total_sum, lock_distance_m));
}

// 判断不循线直行距离是否已经达到。
uint8 yqj_blind_done(int32 encoder_total_sum, float blind_distance_m)
{
    int32 need_count = 2 * yqj_meter_to_count(blind_distance_m);

    if(need_count <= 0)
    {
        return 1;
    }

    return ((encoder_total_sum - yqj_lock_start_count) >= need_count);
}

// 角速度积分得到的相对转角进入目标容差范围后结束转向。
uint8 yqj_turn_target_reached(void)
{
    float turn_angle_abs;

    if(yqj_turn_direction == 0)
    {
        return 0;
    }

    turn_angle_abs = yqj_integrated_angle;
    if(turn_angle_abs < 0.0f) turn_angle_abs = -turn_angle_abs;

    // 转向方向由电机命令决定；结束条件只判断已经转过的角度大小，
    // 避免 IMU 安装方向导致右转积分符号与预期相反而只能等超时。
    return (turn_angle_abs >=
            (YQJ_TURN_TARGET_ANGLE - YQJ_TURN_ANGLE_TOLERANCE));
}

// 根据当前 case 给出的转弯基础速度，用角度环闭环控制转弯。
// turn_base_speed：符号表示方向（正=左、负=右），绝对值表示向前基础速度
// left_target_count/right_target_count：输出左右轮目标编码器计数
void yqj_apply_action(float turn_base_speed,
                      float *left_target_count,
                      float *right_target_count)
{
    if (yqj_action_trigger)
    {
        float forward_base_speed = (turn_base_speed >= 0.0f) ?
                                   turn_base_speed : -turn_base_speed;
        float current_angle_abs;

        // 第一次进入转弯：根据 turn_base_speed 判断方向
        // 正速度 = 左转（右轮快左轮慢），负速度 = 右转（左轮快右轮慢）
        if (yqj_turn_direction == 0)
        {
            yqj_turn_direction = (turn_base_speed >= 0.0f) ? 1 : 2;  // 1=左转, 2=右转
            yqj_angle_pid_reset();
        }

        current_angle_abs = yqj_integrated_angle;
        if(current_angle_abs < 0.0f) current_angle_abs = -current_angle_abs;

        // 角度环只计算还差多少转角，左右方向由下面的轮速分配决定。
        float speed_diff = yqj_angle_pid_calc(YQJ_TURN_TARGET_ANGLE,
                                              current_angle_abs);
        if(speed_diff < 0.0f) speed_diff = 0.0f;

        float final_left_speed;
        float final_right_speed;

        if(yqj_turn_direction == 1)
        {
            // 左转：左轮慢，右轮快。
            final_left_speed = forward_base_speed - speed_diff;
            final_right_speed = forward_base_speed + speed_diff;
        }
        else
        {
            // 右转：左轮快，右轮慢。
            final_left_speed = forward_base_speed + speed_diff;
            final_right_speed = forward_base_speed - speed_diff;
        }

        // 限幅到正负 3.5 m/s
        if (final_left_speed > 5.0f) final_left_speed = 5.0f;
        if (final_left_speed < -5.0f) final_left_speed = -5.0f;
        if (final_right_speed > 5.0f) final_right_speed = 5.0f;
        if (final_right_speed < -5.0f) final_right_speed = -5.0f;

        *left_target_count = yqj_speed_to_target_count(final_left_speed);
        *right_target_count = yqj_speed_to_target_count(final_right_speed);
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
    yqj_turn_direction = 0;
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
