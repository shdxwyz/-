#include "turn_control.h"

typedef enum
{
    TURN_CONTROL_DIRECTION_NONE = 0,
    TURN_CONTROL_DIRECTION_LEFT,
    TURN_CONTROL_DIRECTION_RIGHT
} turn_control_direction_enum;

volatile float turn_control_angle_deg = 0.0f;
float turn_control_angle_pid_output = 0.0f;

static PidTypeDef turn_control_angle_pid;
static float turn_control_pid_period_s = 0.005f;
static float turn_control_encoder_count_per_meter = 12106.0f;

// 四元数解算后的 yaw 持续更新，每次转向只重新记录起点。
static volatile float turn_control_current_yaw_deg = 0.0f;
static volatile float turn_control_start_yaw_deg = 0.0f;
static volatile uint8 turn_control_yaw_valid = 0;
static volatile uint8 turn_control_yaw_started = 0;
static volatile uint8 turn_control_direction = TURN_CONTROL_DIRECTION_NONE;
static volatile uint8 turn_control_yaw_reject_count = 0u;
static volatile uint8 turn_control_yaw_fault = 0u;

// 将速度 m/s 换算成一个速度 PID 周期内的编码器目标计数。
static float turn_control_speed_to_target_count(float speed_mps)
{
    return speed_mps *
           turn_control_pid_period_s *
           turn_control_encoder_count_per_meter;
}

// 将两个 0~360 度 yaw 的差值折算到 -180~180 度，正确处理跨零点。
static float turn_control_wrap_yaw_delta(float yaw_delta)
{
    while (yaw_delta > 180.0f)
    {
        yaw_delta -= 360.0f;
    }
    while (yaw_delta < -180.0f)
    {
        yaw_delta += 360.0f;
    }

    return yaw_delta;
}

// 记录无效帧。单帧毛刺只丢弃，连续异常才使本次转弯进入故障状态。
static void turn_control_reject_yaw_sample(void)
{
    if (TURN_CONTROL_DIRECTION_NONE == turn_control_direction)
    {
        return;
    }

    if (turn_control_yaw_reject_count < 255u)
    {
        turn_control_yaw_reject_count++;
    }

    if (turn_control_yaw_reject_count >=
        TURN_CONTROL_YAW_MAX_REJECT_COUNT)
    {
        turn_control_yaw_fault = 1u;
    }
}

// 第一次输出转向目标时调用，原子地清角度环并记录本次起始 yaw。
static void turn_control_begin(uint8 direction)
{
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    PID_clear(&turn_control_angle_pid);
    turn_control_angle_deg = 0.0f;
    turn_control_angle_pid_output = 0.0f;
    turn_control_start_yaw_deg = turn_control_current_yaw_deg;
    turn_control_yaw_reject_count = 0u;
    turn_control_yaw_fault = 0u;
    turn_control_yaw_started = turn_control_yaw_valid ? 1u : 0u;
    turn_control_direction = direction;

    interrupt_global_enable(interrupt_state);
}

void turn_control_init(float pid_period_s, float encoder_count_per_meter)
{
    turn_control_pid_period_s = pid_period_s;
    turn_control_encoder_count_per_meter = encoder_count_per_meter;

    PID_Init(&turn_control_angle_pid,
             PID_POSITION,
             TURN_CONTROL_ANGLE_PID_MAX_OUT,
             TURN_CONTROL_ANGLE_PID_MAX_IOUT,
             TURN_CONTROL_ANGLE_KP,
             TURN_CONTROL_ANGLE_KI,
             TURN_CONTROL_ANGLE_KD);

    turn_control_stop();
}

// 结束本次转向。保留最终角度供串口观察，下一次开始时再清零。
void turn_control_stop(void)
{
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    turn_control_direction = TURN_CONTROL_DIRECTION_NONE;
    turn_control_yaw_started = 0;
    turn_control_yaw_reject_count = 0u;
    turn_control_yaw_fault = 0u;
    turn_control_angle_pid_output = 0.0f;
    PID_clear(&turn_control_angle_pid);
    interrupt_global_enable(interrupt_state);
}

// 每次四元数数据就绪后更新 yaw，并按转向方向计算相对起点的角度。
void turn_control_update_yaw(float yaw_deg)
{
    float yaw_delta;

    // NaN、无穷大以及明显越界值不能进入回绕循环和角度环。
    if ((yaw_deg != yaw_deg) ||
        (yaw_deg > 720.0f) ||
        (yaw_deg < -360.0f))
    {
        turn_control_reject_yaw_sample();
        return;
    }

    while (yaw_deg >= 360.0f)
    {
        yaw_deg -= 360.0f;
    }
    while (yaw_deg < 0.0f)
    {
        yaw_deg += 360.0f;
    }

    turn_control_current_yaw_deg = yaw_deg;
    turn_control_yaw_valid = 1;
    turn_control_yaw_reject_count = 0u;

    if (TURN_CONTROL_DIRECTION_NONE == turn_control_direction)
    {
        return;
    }

    // 如果开始转向时还没有有效四元数，用第一帧有效 yaw 补记起点。
    if (!turn_control_yaw_started)
    {
        turn_control_start_yaw_deg = yaw_deg;
        turn_control_angle_deg = 0.0f;
        turn_control_yaw_started = 1;
        return;
    }

    // 停止角只需要本次起点与当前姿态之间的转角大小。
    // 最短角差天然处理任意起点和 0/360 度跨界，结果始终位于 0~180 度，
    // 因此反向小抖动不会再被解释为接近 360 度。
    yaw_delta = turn_control_wrap_yaw_delta(yaw_deg -
                                            turn_control_start_yaw_deg);
    if (yaw_delta < 0.0f)
    {
        yaw_delta = -yaw_delta;
    }

    turn_control_angle_deg = yaw_delta;
}

uint8 turn_control_target_reached(void)
{
    if ((TURN_CONTROL_DIRECTION_NONE == turn_control_direction) ||
        !turn_control_yaw_started)
    {
        return 0;
    }

    return (turn_control_angle_deg >= TURN_CONTROL_STOP_YAW_ANGLE_DEG);
}

// 仅检查连续 NaN 或明显越界值；不再检查相邻跳变和 YAW 丢帧时间。
uint8 turn_control_yaw_faulted(void)
{
    if (TURN_CONTROL_DIRECTION_NONE == turn_control_direction)
    {
        return 0u;
    }

    return turn_control_yaw_fault;
}

// 角度误差经过 PID 变成左右轮速度差，再换算为编码器目标计数。
void turn_control_apply(float turn_base_speed,
                        float *left_target_count,
                        float *right_target_count)
{
    float forward_base_speed;
    float speed_diff;
    float final_left_speed;
    float final_right_speed;
    uint8 requested_direction;

    forward_base_speed = (turn_base_speed >= 0.0f) ?
                         turn_base_speed : -turn_base_speed;
    requested_direction = (turn_base_speed >= 0.0f) ?
                          TURN_CONTROL_DIRECTION_LEFT :
                          TURN_CONTROL_DIRECTION_RIGHT;

    if (TURN_CONTROL_DIRECTION_NONE == turn_control_direction)
    {
        turn_control_begin(requested_direction);
    }

    turn_control_angle_pid_output =
        PID_Calc(&turn_control_angle_pid,
                 turn_control_angle_deg,
                 TURN_CONTROL_TARGET_ANGLE_DEG);
    speed_diff = turn_control_angle_pid_output;
    if (speed_diff < 0.0f)
    {
        speed_diff = 0.0f;
    }

    if (TURN_CONTROL_DIRECTION_LEFT == turn_control_direction)
    {
        final_left_speed = forward_base_speed - speed_diff;
        final_right_speed = forward_base_speed + speed_diff;
    }
    else
    {
        final_left_speed = forward_base_speed + speed_diff;
        final_right_speed = forward_base_speed - speed_diff;
    }

    if (final_left_speed > TURN_CONTROL_WHEEL_SPEED_LIMIT_MPS)
    {
        final_left_speed = TURN_CONTROL_WHEEL_SPEED_LIMIT_MPS;
    }
    if (final_left_speed < -TURN_CONTROL_WHEEL_SPEED_LIMIT_MPS)
    {
        final_left_speed = -TURN_CONTROL_WHEEL_SPEED_LIMIT_MPS;
    }
    if (final_right_speed > TURN_CONTROL_WHEEL_SPEED_LIMIT_MPS)
    {
        final_right_speed = TURN_CONTROL_WHEEL_SPEED_LIMIT_MPS;
    }
    if (final_right_speed < -TURN_CONTROL_WHEEL_SPEED_LIMIT_MPS)
    {
        final_right_speed = -TURN_CONTROL_WHEEL_SPEED_LIMIT_MPS;
    }

    *left_target_count = turn_control_speed_to_target_count(final_left_speed);
    *right_target_count = turn_control_speed_to_target_count(final_right_speed);
}
