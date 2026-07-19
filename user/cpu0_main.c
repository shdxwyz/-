#include "zf_common_headfile.h"
#include "isr_config.h"
#include "device.h"
#include "pid.h"
#include "../code/yqj.h"
#include "../code/turn_control.h"
#include "../code/xunji.h"
#include "zf_device_imu660rc.h"

#pragma section all "cpu0_dsram"

// ==================== PIT 与编码器配置 ====================

#define PIT0 (CCU60_CH0)

// 左编码器
#define LEFT_ENCODER (TIM2_ENCODER)
#define LEFT_ENCODER_PULSE (TIM2_ENCODER_CH1_P33_7)
#define LEFT_ENCODER_DIR (TIM2_ENCODER_CH2_P33_6)

// 右编码器
#define RIGHT_ENCODER (TIM4_ENCODER)
#define RIGHT_ENCODER_PULSE (TIM4_ENCODER_CH1_P02_8)
#define RIGHT_ENCODER_DIR (TIM4_ENCODER_CH2_P00_9)

// ==================== 速度 PID 参数 ====================

// 左轮实测：小车走 0.5 米约 27000 个编码器计数，1 米约 54000。
// 更换右编码器后，右轮每米计数略微提高，用于补偿直行时轻微右偏。
#define ENCODER_COUNT_PER_METER (54000.0f)
#define RIGHT_ENCODER_COUNT_PER_METER (60000.0f)
#define RIGHT_ENCODER_COUNT_SCALE \
    (RIGHT_ENCODER_COUNT_PER_METER / ENCODER_COUNT_PER_METER)

// 目标基础速度 1.0 m/s
#define TARGET_SPEED_MPS (1.2f)

// 编码器采样与速度 PID 周期 5ms。
#define PID_PERIOD_MS (5)
#define PID_PERIOD_S (0.005f)

// 任一电机实测速度绝对值超过该值时，锁存急停并关闭全部电机。
#define MOTOR_MAX_SAFE_SPEED_MPS (5.0f)
#define LEFT_MOTOR_MAX_SAFE_COUNT \
    (MOTOR_MAX_SAFE_SPEED_MPS * ENCODER_COUNT_PER_METER * PID_PERIOD_S)
#define RIGHT_MOTOR_MAX_SAFE_COUNT \
    (MOTOR_MAX_SAFE_SPEED_MPS * RIGHT_ENCODER_COUNT_PER_METER * PID_PERIOD_S)

// 5ms 内基础目标计数 = TARGET_SPEED_MPS * 0.005 * ENCODER_COUNT_PER_METER
#define BASE_TARGET_COUNT (TARGET_SPEED_MPS * PID_PERIOD_S * ENCODER_COUNT_PER_METER)
#define RIGHT_BASE_TARGET_COUNT (BASE_TARGET_COUNT * RIGHT_ENCODER_COUNT_SCALE)

// PID 输出范围
#define SPEED_PID_MAX_OUT (8000.0f)
#define SPEED_PID_MAX_IOUT (2000.0f)

// 速度 PID 直接使用单周期编码器计数。周期从 20ms 缩短为 5ms 后，
// 同一速度误差的计数变为 1/4，因此 Kp 放大 4 倍以保持比例输出一致。
// Ki 的单次误差和调用频率刚好互相抵消，保持原值即可。
#define SPEED_KP (11.2f)
#define SPEED_KI (0.02f)
#define SPEED_KD (0.0f)

// 前馈系数：PWM = FEEDFORWARD_GAIN * target_speed
#define FEEDFORWARD_GAIN (500.0f)

// 主循环固定等待时间
#define MAIN_LOOP_PERIOD_MS (1)

// ==================== 巡线参数 ====================

#define SENSOR_NUM (XUNJI_SENSOR_TOTAL)

// 主循环从 2ms 改为 1ms 后，五帧中值仍覆盖约 4ms，和原三帧中值相同。
// alpha=1-sqrt(1-0.9)，使 1ms 更新两次后的低通响应等效于原 2ms 更新一次。
#define ADC_FILTER_ALPHA (0.6838f)
#define ADC_FILTER_HISTORY_NUM (5u)

// ==================== ADC 变量 ====================

// 从左到右：A0 A1 A2 A3 A4 A5 A6 A7 A8 A10 A11 A12 A13 A16 A17
// A0 A1 和 A16 A17 不用于巡线，用于转弯操作
uint16 adc_value[SENSOR_NUM];

static uint16 adc_filter_history[SENSOR_NUM][ADC_FILTER_HISTORY_NUM];
static float adc_filter_output[SENSOR_NUM];
static uint8 adc_filter_history_index = 0;
static uint8 adc_filter_initialized = 0;

adc_channel_enum adc_list[SENSOR_NUM] =
    {
        ADC0_CH0_A0,
        ADC0_CH1_A1,
        ADC0_CH2_A2,
        ADC0_CH3_A3,
        ADC0_CH4_A4,
        ADC0_CH5_A5,
        ADC0_CH6_A6,
        ADC0_CH7_A7,
        ADC0_CH8_A8,
        ADC0_CH10_A10,
        ADC0_CH11_A11,
        ADC0_CH12_A12,
        ADC0_CH13_A13,
        ADC1_CH0_A16,
        ADC1_CH1_A17};

// ==================== 编码器与 PID 变量 ====================

// 5ms 内编码器增量，用于速度 PID
volatile int16 left_encoder_count = 0;
volatile int16 right_encoder_count = 0;

// 软件累计总计数，用于计算总路程
volatile int32 left_encoder_total = 0;
volatile int32 right_encoder_total = 0;

// 小车总路程，单位 m
volatile float car_distance_m = 0.0f;

// 巡线算出的左右目标编码器计数
volatile float left_target_count = BASE_TARGET_COUNT;
volatile float right_target_count = RIGHT_BASE_TARGET_COUNT;

// PID 输出 PWM
volatile float left_base_pwm = 0;
volatile float right_base_pwm = 0;

volatile uint8 left_speed_decel_flag = 0;
volatile uint8 right_speed_decel_flag = 0;

PidTypeDef left_speed_pid;
PidTypeDef right_speed_pid;

// ==================== 函数声明 ====================

void adc_all_init(void);
void adc_all_read(void);

int16 limit_int16(int16 value, int16 min, int16 max);
void set_speed_change_flags(float next_left_target,
                            float next_right_target,
                            float current_left_target,
                            float current_right_target);

// ==================== 元器件顺序配置 ====================

typedef uint8 (*yqj_trigger_func)(const uint16 adc_value[]);

typedef struct
{
    yqj_trigger_func trigger;
    uint8 turn_enabled;
    float pass_speed_mps;
    float turn_base_speed;
    uint32 delay_ms;
    float delay_distance_m;
    uint32 run_ms;
    float run_distance_m;
    uint32 lock_ms;
    float lock_distance_m;
} yqj_case_config_struct;

// 参数顺序：触发函数、是否转弯、直行速度、转弯速度、
//           延时(ms)、延时距离(m)、运行(ms)、运行距离(m)、锁定(ms)、锁定距离(m)。
// 新增步骤时只需在数组中插入或追加一行，后续步骤序号会自动顺延。
#define YQJ_CASE(trigger_, turn_, pass_speed_, turn_speed_, delay_ms_, delay_m_, run_ms_, run_m_, lock_ms_, lock_m_) \
    {trigger_, turn_, pass_speed_, turn_speed_, delay_ms_, delay_m_, run_ms_, run_m_, lock_ms_, lock_m_}

static const yqj_case_config_struct yqj_case_table[] =
{
    // 电源
    YQJ_CASE(yqj_dianyuan_trigger,        0, 2.0f,             0.0f,  0, 0.0f,  10, 0.2f,  66, 0.2f),
    // 右转
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.0f,  0, 0.0f, 120, 0.0f,  50, 0.6f),
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.0f,  0, 0.0f, 120, 0.0f,  50, 0.2f),
        // 二级管，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 1.5f,             0.0f,  0, 0.0f,   0, 0.2f,  50, 0.2f),
    // 左转
    YQJ_CASE(yqj_left_turn_trigger,       1, TARGET_SPEED_MPS,  1.0f,  0, 0.0f, 120, 0.0f,  50, 0.2f),
    // 三极管，直行通过
    YQJ_CASE(yqj_sanjiguan1_2trigger,     0, 2.0f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.5f),
        // 二极管，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 1.5f,             0.0f,  0, 0.0f,   0, 0.0f,  50, 0.4f),
    // 三极管，左转
    YQJ_CASE(yqj_sanjiguan0_1trigger,     1, TARGET_SPEED_MPS,  1.2f, 50, 0.0f,  20, 0.0f,  50, 0.4f),
    // 左弯，直行通过
    YQJ_CASE(yqj_left_turn_trigger,       0, 1.5f,             0.0f,  0, 0.0f,   0, 0.2f,  50, 0.3f),
    YQJ_CASE(yqj_left_turn_trigger,       0, 1.5f,             0.0f,  0, 0.0f,   0, 0.2f,  50, 0.3f),
    // 左转
    YQJ_CASE(yqj_left_turn_trigger,       1.2, TARGET_SPEED_MPS,  1.0f,  0, 0.0f, 100, 0.0f,  33, 0.2f),
    // 二极管，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 2.5f,             0.0f,  0, 0.0f,   0, 0.0f,  50, 0.2f),
    // 电容，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 1.5f,             0.0f,  0, 0.0f,   0, 0.0f,  50, 0.2f),
    // 三极管，左转
    YQJ_CASE(yqj_sanjiguan1_2trigger,     1, TARGET_SPEED_MPS,  1.1f,  0, 0.0f, 120, 0.0f,  33, 0.2f),

    // 左转
    YQJ_CASE(yqj_left_turn_trigger,       1, TARGET_SPEED_MPS,  1.1f,  0, 0.0f,  10, 0.0f,  10, 0.2f),
        // 二级管，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 1.5f,             0.0f,  0, 0.0f,   0, 0.0f,  0, 0.4f),
    // 右弯，直行通过
    YQJ_CASE(yqj_right_turn_trigger,      0, 2.0f,             0.0f,  0, 0.0f,   0, 0.1f,  0, 0.0f),
    // 电容，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 1.0f,             0.0f,  0, 0.0f,   0, 0.0f,  0, 0.2f),
    // 双触发，右转
    YQJ_CASE(yqj_double_trigger,          1, TARGET_SPEED_MPS, -1.0f, 0, 0.0f, 100, 0.0f,  33, 0.2f),
    // 右转
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.0f,  0, 0.0f, 120, 0.0f, 150, 0.8f),
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.0f,  0, 0.0f, 120, 0.0f,  50, 0.0f),
    // 双触发，右转
    YQJ_CASE(yqj_double_trigger,          1, TARGET_SPEED_MPS, -1.1f, 0, 0.0f,  10, 0.0f,  33, 0.1f),
        // 二级管，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 1.5f,             0.0f,  0, 0.0f,   0, 0.0f,  50, 0.2f),
        // 双触发，右转
    YQJ_CASE(yqj_double_trigger,          1, TARGET_SPEED_MPS, -1.0f,  0, 0.0f, 0, 0.0f,  33, 0.5f),
   // 右弯，直行通过
    YQJ_CASE(yqj_right_turn_trigger,      0, 2.2f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.3f),
    // 三极管，直行通过
    YQJ_CASE(yqj_sanjiguan2_0trigger,     0, 1.2f,             0.0f,  0, 0.0f,   0, 0.2f,  50, 0.3f),
    // 右弯，直行通过
    YQJ_CASE(yqj_right_turn_trigger,      0, 1.2f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.1f),
    // 右转
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.1f,  0, 0.0f,  10, 0.0f,  33, 0.8f),
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.0f,  0, 0.0f,  12, 0.0f,  15, 0.5f),
    // 特别二极管，右转
    YQJ_CASE(yqj_tberjiguan_trigger,      1, TARGET_SPEED_MPS, -1.0f,  0, 0.05f,120, 0.0f,  50, 0.4f),
    // 右转
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -0.9f,  0, 0.0f,  12, 0.0f,  15, 0.1f),
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.0f,  0, 0.0f,  12, 0.0f,  15, 0.4f),
    // 右弯，直行通过
    YQJ_CASE(yqj_right_turn_trigger,      0, 2.3f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.6f),
    YQJ_CASE(yqj_right_turn_trigger,      0, 2.5f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.2f),
    YQJ_CASE(yqj_right_turn_trigger,      0, 2.3f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.2f),
    // 右转
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.0f,  0, 0.0f,  12, 0.0f,  15, 2.4f),
   // YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.0f,  0, 0.0f, 120, 0.0f, 150, 2.4f),
    // 开关
   // YQJ_CASE(yqj_kaiguang0_1trigger,      1, TARGET_SPEED_MPS,  0.0f,  0, 0.0f, 100, 0.0f, 140, 0.5f),
    // 电源，左转
 //   YQJ_CASE(yqj_dianyuan_trigger,        1, TARGET_SPEED_MPS,  1.0f,  0, 0.0f, 100, 0.0f,  66, 0.4f)
};

#define YQJ_CASE_COUNT ((uint16)(sizeof(yqj_case_table) / sizeof(yqj_case_table[0])))

// ==================== 主函数 ====================

int core0_main(void)
{
    xunji_result_struct line_result = {0, 0, BASE_TARGET_COUNT, BASE_TARGET_COUNT};

    int16 left_pwm = 0;
    int16 right_pwm = 0;
    int16 left_pwm_min = 0;
    int16 right_pwm_min = 0;

    uint32 print_count = 0;
    float final_left_target = BASE_TARGET_COUNT;
    float final_right_target = BASE_TARGET_COUNT;
    float action_left_target = BASE_TARGET_COUNT;
    float action_right_target = BASE_TARGET_COUNT;

    uint8 yqj_condition = 0;
    uint8 yqj_case_trigger = 0;
    uint8 yqj_turn_done = 1;
    uint32 yqj_delay_ms = 0;
    float yqj_delay_distance_m = 0.0f; // DELAY 阶段不巡线距离，0 表示不限制距离
    uint32 yqj_run_ms = 0;
    uint32 yqj_lock_ms = 0;
    float yqj_lock_distance_m = 0.0f;
    float yqj_run_distance_m = 0.0f; // RUN 阶段不巡线距离，0 表示不限制距离
    float yqj_pass_speed_mps = TARGET_SPEED_MPS; // RUN 中非转向阶段的同速直行速度
    float yqj_turn_base_speed = 0.0f;  // 转弯基础速度（m/s），正数=左转，负数=右转

    clock_init();
    debug_init();

    // ADC 初始化
    adc_all_init();

    // 编码器初始化
    encoder_dir_init(LEFT_ENCODER, LEFT_ENCODER_PULSE, LEFT_ENCODER_DIR);
    encoder_dir_init(RIGHT_ENCODER, RIGHT_ENCODER_PULSE, RIGHT_ENCODER_DIR);

    // 电机初始化
    motor_init();

    // PID 初始化
    PID_Init(&left_speed_pid,
             PID_POSITION,
             SPEED_PID_MAX_OUT,
             SPEED_PID_MAX_IOUT,
             SPEED_KP,
             SPEED_KI,
             SPEED_KD);

    PID_Init(&right_speed_pid,
             PID_POSITION,
             SPEED_PID_MAX_OUT,
             SPEED_PID_MAX_IOUT,
             SPEED_KP,
             SPEED_KI,
             SPEED_KD);

    cpu_wait_event_ready();

    pwm_init(ATOM0_CH6_P02_6, 100, 1300);
    system_delay_ms(2000);

    // IMU660RC 初始化（240Hz 四元数输出）
    imu660rc_init(IMU660RC_QUARTERNION_240HZ);
   
    turn_control_init(PID_PERIOD_S, ENCODER_COUNT_PER_METER);
    yqj_init(ENCODER_COUNT_PER_METER);

    encoder_clear_count(LEFT_ENCODER);
    encoder_clear_count(RIGHT_ENCODER);
    left_encoder_count = 0;
    right_encoder_count = 0;
    pit_ms_init(PIT0, PID_PERIOD_MS);
    while (TRUE)
    {
        // ==================== 读取 15 路 ADC ====================

        adc_all_read();
        // ==================== 巡线层 ====================
        // xunji 只根据 ADC 计算普通巡线目标，不处理特殊动作命令。
        // 传入完整 ADC 数组，xunji 直接使用真实下标 [2]~[12]。
        xunji_update(adc_value, BASE_TARGET_COUNT, &line_result);

        // ==================== 元器件顺序层 ====================
        // 总流程：正常巡线、判断当前 flag、延时、执行动作、自锁、flag 加一。
        yqj_condition = 0;
        yqj_case_trigger = 0;
        yqj_delay_ms = 0;
        yqj_delay_distance_m = 0.0f;
        yqj_run_ms = 0;
        yqj_lock_ms = 0;
        yqj_lock_distance_m = 0.0f;
        yqj_run_distance_m = 0.0f;
        yqj_pass_speed_mps = TARGET_SPEED_MPS;
        yqj_turn_base_speed = 0.0f;

        final_left_target = line_result.left_target_count;
        final_right_target = line_result.right_target_count;

        if ((yqj_flag >= 1u) && (yqj_flag <= YQJ_CASE_COUNT))
        {
            const yqj_case_config_struct *case_config = &yqj_case_table[yqj_flag - 1u];

            yqj_condition = case_config->trigger(adc_value);
            yqj_case_trigger = case_config->turn_enabled;
            yqj_pass_speed_mps = case_config->pass_speed_mps;
            yqj_turn_base_speed = case_config->turn_base_speed;
            yqj_delay_ms = case_config->delay_ms;
            yqj_delay_distance_m = case_config->delay_distance_m;
            yqj_run_ms = case_config->run_ms;
            yqj_run_distance_m = case_config->run_distance_m;
            yqj_lock_ms = case_config->lock_ms;
            yqj_lock_distance_m = case_config->lock_distance_m;
        }
        else
        {
            motor_stop();
            system_delay_ms(20000);
        }

        if (YQJ_STATE_LINE == yqj_state)
        {
            if (yqj_condition)
            {
                yqj_start_case(yqj_case_trigger,
                               left_encoder_total + right_encoder_total);
            }
        }
        else if (YQJ_STATE_DELAY == yqj_state)
        {
            if (yqj_delay_done(left_encoder_total + right_encoder_total,
                               yqj_delay_ms,
                               yqj_delay_distance_m))
            {
                if (yqj_get_action_trigger())
                {
                    action_left_target = yqj_turn_base_speed * PID_PERIOD_S * ENCODER_COUNT_PER_METER;
                    action_right_target = yqj_turn_base_speed * PID_PERIOD_S * ENCODER_COUNT_PER_METER;
                    set_speed_change_flags(action_left_target,
                                           action_right_target,
                                           final_left_target,
                                           final_right_target);
                }

                yqj_turn_done = yqj_get_action_trigger() ? 0u : 1u;
                yqj_start_run(left_encoder_total + right_encoder_total);
            }
        }
        else if (YQJ_STATE_RUN == yqj_state)
        {
            uint8 run_conditions_done;

            if (yqj_get_action_trigger() && !yqj_turn_done)
            {
                // 转向由相对 yaw 达到 85 度结束；超时仅用于传感器异常或堵转保护。
                if (turn_control_target_reached() ||
                    yqj_time_reached(yqj_state_start_time,
                                     TURN_CONTROL_TIMEOUT_MS))
                {
                    uint32 turn_stop_interrupt_state;

                    // yaw 转向结束后立即清除转向输出；未完成的 RUN 时间/距离改为同速直行。
                    yqj_turn_done = 1;
                    turn_control_stop();
                    left_speed_decel_flag = 0;
                    right_speed_decel_flag = 0;

                    // 立即清除旧的转弯 PWM，避免等待下一个 5ms 速度环周期时继续转动。
                    turn_stop_interrupt_state = interrupt_global_disable();
                    left_target_count = 0.0f;
                    right_target_count = 0.0f;
                    left_base_pwm = 0.0f;
                    right_base_pwm = 0.0f;
                    PID_clear(&left_speed_pid);
                    PID_clear(&right_speed_pid);
                    motor_set_left(0);
                    motor_set_right(0);
                    interrupt_global_enable(turn_stop_interrupt_state);
                }
            }

            run_conditions_done =
                yqj_run_done(left_encoder_total + right_encoder_total,
                             yqj_run_ms,
                             yqj_run_distance_m);

            if (yqj_turn_done && run_conditions_done)
            {
                yqj_start_lock(left_encoder_total + right_encoder_total);
            }
        }
        else if (YQJ_STATE_LOCK == yqj_state)
        {
            if (yqj_lock_done(left_encoder_total + right_encoder_total,
                              yqj_lock_ms,
                              yqj_lock_distance_m))
            {
                left_speed_decel_flag = 0;
                right_speed_decel_flag = 0;
                yqj_finish_case();
            }
        }
        else
        {
            yqj_turn_done = 1;
            turn_control_stop();
            yqj_set_flag(0);
        }

        if (YQJ_STATE_DELAY == yqj_state)
        {
            // DELAY 全程不巡线，左右轮按通过速度同速直行。
            float delay_target_count = yqj_pass_speed_mps *
                                       PID_PERIOD_S *
                                       ENCODER_COUNT_PER_METER;
            final_left_target = delay_target_count;
            final_right_target = delay_target_count;
        }
        else if (YQJ_STATE_RUN == yqj_state)
        {
            if (yqj_get_action_trigger() && !yqj_turn_done)
            {
                turn_control_apply(yqj_turn_base_speed,
                                   &final_left_target,
                                   &final_right_target);
            }
            else
            {
                // RUN 全程不巡线；非转向阶段左右轮按通过速度同速直行。
                float run_target_count = yqj_pass_speed_mps *
                                         PID_PERIOD_S *
                                         ENCODER_COUNT_PER_METER;
                final_left_target = run_target_count;
                final_right_target = run_target_count;
            }
        }

        // ==================== 直接使用巡线结果，不做软启动 ====================
        left_target_count = final_left_target;
        // 高层控制统一按左轮计数标定计算，写入速度环前换算成右编码器计数。
        right_target_count = final_right_target * RIGHT_ENCODER_COUNT_SCALE;

        left_pwm_min = left_speed_decel_flag ? -PWM_DUTY_MAX : 0;
        right_pwm_min = right_speed_decel_flag ? -PWM_DUTY_MAX : 0;

        left_pwm = limit_int16((int16)left_base_pwm, left_pwm_min, PWM_DUTY_MAX);
        right_pwm = limit_int16((int16)right_base_pwm, right_pwm_min, PWM_DUTY_MAX);

        motor_control(left_pwm, right_pwm);

        // ==================== 串口调试 ====================

        print_count++;
        // 主循环固定等待从 2ms 改为 1ms，计数翻倍后仍约每 100ms 输出一次。
        if (print_count >= 100)
        {
            uint8 i;
            float left_speed_mps = (float)left_encoder_count /
                                   (ENCODER_COUNT_PER_METER * PID_PERIOD_S);
            float right_speed_mps = (float)right_encoder_count /
                                    (RIGHT_ENCODER_COUNT_PER_METER * PID_PERIOD_S);

            print_count = 0;

            printf("ADC:");
            for (i = 0; i < SENSOR_NUM; i++)
            {
                printf(" %5d", adc_value[i]);
            }
            printf("\r\n");

            printf("flag=%2d motorFault=%d turnAngle=%7.2f pwmL=%5d pwmR=%5d targetL=%5d targetR=%5d encL=%5d encR=%5d speedL=%5.3f speedR=%5.3f\r\n",
                   yqj_flag,
                   motor_emergency_is_latched(),
                   turn_control_angle_deg,
                   left_pwm,
                   right_pwm,
                   (int)left_target_count,
                   (int)right_target_count,
                   left_encoder_count,
                   right_encoder_count,
                   left_speed_mps,
                   right_speed_mps);
        }

        system_delay_ms(MAIN_LOOP_PERIOD_MS);
    }
}

// ==================== ADC 初始化 ====================

void adc_all_init(void)
{
    uint8 i;
    for (i = 0; i < SENSOR_NUM; i++)
    {
        adc_init(adc_list[i], ADC_12BIT);
    }
}

// ==================== ADC 读取 ====================

static uint16 adc_history_median(const uint16 values[])
{
    uint16 sorted[ADC_FILTER_HISTORY_NUM];
    uint16 temp;
    uint8 i;
    uint8 j;

    for (i = 0; i < ADC_FILTER_HISTORY_NUM; i++)
    {
        sorted[i] = values[i];
        j = i;
        while ((j > 0u) && (sorted[j - 1u] > sorted[j]))
        {
            temp = sorted[j - 1u];
            sorted[j - 1u] = sorted[j];
            sorted[j] = temp;
            j--;
        }
    }

    return sorted[ADC_FILTER_HISTORY_NUM / 2u];
}

void adc_all_read(void)
{
    uint8 i;
    uint8 j;
    uint16 raw_value;
    uint16 median_value;

    for (i = 0; i < SENSOR_NUM; i++)
    {
        // 先对当前时刻连续采样 3 次取平均，再对主循环之间的数据做时域滤波。
        raw_value = adc_mean_filter_convert(adc_list[i], 3);

        if (!adc_filter_initialized)
        {
            for (j = 0; j < ADC_FILTER_HISTORY_NUM; j++)
            {
                adc_filter_history[i][j] = raw_value;
            }
            adc_filter_output[i] = (float)raw_value;
            adc_value[i] = raw_value;
        }
        else
        {
            adc_filter_history[i][adc_filter_history_index] = raw_value;
            median_value = adc_history_median(adc_filter_history[i]);

            adc_filter_output[i] += ADC_FILTER_ALPHA *
                                    ((float)median_value - adc_filter_output[i]);
            adc_value[i] = (uint16)(adc_filter_output[i] + 0.5f);
        }
    }

    if (!adc_filter_initialized)
    {
        adc_filter_initialized = 1;
        adc_filter_history_index = 0;
    }
    else
    {
        adc_filter_history_index =
            (uint8)((adc_filter_history_index + 1u) % ADC_FILTER_HISTORY_NUM);
    }
}

// ==================== 5ms 速度 PID 中断 ====================

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);

    // ==================== 读取 5ms 内编码器增量 ====================
    // 左编码器前进时是负数，所以取反变成正数。
    left_encoder_count = -encoder_get_count(LEFT_ENCODER);

    // 右编码器前进时是正数，直接读取。
    right_encoder_count = encoder_get_count(RIGHT_ENCODER);

    // ==================== 读完立刻清空硬件编码器 ====================

    encoder_clear_count(LEFT_ENCODER);
    encoder_clear_count(RIGHT_ENCODER);

    // ==================== 电机超速保护 ====================
    // 使用编码器实测速度而不是目标速度；正转、反转均按绝对值判断。
    // 5ms 内任一编码器计数超过 5m/s 对应阈值，立即锁存并关闭两侧电机。
    if(((float)left_encoder_count > LEFT_MOTOR_MAX_SAFE_COUNT) ||
       ((float)left_encoder_count < -LEFT_MOTOR_MAX_SAFE_COUNT) ||
       ((float)right_encoder_count > RIGHT_MOTOR_MAX_SAFE_COUNT) ||
       ((float)right_encoder_count < -RIGHT_MOTOR_MAX_SAFE_COUNT))
    {
        left_target_count = 0.0f;
        right_target_count = 0.0f;
        left_base_pwm = 0.0f;
        right_base_pwm = 0.0f;
        PID_clear(&left_speed_pid);
        PID_clear(&right_speed_pid);
        motor_emergency_stop();
        return;
    }

    // 已触发的故障禁止速度环继续运算，急停锁存只能通过重新上电清除。
    if(motor_emergency_is_latched())
    {
        left_base_pwm = 0.0f;
        right_base_pwm = 0.0f;
        return;
    }

    // ==================== 软件累计总路程 ====================

    left_encoder_total += left_encoder_count;
    right_encoder_total += right_encoder_count;

    car_distance_m = 0.5f *
                     ((float)left_encoder_total / ENCODER_COUNT_PER_METER +
                      (float)right_encoder_total / RIGHT_ENCODER_COUNT_PER_METER);

    // ==================== 速度 PID ====================
    // PID_Calc(pid, 实际值, 目标值)

    left_base_pwm = PID_Calc(&left_speed_pid,
                             (float)left_encoder_count,
                             left_target_count);

    right_base_pwm = PID_Calc(&right_speed_pid,
                              (float)right_encoder_count,
                              right_target_count);

    // ==================== 前馈控制 ====================
    // 根据目标速度直接给基础 PWM，减小 PID 负担。
    // 前馈 = FEEDFORWARD_GAIN * target_speed
    // target_speed = target_count / (PID_PERIOD_S * ENCODER_COUNT_PER_METER)
    left_base_pwm += FEEDFORWARD_GAIN * left_target_count / (PID_PERIOD_S * ENCODER_COUNT_PER_METER);
    right_base_pwm += FEEDFORWARD_GAIN * right_target_count /
                      (PID_PERIOD_S * RIGHT_ENCODER_COUNT_PER_METER);
}

// ==================== 限幅函数 ====================

int16 limit_int16(int16 value, int16 min, int16 max)
{
    if (value > max)
    {
        return max;
    }
    else if (value < min)
    {
        return min;
    }
    else
    {
        return value;
    }
}

void set_speed_change_flags(float next_left_target,
                            float next_right_target,
                            float current_left_target,
                            float current_right_target)
{
    left_speed_decel_flag = (next_left_target < current_left_target) ? 1 : 0;
    right_speed_decel_flag = (next_right_target < current_right_target) ? 1 : 0;
}

#pragma section all restore
