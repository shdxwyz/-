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

#define SPEED_CONTROL_PIT (CCU60_CH0)
#define MAIN_CONTROL_PIT (CCU60_CH1)

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
// 两侧编码器硬件相同；当前右侧60000作为既有控制补偿保留。
#define ENCODER_COUNT_PER_METER (54000.0f)
#define RIGHT_ENCODER_COUNT_PER_METER (62000.0f)
#define RIGHT_ENCODER_COUNT_SCALE \
    (RIGHT_ENCODER_COUNT_PER_METER / ENCODER_COUNT_PER_METER)

// 目标基础速度 1.4 m/s
#define TARGET_SPEED_MPS (1.4f)

// 编码器采样与速度 PID 周期 2ms。
#define PID_PERIOD_MS (2)
#define PID_PERIOD_S ((float)PID_PERIOD_MS / 1000.0f)
#define PID_PERIOD_10NS_TICKS ((uint32)(PID_PERIOD_MS * 100000UL))

// 任一电机实测速度绝对值超过该值时，锁存急停并关闭全部电机。
#define MOTOR_MAX_SAFE_SPEED_MPS (12.0f)
#define LEFT_MOTOR_MAX_SAFE_COUNT \
    (MOTOR_MAX_SAFE_SPEED_MPS * ENCODER_COUNT_PER_METER * PID_PERIOD_S)
#define RIGHT_MOTOR_MAX_SAFE_COUNT \
    (MOTOR_MAX_SAFE_SPEED_MPS * RIGHT_ENCODER_COUNT_PER_METER * PID_PERIOD_S)

// 单个速度环周期内的基础目标计数。
#define BASE_TARGET_COUNT (TARGET_SPEED_MPS * PID_PERIOD_S * ENCODER_COUNT_PER_METER)
#define RIGHT_BASE_TARGET_COUNT (BASE_TARGET_COUNT * RIGHT_ENCODER_COUNT_SCALE)

// PID 输出范围
#define SPEED_PID_MAX_OUT (10000.0f)
#define SPEED_PID_MAX_IOUT (8000.0f)

// 速度环由 5ms 缩短为 2ms 后，count 域误差缩小为 2/5，
// Kp 放大到原来的 5/2，以保持相同 m/s 误差对应的 PWM 修正。
// 右编码器每米计数更多，因此右轮 count 域增益按比例减小，
// 使相同的 m/s 误差在左右两侧产生近似相同的 PWM 修正。
#define LEFT_SPEED_KP (30.0f)
#define RIGHT_SPEED_KP (LEFT_SPEED_KP / RIGHT_ENCODER_COUNT_SCALE)
#define LEFT_SPEED_KI (0.02f)
#define RIGHT_SPEED_KI (LEFT_SPEED_KI / RIGHT_ENCODER_COUNT_SCALE)
#define SPEED_KD (0.0f)

// 前馈系数：PWM = FEEDFORWARD_GAIN * target_speed
#define FEEDFORWARD_GAIN (500.0f)

// 主控制任务由独立 PIT 提供严格的 1kHz 节拍。
#define MAIN_CONTROL_PERIOD_MS (1)

// printf 在 115200 波特率下会阻塞主循环约 20ms。
// 实车高速运行时默认关闭；台架调试时可临时改为 1。
#define CONTROL_DEBUG_PRINT_ENABLE (0)
#define CONTROL_DEBUG_PRINT_INTERVAL_LOOPS (100u)

// ==================== 巡线参数 ====================

#define SENSOR_NUM (XUNJI_SENSOR_TOTAL)

// 三帧中值可剔除单帧尖峰，同时只引入约 1ms 的检测延迟。
// 巡线继续使用低通输出；元器件触发直接使用中值结果以更快响应。
#define ADC_FILTER_ALPHA (0.85f)
#define ADC_FILTER_HISTORY_NUM (3u)

// ==================== ADC 变量 ====================

// 从左到右：A0 A1 A2 A3 A4 A5 A6 A7 A8 A10 A11 A12 A13 A16 A17
// A0 A1 和 A16 A17 不用于巡线，用于转弯操作
uint16 adc_value[SENSOR_NUM];

static uint16 adc_filter_history[SENSOR_NUM][ADC_FILTER_HISTORY_NUM];
static uint16 adc_trigger_value[SENSOR_NUM];
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

// 单个速度环周期内的编码器增量，用于速度 PID。
volatile int16 left_encoder_count = 0;
volatile int16 right_encoder_count = 0;

// 根据真实采样间隔归一化到一个 2ms 周期的反馈计数，供速度 PID 使用。
static volatile float left_encoder_feedback_count = 0.0f;
static volatile float right_encoder_feedback_count = 0.0f;
static uint16 left_encoder_previous_raw = 0u;
static uint16 right_encoder_previous_raw = 0u;
static uint32 encoder_previous_sample_time = 0u;

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
static volatile int16 left_applied_pwm = 0;
static volatile int16 right_applied_pwm = 0;

volatile uint8 left_speed_decel_flag = 0;
volatile uint8 right_speed_decel_flag = 0;
static volatile uint8 speed_control_enabled = 0;

// CCU60_CH1 每 1ms 递增一次；主循环只处理最新节拍，不补跑过期任务。
volatile uint32 main_control_tick = 0;

PidTypeDef left_speed_pid;
PidTypeDef right_speed_pid;

// ==================== 函数声明 ====================

void adc_all_init(void);
void adc_all_read(void);

static int16 limit_pwm_float(float value, int16 min, int16 max);
static void clear_speed_change_flags(void);
static void speed_control_stop_immediate(void);
static void publish_speed_targets(float left_reference_count,
                                  float right_reference_count,
                                  uint8 left_negative_pwm_allowed,
                                  uint8 right_negative_pwm_allowed);
static int32 snapshot_encoder_total_sum(void);

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
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.4f,  0, 0.0f, 120, 0.0f,  50, 0.8f),
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.4f,  0, 0.0f, 120, 0.0f,  50, 0.2f),
        // 二级管，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 1.5f,             0.0f,  0, 0.0f,   0, 0.2f,  50, 0.2f),
    // 左转
    YQJ_CASE(yqj_left_turn_trigger,       1, TARGET_SPEED_MPS,  1.4f,  0, 0.0f, 120, 0.0f,  50, 0.2f),
    // 三极管，直行通过
    YQJ_CASE(yqj_sanjiguan1_2trigger,     0, 2.0f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.5f),
        // 二极管，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 1.5f,             0.0f,  0, 0.0f,   0, 0.0f,  50, 0.4f),
    // 三极管，左转
    YQJ_CASE(yqj_sanjiguan0_1trigger,     1, TARGET_SPEED_MPS,  1.4f, 50, 0.0f,  0, 0.0f,  50, 0.2f),
    // 左弯，直行通过
    YQJ_CASE(yqj_left_turn_trigger,       0, 1.5f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.1f),
    YQJ_CASE(yqj_left_turn_trigger,       0, 1.5f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.1f),
    // 左转
    YQJ_CASE(yqj_left_turn_trigger,       1, TARGET_SPEED_MPS,  1.4f,  0, 0.0f, 10, 0.0f,  33, 0.2f),
    // 二极管，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 2.5f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.2f),
    // 电容，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 1.5f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.2f),
    // 三极管，左转
    YQJ_CASE(yqj_sanjiguan1_2trigger,     1, TARGET_SPEED_MPS,  1.4f,  0, 0.0f, 12, 0.0f,  33, 0.1f),

    // 左转
    YQJ_CASE(yqj_left_turn_trigger,       1, TARGET_SPEED_MPS,  1.4f,  0, 0.0f,  10, 0.0f,  10, 0.2f),
        // 二级管，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 1.5f,             0.0f,  0, 0.0f,   0, 0.0f,  0, 0.4f),
    // 右弯，直行通过
    YQJ_CASE(yqj_right_turn_trigger,      0, 2.0f,             0.0f,  0, 0.0f,   0, 0.1f,  0, 0.2f),
    // 电容，直行通过
    YQJ_CASE(yqj_erjiguan_trigger,        0, 1.5f,             0.0f,  0, 0.0f,   0, 0.0f,  0, 0.2f),
    // 双触发，右转
    YQJ_CASE(yqj_double_trigger,          1, TARGET_SPEED_MPS, -1.4f, 0, 0.0f, 10, 0.0f,  33, 0.2f),
    // 右转
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.3f,  0, 0.0f, 10, 0.0f, 15, 0.7f),
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.3f,  0, 0.0f, 10, 0.0f,  50, 0.2f),
    // 双触发，右转
    YQJ_CASE(yqj_right_turn_trigger,          1, TARGET_SPEED_MPS, -1.4f, 0, 0.0f,  10, 0.0f,  33, 0.6f),
        // 二级管，直行通过
  //  YQJ_CASE(yqj_erjiguan_trigger,        0, 1.5f,             0.0f,  0, 0.0f,   0, 0.0f,  50, 0.2f),
        // 双触发，右转
    YQJ_CASE(yqj_right_turn_trigger,          1, TARGET_SPEED_MPS, -1.4f,  0, 0.0f, 0, 0.0f,  33, 0.5f),
   // 右弯，直行通过
    YQJ_CASE(yqj_right_turn_trigger,      0, 2.2f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.2f),
    // 三极管，直行通过
    YQJ_CASE(yqj_sanjiguan2_0trigger,     0, 1.5f,             0.0f,  0, 0.0f,   10, 0.2f,  50, 0.3f),
    // 右弯，直行通过
    YQJ_CASE(yqj_right_turn_trigger,      0, 1.5f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.1f),
    // 右转
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.4f,  0, 0.0f,  10, 0.0f,  33, 0.7f),
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.4f,  0, 0.0f,  12, 0.0f,  15, 0.4f),
    // 特别二极管，右转
    YQJ_CASE(yqj_tberjiguan_trigger,      1, TARGET_SPEED_MPS, -1.4f,  0, 0.05f,120, 0.0f,  50, 0.4f),
    // 右转
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.4f,  0, 0.0f,  12, 0.0f,  15, 0.3f),
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.4f,  0, 0.0f,  12, 0.0f,  15, 0.4f),
    // 右弯，直行通过
    YQJ_CASE(yqj_right_turn_trigger,      0, 2.3f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.6f),
    YQJ_CASE(yqj_right_turn_trigger,      0, 2.5f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.2f),
    YQJ_CASE(yqj_right_turn_trigger,      0, 2.3f,             0.0f,  0, 0.0f,   0, 0.1f,  50, 0.2f),
    // 右转
    YQJ_CASE(yqj_right_turn_trigger,      1, TARGET_SPEED_MPS, -1.4f,  0, 0.0f,  12, 0.0f,  15, 2.4f),
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

#if CONTROL_DEBUG_PRINT_ENABLE
    uint32 print_count = 0;
#endif
    float final_left_target = BASE_TARGET_COUNT;
    float final_right_target = BASE_TARGET_COUNT;
    int32 encoder_total_sum = 0;
    uint32 processed_control_tick = 0;

    uint8 yqj_condition = 0;
    uint8 yqj_case_trigger = 0;
    uint8 yqj_turn_done = 1;
    uint8 left_negative_pwm_allowed = 0;
    uint8 right_negative_pwm_allowed = 0;
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
             LEFT_SPEED_KP,
             LEFT_SPEED_KI,
             SPEED_KD);

    PID_Init(&right_speed_pid,
             PID_POSITION,
             SPEED_PID_MAX_OUT,
             SPEED_PID_MAX_IOUT,
             RIGHT_SPEED_KP,
             RIGHT_SPEED_KI,
             SPEED_KD);

    cpu_wait_event_ready();

    pwm_init(ATOM0_CH6_P02_6, 100, 1400);
    system_delay_ms(2000);

    // IMU660RC 初始化（240Hz 四元数输出）
    imu660rc_init(IMU660RC_QUARTERNION_240HZ);
   
    turn_control_init(PID_PERIOD_S, ENCODER_COUNT_PER_METER);
    yqj_init(ENCODER_COUNT_PER_METER);

    encoder_clear_count(LEFT_ENCODER);
    encoder_clear_count(RIGHT_ENCODER);
    left_encoder_previous_raw = (uint16)encoder_get_count(LEFT_ENCODER);
    right_encoder_previous_raw = (uint16)encoder_get_count(RIGHT_ENCODER);
    encoder_previous_sample_time = system_getval();
    left_encoder_count = 0;
    right_encoder_count = 0;
    main_control_tick = 0;
    pit_ms_init(SPEED_CONTROL_PIT, PID_PERIOD_MS);
    pit_ms_init(MAIN_CONTROL_PIT, MAIN_CONTROL_PERIOD_MS);
    while (TRUE)
    {
        // 等待下一个硬件 1ms 节拍。若某轮超时，只处理最新节拍，不连续补跑旧任务。
        while (processed_control_tick == main_control_tick)
        {
        }
        processed_control_tick = main_control_tick;

        // ==================== 读取 15 路 ADC ====================

        adc_all_read();
        // ==================== 巡线层 ====================
        // xunji 只根据 ADC 计算普通巡线目标，不处理特殊动作命令。
        // 传入完整 ADC 数组，xunji 直接使用真实下标 [2]~[12]。
        xunji_update(adc_value, BASE_TARGET_COUNT, &line_result);
        encoder_total_sum = snapshot_encoder_total_sum();

        // ==================== 元器件顺序层 ====================
        // 总流程：正常巡线、判断当前 flag、延时、执行动作、自锁、flag 加一。
        yqj_condition = 0;
        yqj_case_trigger = 0;
        left_negative_pwm_allowed = 0;
        right_negative_pwm_allowed = 0;
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

            yqj_condition = case_config->trigger(adc_trigger_value);
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
            speed_control_stop_immediate();
            system_delay_ms(20000);
            continue;
        }

        if (YQJ_STATE_LINE == yqj_state)
        {
            if (yqj_condition)
            {
                yqj_start_case(yqj_case_trigger,
                               encoder_total_sum);
            }
        }
        else if (YQJ_STATE_DELAY == yqj_state)
        {
            if (yqj_delay_done(encoder_total_sum,
                               yqj_delay_ms,
                               yqj_delay_distance_m))
            {
                yqj_turn_done = yqj_get_action_trigger() ? 0u : 1u;
                yqj_start_run(encoder_total_sum);
            }
        }
        else if (YQJ_STATE_RUN == yqj_state)
        {
            uint8 run_conditions_done;

            if (yqj_get_action_trigger() && !yqj_turn_done)
            {
                // 转向由相对 yaw 达到 90 度结束；超时仅用于传感器异常或堵转保护。
                if (turn_control_target_reached() ||
                    yqj_time_reached(yqj_state_start_time,
                                     TURN_CONTROL_TIMEOUT_MS))
                {
                    uint32 turn_stop_interrupt_state;

                    // yaw 转向结束后立即清除转向输出；未完成的 RUN 时间/距离改为同速直行。
                    yqj_turn_done = 1;
                    // 把角度环、速度目标、PID 和实际 PWM 作为一个整体清零，
                    // 避免 2ms 速度 ISR 插入各清零步骤之间。
                    turn_stop_interrupt_state = interrupt_global_disable();
                    turn_control_stop();
                    speed_control_stop_immediate();
                    interrupt_global_enable(turn_stop_interrupt_state);
                }
            }

            run_conditions_done =
                yqj_run_done(encoder_total_sum,
                             yqj_run_ms,
                             yqj_run_distance_m);

            if (yqj_turn_done && run_conditions_done)
            {
                yqj_start_lock(encoder_total_sum);
            }
        }
        else if (YQJ_STATE_LOCK == yqj_state)
        {
            if (yqj_lock_done(encoder_total_sum,
                              yqj_lock_ms,
                              yqj_lock_distance_m))
            {
                clear_speed_change_flags();
                yqj_finish_case();
            }
        }
        else
        {
            yqj_turn_done = 1;
            {
                uint32 invalid_state_interrupt_state;

                invalid_state_interrupt_state = interrupt_global_disable();
                turn_control_stop();
                speed_control_stop_immediate();
                interrupt_global_enable(invalid_state_interrupt_state);
            }
            yqj_set_flag(0);
            continue;
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
                if (yqj_turn_base_speed >= 0.0f)
                {
                    left_negative_pwm_allowed = 1;
                }
                else
                {
                    right_negative_pwm_allowed = 1;
                }
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
        // 高层控制统一按左轮计数标定计算，再原子发布左右速度目标。
        publish_speed_targets(final_left_target,
                              final_right_target,
                              left_negative_pwm_allowed,
                              right_negative_pwm_allowed);

        // ==================== 串口调试 ====================

#if CONTROL_DEBUG_PRINT_ENABLE
        print_count++;
        if (print_count >= CONTROL_DEBUG_PRINT_INTERVAL_LOOPS)
        {
            uint8 i;
            float left_speed_mps = left_encoder_feedback_count /
                                   (ENCODER_COUNT_PER_METER * PID_PERIOD_S);
            float right_speed_mps = right_encoder_feedback_count /
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
                   left_applied_pwm,
                   right_applied_pwm,
                   (int)left_target_count,
                   (int)right_target_count,
                   left_encoder_count,
                   right_encoder_count,
                   left_speed_mps,
                   right_speed_mps);
        }
#endif
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
        // ADC 已经在后台连续转换，每路只读取一次最新结果。
        // 跨帧中值负责剔除尖峰，避免同一结果寄存器被连续重复读取。
        raw_value = adc_convert(adc_list[i]);

        if (!adc_filter_initialized)
        {
            for (j = 0; j < ADC_FILTER_HISTORY_NUM; j++)
            {
                adc_filter_history[i][j] = raw_value;
            }
            adc_filter_output[i] = (float)raw_value;
            adc_trigger_value[i] = raw_value;
            adc_value[i] = raw_value;
        }
        else
        {
            adc_filter_history[i][adc_filter_history_index] = raw_value;
            median_value = adc_history_median(adc_filter_history[i]);

            // 元器件触发不再经过低通，较巡线数据提前约一帧响应。
            adc_trigger_value[i] = median_value;
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

// ==================== 2ms 速度 PID 与电机输出中断 ====================

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    uint16 left_encoder_raw;
    uint16 right_encoder_raw;
    uint32 sample_time;
    uint32 sample_ticks;
    float sample_scale = 1.0f;
    int16 left_pwm_min;
    int16 right_pwm_min;
    int16 next_left_pwm;
    int16 next_right_pwm;

    pit_clear_flag(CCU60_CH0);

    // 在重新开启中断嵌套前完成两路快照，避免高优先级 IMU 中断插在左右读取之间。
    sample_time = system_getval();
    left_encoder_raw = (uint16)encoder_get_count(LEFT_ENCODER);
    right_encoder_raw = (uint16)encoder_get_count(RIGHT_ENCODER);

    // 编码器保持自由运行，通过 16 位模差得到本周期增量。
    // 这样不会丢失“读取后、清零前”到来的脉冲。
    left_encoder_count =
        -(int16)((uint16)(left_encoder_raw - left_encoder_previous_raw));
    right_encoder_count =
        (int16)((uint16)(right_encoder_raw - right_encoder_previous_raw));
    left_encoder_previous_raw = left_encoder_raw;
    right_encoder_previous_raw = right_encoder_raw;

    sample_ticks = (uint32)(sample_time - encoder_previous_sample_time);
    encoder_previous_sample_time = sample_time;

    interrupt_global_enable(0);

    // PIT 可能被 IMU/STM 中断短暂延迟。将实测增量归一化到名义 2ms，
    // 防止一个长采样窗被误认为突然加速，下一短采样窗又被误认为减速。
    if ((sample_ticks >= (PID_PERIOD_10NS_TICKS / 4u)) &&
        (sample_ticks <= (PID_PERIOD_10NS_TICKS * 4u)))
    {
        sample_scale = (float)PID_PERIOD_10NS_TICKS / (float)sample_ticks;
    }

    left_encoder_feedback_count =
        (float)left_encoder_count * sample_scale;
    right_encoder_feedback_count =
        (float)right_encoder_count * sample_scale;

    // ==================== 电机超速保护 ====================
    // 使用编码器实测速度而不是目标速度；正转、反转均按绝对值判断。
    // 单周期内任一编码器计数超过配置速度阈值，立即锁存并关闭两侧电机。
    if((left_encoder_feedback_count > LEFT_MOTOR_MAX_SAFE_COUNT) ||
       (left_encoder_feedback_count < -LEFT_MOTOR_MAX_SAFE_COUNT) ||
       (right_encoder_feedback_count > RIGHT_MOTOR_MAX_SAFE_COUNT) ||
       (right_encoder_feedback_count < -RIGHT_MOTOR_MAX_SAFE_COUNT))
    {
        left_target_count = 0.0f;
        right_target_count = 0.0f;
        left_base_pwm = 0.0f;
        right_base_pwm = 0.0f;
        PID_clear(&left_speed_pid);
        PID_clear(&right_speed_pid);
        left_applied_pwm = 0;
        right_applied_pwm = 0;
        speed_control_enabled = 0;
        motor_emergency_stop();
        return;
    }

    // 已触发的故障禁止速度环继续运算，急停锁存只能通过重新上电清除。
    if(motor_emergency_is_latched())
    {
        left_base_pwm = 0.0f;
        right_base_pwm = 0.0f;
        left_applied_pwm = 0;
        right_applied_pwm = 0;
        speed_control_enabled = 0;
        return;
    }

    // ==================== 软件累计总路程 ====================

    left_encoder_total += left_encoder_count;
    right_encoder_total += right_encoder_count;

    car_distance_m = 0.5f *
                     ((float)left_encoder_total / ENCODER_COUNT_PER_METER +
                      (float)right_encoder_total / RIGHT_ENCODER_COUNT_PER_METER);

    // 停车状态不运行PID，避免编码器回弹或噪声把零目标重新变成正PWM。
    if (!speed_control_enabled)
    {
        left_base_pwm = 0.0f;
        right_base_pwm = 0.0f;
        left_applied_pwm = 0;
        right_applied_pwm = 0;
        PID_clear(&left_speed_pid);
        PID_clear(&right_speed_pid);
        motor_control(0, 0);
        return;
    }

    // ==================== 速度 PID ====================
    // PID_Calc(pid, 实际值, 目标值)

    left_base_pwm = PID_Calc(&left_speed_pid,
                             left_encoder_feedback_count,
                             left_target_count);

    right_base_pwm = PID_Calc(&right_speed_pid,
                              right_encoder_feedback_count,
                              right_target_count);

    // ==================== 前馈控制 ====================
    // 根据目标速度直接给基础 PWM，减小 PID 负担。
    // 前馈 = FEEDFORWARD_GAIN * target_speed
    // target_speed = target_count / (PID_PERIOD_S * ENCODER_COUNT_PER_METER)
    left_base_pwm += FEEDFORWARD_GAIN * left_target_count / (PID_PERIOD_S * ENCODER_COUNT_PER_METER);
    right_base_pwm += FEEDFORWARD_GAIN * right_target_count /
                      (PID_PERIOD_S * RIGHT_ENCODER_COUNT_PER_METER);

    // PID计算完成后立即下发PWM，省去等待下一轮主循环的0~1ms延迟。
    // 正常巡线禁止负PWM；只有对应转向内侧轮被明确授权时才允许反转。
    left_pwm_min = left_speed_decel_flag ? -PWM_DUTY_MAX : 0;
    right_pwm_min = right_speed_decel_flag ? -PWM_DUTY_MAX : 0;
    next_left_pwm = limit_pwm_float(left_base_pwm, left_pwm_min, PWM_DUTY_MAX);
    next_right_pwm = limit_pwm_float(right_base_pwm, right_pwm_min, PWM_DUTY_MAX);
    left_applied_pwm = next_left_pwm;
    right_applied_pwm = next_right_pwm;
    motor_control(next_left_pwm, next_right_pwm);
}

// 在转换为 int16 前先按 float 限幅，避免极端目标使强制转换溢出并反向。
static int16 limit_pwm_float(float value, int16 min, int16 max)
{
    if (value > (float)max)
    {
        return max;
    }
    else if (value < (float)min)
    {
        return min;
    }
    else
    {
        return (int16)value;
    }
}

// 立即停止速度环和两侧电机；可安全嵌套在外层临界区中。
static void speed_control_stop_immediate(void)
{
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    left_target_count = 0.0f;
    right_target_count = 0.0f;
    left_base_pwm = 0.0f;
    right_base_pwm = 0.0f;
    left_applied_pwm = 0;
    right_applied_pwm = 0;
    left_speed_decel_flag = 0;
    right_speed_decel_flag = 0;
    speed_control_enabled = 0;
    PID_clear(&left_speed_pid);
    PID_clear(&right_speed_pid);
    motor_control(0, 0);
    interrupt_global_enable(interrupt_state);
}

// 左右目标和反转权限必须作为一组提交，避免速度中断读到半套命令。
static void publish_speed_targets(float left_reference_count,
                                  float right_reference_count,
                                  uint8 left_negative_pwm_allowed,
                                  uint8 right_negative_pwm_allowed)
{
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    left_target_count = left_reference_count;
    right_target_count =
        right_reference_count * RIGHT_ENCODER_COUNT_SCALE;
    left_speed_decel_flag = left_negative_pwm_allowed;
    right_speed_decel_flag = right_negative_pwm_allowed;
    speed_control_enabled = 1;
    interrupt_global_enable(interrupt_state);
}

static void clear_speed_change_flags(void)
{
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    left_speed_decel_flag = 0;
    right_speed_decel_flag = 0;
    interrupt_global_enable(interrupt_state);
}

// 元器件状态机每轮只使用一次一致的左右总里程快照。
static int32 snapshot_encoder_total_sum(void)
{
    uint32 interrupt_state;
    int32 total_sum;

    interrupt_state = interrupt_global_disable();
    total_sum = left_encoder_total + right_encoder_total;
    interrupt_global_enable(interrupt_state);

    return total_sum;
}

#pragma section all restore
