#include "zf_common_headfile.h"
#include "isr_config.h"
#include "device.h"
#include "pid.h"
#include "../code/yqj.h"
#include "../code/xunji.h"

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

// 实测（强磁电机）：小车走 0.5 米约 27000 个编码器计数
// 1 米 = 27000 * 2 = 54000
#define ENCODER_COUNT_PER_METER (54000.0f)

// 目标基础速度 1.0 m/s
#define TARGET_SPEED_MPS (1.0f)

// PID 周期 20ms
#define PID_PERIOD_MS (20)
#define PID_PERIOD_S (0.02f)

// 20ms 内基础目标计数 = TARGET_SPEED_MPS * 0.02 * ENCODER_COUNT_PER_METER
#define BASE_TARGET_COUNT (TARGET_SPEED_MPS * PID_PERIOD_S * ENCODER_COUNT_PER_METER)

// PID 输出范围
#define SPEED_PID_MAX_OUT (8000.0f)
#define SPEED_PID_MAX_IOUT (2000.0f)

// 速度 PID 参数
#define SPEED_KP (2.5f)
#define SPEED_KI (0.02f)
#define SPEED_KD (0.0f)

// 前馈系数：PWM = FEEDFORWARD_GAIN * target_speed
#define FEEDFORWARD_GAIN (500.0f)

// Startup soft-start parameters.
#define MAIN_LOOP_PERIOD_MS (2)
#define TARGET_RAMP_ACCEL_MPS2 (2.0f)
#define TARGET_RAMP_STEP_COUNT (TARGET_RAMP_ACCEL_MPS2 * ((float)MAIN_LOOP_PERIOD_MS / 1000.0f) * PID_PERIOD_S * ENCODER_COUNT_PER_METER)
#define TARGET_START_EPS_COUNT (1.0f)
#define STARTUP_PWM_STEP (10)

// ==================== 巡线参数 ====================

#define SENSOR_NUM (XUNJI_SENSOR_NUM)

// ==================== ADC 变量 ====================

// 从左到右：A1 A2 A3 A4 A5 A6 A7 A8 A10 A11
uint16 adc_value[SENSOR_NUM];

adc_channel_enum adc_list[SENSOR_NUM] =
    {
        ADC0_CH1_A1,
        ADC0_CH2_A2,
        ADC0_CH3_A3,
        ADC0_CH4_A4,
        ADC0_CH5_A5,
        ADC0_CH6_A6,
        ADC0_CH7_A7,
        ADC0_CH8_A8,
        ADC0_CH10_A10,
        ADC0_CH11_A11};

// ==================== 编码器与 PID 变量 ====================

// 20ms 内编码器增量，用于速度 PID
volatile int16 left_encoder_count = 0;
volatile int16 right_encoder_count = 0;

// 软件累计总计数，用于计算总路程
volatile int32 left_encoder_total = 0;
volatile int32 right_encoder_total = 0;

// 小车总路程，单位 m
volatile float car_distance_m = 0.5f;

// 巡线算出的左右目标编码器计数
volatile float left_target_count = BASE_TARGET_COUNT;
volatile float right_target_count = BASE_TARGET_COUNT;

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
float ramp_target_count(float current, float target);
int16 ramp_pwm_int16(int16 current, int16 target, int16 step);
void set_speed_change_flags(float next_left_target,
                            float next_right_target,
                            float current_left_target,
                            float current_right_target);

// ==================== 主函数 ====================

int core0_main(void)
{
    xunji_result_struct line_result = {0, 0, BASE_TARGET_COUNT, BASE_TARGET_COUNT};

    int16 left_pwm = 0;
    int16 right_pwm = 0;
    int16 target_left_pwm = 0;
    int16 target_right_pwm = 0;
    int16 left_pwm_min = 0;
    int16 right_pwm_min = 0;

    uint32 print_count = 0;
    float final_left_target = BASE_TARGET_COUNT;
    float final_right_target = BASE_TARGET_COUNT;
    float ramped_left_target = 0.0f;
    float ramped_right_target = 0.0f;
    float action_left_target = BASE_TARGET_COUNT;
    float action_right_target = BASE_TARGET_COUNT;
    int16 startup_left_pwm = 0;
    int16 startup_right_pwm = 0;
    uint8 startup_ramp_done = 0;
    uint8 target_ramp_reached = 0;

    uint8 yqj_condition = 0;
    uint8 yqj_case_trigger = 0;
    uint32 yqj_delay_ms = 0;
    uint32 yqj_run_ms = 0;
    uint32 yqj_lock_ms = 0;
    float yqj_lock_distance_m = 0.0f;
    float yqj_left_speed_mps = 0.0f;
    float yqj_right_speed_mps = 0.0f;

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

    pwm_init(ATOM0_CH6_P02_6, 100, 1500);
    system_delay_ms(2000);
   
    yqj_init(PID_PERIOD_S, ENCODER_COUNT_PER_METER);
        
    pit_ms_init(PIT0, PID_PERIOD_MS);
    while (TRUE)
    {
        // ==================== 读取 10 路 ADC ====================

        adc_all_read();
        // ==================== 巡线层 ====================
        // xunji 只根据 ADC 计算普通巡线目标，不处理特殊动作命令。
        xunji_update(adc_value, BASE_TARGET_COUNT, &line_result);

        // ==================== 元器件顺序层 ====================
        // 总流程：正常巡线、判断当前 flag、延时、执行动作、自锁、flag 加一。
        yqj_condition = 0;
        yqj_case_trigger = 0;
        yqj_delay_ms = 0;
        yqj_run_ms = 0;
        yqj_lock_ms = 0;
        yqj_lock_distance_m = 0.0f;
        yqj_left_speed_mps = 0.0f;
        yqj_right_speed_mps = 0.0f;

        final_left_target = line_result.left_target_count;
        final_right_target = line_result.right_target_count;

        switch (yqj_flag)
        {
      /*se 1:
            // 开关
            yqj_condition = yqj_kaiguang0_1trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.5f;
            break;*/
        case 2:
            // 电源
            yqj_condition = yqj_dianyuan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.5f;
            yqj_right_speed_mps = 1.5f; 
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.5f;
            break;
        case 1:
            // 右转
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 3.5f       ;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 120;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        case 4:
            // 电阻
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        case 5:
            // 左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 1.5f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.4f;
            break;
        case 6:
            // 左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 1.5f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.4f;
            break;
        case 7:
            // 二级管
            yqj_condition = yqj_erjiguan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 100;
            yqj_run_ms = 350;
            yqj_lock_ms = 66;
            yqj_lock_distance_m = 0.4f;
            break;
        case 8:
            // 三极管
            yqj_condition = yqj_sanjiguan0_1trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 100;
            yqj_run_ms = 350;
            yqj_lock_ms = 66;
            yqj_lock_distance_m = 0.4f;
            break;
        case 9:
            // 二级管
            yqj_condition = yqj_erjiguan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 100;
            yqj_run_ms = 350;
            yqj_lock_ms = 66;
            yqj_lock_distance_m = 0.4f;
            break;
        case 10:
            // 开关
            yqj_condition = yqj_kaiguang1_0trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.5f;
            break;
        case 11:
            // 右转
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 1.0f;
            break;
        case 12:
            // 右转
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 66;
            yqj_lock_distance_m = 0.6f;
            break;
        case 13:
            // 三极管
            yqj_condition = yqj_sanjiguan0_1trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 1.5f;
            yqj_delay_ms = 100;
            yqj_run_ms = 350;
            yqj_lock_ms = 66;
            yqj_lock_distance_m = 0.4f;
            break;
        case 14:
            // 左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 1.5f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.4f;
            break;
        case 15:
            // 又转
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 66;
            yqj_lock_distance_m = 0.6f;
            break;
        case 16:
            // 电感
            yqj_condition = yqj_ldiangan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 350;
            yqj_lock_ms = 1;
            yqj_lock_distance_m = 0.1f;
            break;
        case 17:
            // 电源
            yqj_condition = yqj_dianyuan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.5f;
            break;
        case 18:
            // 线圈
            yqj_xianquan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 350;
            yqj_lock_ms = 1;
            yqj_lock_distance_m = 0.1f;
            break;
        case 19:
            // 左转
            yqj_condition = yqj_double_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 1.5f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.1f;
            break;
        case 20:
            // 左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 1.5f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.4f;
            break;
        case 21:
            // 开关
            yqj_condition = yqj_kaiguang0_1trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.5f;
            break;
        case 22:
            //二极管
            yqj_condition = yqj_erji_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.5f;
            break;
        case 23:
            // 二极管
            yqj_condition = yqj_erjiguan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 1.0f;
            break;
        case 24:
            // 左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 1.5f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.4f;
            break;
        case 25:
            // 电池
            yqj_dianchi_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 350;
            yqj_lock_ms = 1;
            yqj_lock_distance_m = 0.1f;
            break;
        case 26:
            //电阻
            yqj_condition = yqj_dianzu_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 350;
            yqj_lock_ms = 1;
            yqj_lock_distance_m = 0.1f;
            break;
        case 27:
            // 电容
            yqj_condition = yqj_dianrong_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 350;
            yqj_lock_ms = 1;
            yqj_lock_distance_m = 0.1f;
            break;
        case 28:
            //左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 1.5f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.4f;
            break;
        case 29:
        
            //左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 1.5f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.4f;
            break;
        case 30:
            // 电感
            yqj_condition = yqj_ldiangan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 350;
            yqj_lock_ms = 1;
            yqj_lock_distance_m = 1.0f;
            break;
        case 31:        
            //非门
            yqj_condition = yqj_feimen_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 350;
            yqj_lock_ms = 1;
            yqj_lock_distance_m = 0.5f;
            break;
        case 32:    
            //右转
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 66;
            yqj_lock_distance_m = 1.2f;
            break;
        case 33:
            //电容
            yqj_condition = yqj_dianrong_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 350;
            yqj_lock_ms = 1;
            yqj_lock_distance_m = 0.5f;
            break;
        case 34:
            // 电感
            yqj_condition = yqj_ldiangan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 350;
            yqj_lock_ms = 1;
            yqj_lock_distance_m = 0.1f;
            break;
        case 35:
            // 电源
            yqj_condition = yqj_dianyuan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.5f;
            break;
        case 36:
            // 线圈
            yqj_xianquan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 350;
            yqj_lock_ms = 1;
            yqj_lock_distance_m = 0.1f;
            break;
        case 37:
            //右转
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 400;
            yqj_lock_ms = 66;
            yqj_lock_distance_m = 1.2f;
            break;
        case 38:
            // 开关
            yqj_condition = yqj_kaiguang0_1trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.5f;
            break;
        case 39:
            // 电源
            yqj_condition = yqj_dianyuan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 1.0f;
            break;
            //    
            //             case 2:
            //                             右转
            //                             yqj_condition = yqj_right_turn_trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 1.5f;
            //                             yqj_right_speed_mps = 0.0f;
            //                             yqj_delay_ms = 0;
            //                             yqj_run_ms = 400;
            //                             yqj_lock_ms = 33;
            //                             yqj_lock_distance_m = 0.2f;
            //                             break;
            //             case 3:
            //                 右转
            //                 yqj_condition = yqj_right_turn_trigger(adc_value);
            //                 yqj_case_trigger = 1;
            //                 yqj_left_speed_mps = 1.5f;
            //                 yqj_right_speed_mps = 0.0f;
            //                 yqj_delay_ms = 0;
            //                 yqj_run_ms = 400;
            //                 yqj_lock_ms = 66;
            //                 yqj_lock_distance_m = 0.6f;
            //                 break;

            // //            case 4:
            // //                // 电阻
            // //                yqj_condition = yqj_dianzu_trigger(adc_value);
            // //                yqj_case_trigger = 1;
            // //                yqj_left_speed_mps = 0.3f;
            // //                yqj_right_speed_mps = 0.3f;
            // //                yqj_delay_ms = 0;
            // //                yqj_run_ms = 350;
            // //                yqj_lock_ms = 1;
            // //                yqj_lock_distance_m = 0.1f;
            // //                break;
            //             case 4:
            //                 左转
            //                 yqj_condition = yqj_left_turn_trigger(adc_value);
            //                 yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 0.0f;
            //                 yqj_right_speed_mps = 1.5f;
            //                 yqj_delay_ms = 0;
            //                             yqj_run_ms = 400;
            //                 yqj_lock_ms = 33;
            //                 yqj_lock_distance_m = 0.4f;
            //                 break;

            //             case 5:
            //                             左转
            //                             yqj_condition = yqj_double_trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 0.0f;
            //                             yqj_right_speed_mps = 1.5f;
            //                             yqj_delay_ms = 0;
            //                             yqj_run_ms = 400;
            //                             yqj_lock_ms = 33;
            //                             yqj_lock_distance_m = 0.1f;
            //                             break;

            //             case 6:
            //                             三极管
            //                             yqj_condition = yqj_sanjiguan0_1trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 1.5f;
            //                             yqj_right_speed_mps = 0.0f;
            //                             yqj_delay_ms = 100;
            //                             yqj_run_ms = 350;
            //                             yqj_lock_ms = 66;
            //                             yqj_lock_distance_m = 0.4f;
            //                             break;
            //             case 7:
            //                             右转
            //                             yqj_condition = yqj_right_turn_trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 1.5f;
            //                             yqj_right_speed_mps = 0.0f;
            //                             yqj_delay_ms = 0;
            //                             yqj_run_ms = 400;
            //                             yqj_lock_ms = 66;
            //                             yqj_lock_distance_m = 1.2f;
            //                             break;

            // //            case 8:
            // //                            //电阻
            // //                            yqj_condition = yqj_dianzu_trigger(adc_value);
            // //                            yqj_case_trigger = 1;
            // //                            yqj_left_speed_mps = 0.3f;
            // //                            yqj_right_speed_mps = 0.3f;
            // //                            yqj_delay_ms = 0;
            // //                            yqj_run_ms = 350;
            // //                            yqj_lock_ms = 200;
            // //                            yqj_lock_distance_m = 0.1f;
            // //                            break;

            //             case 8:
            //                             右转
            //                             yqj_condition = yqj_right_turn_trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 1.5f;
            //                             yqj_right_speed_mps = 0.0f;
            //                             yqj_delay_ms = 0;
            //                             yqj_run_ms = 400;
            //                             yqj_lock_ms = 66;
            //                             yqj_lock_distance_m = 0.1f;
            //                             break;
            //             case 9:
            //                             三极管
            //                             yqj_condition = yqj_sanjiguan0_1trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 0.0f;
            //                             yqj_right_speed_mps = 1.5f;
            //                             yqj_delay_ms = 100;
            //                             yqj_run_ms = 300;
            //                             yqj_lock_ms = 66;
            //                             yqj_lock_distance_m = 0.4f;
            //                             break;

            //             case 10:
            //                             左转
            //                             yqj_condition = yqj_double_trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 0.0f;
            //                             yqj_right_speed_mps = 1.5f;
            //                             yqj_delay_ms = 0;
            //                             yqj_run_ms = 400;
            //                             yqj_lock_ms = 66;
            //                             yqj_lock_distance_m = 0.3f;
            //                             break;
            //             case 11:
            //                             左转
            //                             yqj_condition = yqj_left_turn_trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 0.0f;
            //                             yqj_right_speed_mps = 1.5f;
            //                             yqj_delay_ms = 0;
            //                             yqj_run_ms = 350;
            //                             yqj_lock_ms = 66;
            //                             yqj_lock_distance_m = 2.0f;
            //                             break;
            // //            case 12:
            // //                                // 电阻
            // //                                yqj_condition = yqj_dianzu_trigger(adc_value);
            // //                                yqj_case_trigger = 1;
            // //                                yqj_left_speed_mps = 0.3f;
            // //                                yqj_right_speed_mps = 0.3f;
            // //                                yqj_delay_ms = 0;
            // //                                yqj_run_ms = 350;
            // //                                yqj_lock_ms = 200;
            // //                                yqj_lock_distance_m = 0.1f;
            // //                                break;
            // //            case 13:
            // //                            // 左拐
            // //                            yqj_condition = yqj_left_turn_trigger(adc_value);
            // //                            yqj_case_trigger = 1;
            // //                            yqj_left_speed_mps = 0.3f;
            // //                            yqj_right_speed_mps = 0.3f;
            // //                            yqj_delay_ms = 0;
            // //                            yqj_run_ms = 300;
            // //                            yqj_lock_ms = 200;
            // //                            yqj_lock_distance_m = 0.1f;
            // //                            break;
            // //            case 14:
            // //                            //电阻
            // //                            yqj_condition = yqj_dianzu_trigger(adc_value);
            // //                            yqj_case_trigger = 1;
            // //                            yqj_left_speed_mps = 0.3f;
            // //                            yqj_right_speed_mps = 0.3f;
            // //                            yqj_delay_ms = 0;
            // //                            yqj_run_ms = 350;
            // //                            yqj_lock_ms = 200;
            // //                            yqj_lock_distance_m = 0.1f;
            // //                            break;
            //             支角弯左转：左边检测到白线，右边没有检测到白线。
            //                             yqj_condition=yqj_double_trigger(adc_value);
            //                             yqj_case_trigger=1;
            //                             yqj_left_speed_mps=1.5f;
            //                             yqj_right_speed_mps=0.0f;
            //                             yqj_delay_ms=0;
            //                             yqj_run_ms=400;
            //                             yqj_lock_ms=66;
            //                             yqj_lock_distance_m=0.6f;
            //                             break;
            // //            case 16 :
            // //                               //电阻
            // //                                yqj_condition = yqj_dianzu_trigger(adc_value);
            // //                                yqj_case_trigger = 1;
            // //                                yqj_left_speed_mps = 0.3f;
            // //                                yqj_right_speed_mps = 0.3f;
            // //                                yqj_delay_ms = 0;
            // //                                yqj_run_ms = 350;
            // //                                yqj_lock_ms = 200;
            // //                                yqj_lock_distance_m = 0.1f;
            // //                                break;
            //             case 13:
            //                            右转
            //                             yqj_condition = yqj_right_turn_trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 1.5f;
            //                             yqj_right_speed_mps = 0.0f;
            //                             yqj_delay_ms = 0;
            //                             yqj_run_ms = 400;
            //                             yqj_lock_ms = 66;
            //                             yqj_lock_distance_m = 0.1f;
            //                             break;

            //             case 14:
            //                             // 二极管
            //                             二极管
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 0.8f;
            //                             yqj_right_speed_mps = 0.8f;
            //                             yqj_delay_ms = 0;
            //                             yqj_run_ms = 500;
            //                             yqj_lock_ms = 66;
            //                             yqj_lock_distance_m = 0.1f;
            //                             break;

            //             case 15:
            //                             右转
            //                             yqj_condition = yqj_right_turn_trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 1.5f;
            //                             yqj_right_speed_mps = 0.0f;
            //                             yqj_delay_ms = 0;
            //                             yqj_run_ms = 400;
            //                             yqj_lock_ms = 66;
            //                             yqj_lock_distance_m = 0.8f;
            //                             break;
            //             case 16:
            //                             // 开关_1
            //                             yqj_condition = yqj_kaiguang0_1trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 0.8f;
            //                             yqj_right_speed_mps = 0.8f;
            //                             yqj_delay_ms = 0;
            //                             yqj_run_ms = 500;
            //                             yqj_lock_ms = 66;
            //                             yqj_lock_distance_m = 0.4f;
            //                             break;

            //             case 17:
            //                             右转
            //                             yqj_condition = yqj_right_turn_trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 1.5f;
            //                             yqj_right_speed_mps = 0.0f;
            //                             yqj_delay_ms = 0;
            //                             yqj_run_ms = 400;
            //                             yqj_lock_ms = 66;
            //                             yqj_lock_distance_m = 0.1f;
            //                             break;

            //             case 18:
            //                             // 电源
            //                             yqj_condition = yqj_dianyuan_trigger(adc_value);
            //                             yqj_case_trigger = 1;
            //                             yqj_left_speed_mps = 0.6f;
            //                             yqj_right_speed_mps = 0.6f;
            //                             yqj_delay_ms = 0;
            //                             yqj_run_ms = 500;
            //                             yqj_lock_ms = 66;
            //                             yqj_lock_distance_m = 0.1f;
            //                             break;

        default:
            // 停止
            //这里是总流程：正常巡线、判断当前 flag、延时、执行动作、自锁、flag 加一。
            motor_stop();
            system_delay_ms(20000);
            break;
        }

        if (YQJ_STATE_LINE == yqj_state)
        {
            if (yqj_condition)
            {
                yqj_start_case(yqj_case_trigger);
            }
        }
        else if (YQJ_STATE_DELAY == yqj_state)
        {
            if (yqj_time_reached(yqj_state_start_time, yqj_delay_ms))
            {
                if (yqj_get_action_trigger())
                {
                    action_left_target = yqj_left_speed_mps * PID_PERIOD_S * ENCODER_COUNT_PER_METER;
                    action_right_target = yqj_right_speed_mps * PID_PERIOD_S * ENCODER_COUNT_PER_METER;
                    set_speed_change_flags(action_left_target,
                                           action_right_target,
                                           final_left_target,
                                           final_right_target);
                }

                yqj_state = YQJ_STATE_RUN;
                yqj_state_start_time = system_getval();
            }
        }
        else if (YQJ_STATE_RUN == yqj_state)
        {
            if (yqj_time_reached(yqj_state_start_time, yqj_run_ms))
            {
                if (yqj_get_action_trigger())
                {
                    set_speed_change_flags(final_left_target,
                                           final_right_target,
                                           left_target_count,
                                           right_target_count);
                }

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
            yqj_set_flag(0);
        }

        if (YQJ_STATE_RUN == yqj_state)
        {
            yqj_apply_action(yqj_left_speed_mps, yqj_right_speed_mps, &final_left_target, &final_right_target);
        }

        target_ramp_reached = 1;
        if (!startup_ramp_done)
        {
            target_ramp_reached = 0;
            ramped_left_target = ramp_target_count(ramped_left_target, final_left_target);
            ramped_right_target = ramp_target_count(ramped_right_target, final_right_target);

            left_target_count = ramped_left_target;
            right_target_count = ramped_right_target;

            if (ramped_left_target == final_left_target &&
                ramped_right_target == final_right_target)
            {
                target_ramp_reached = 1;
            }
        }
        else
        {
            ramped_left_target = final_left_target;
            ramped_right_target = final_right_target;

            left_target_count = final_left_target;
            right_target_count = final_right_target;
        }

        if (!startup_ramp_done)
        {
            left_speed_decel_flag = 0;
            right_speed_decel_flag = 0;
        }

        left_pwm_min = left_speed_decel_flag ? -PWM_DUTY_MAX : 0;
        right_pwm_min = right_speed_decel_flag ? -PWM_DUTY_MAX : 0;

        // ==================== 输出电机 ====================
        // PID 在 20ms 中断里根据左右目标计数输出 PWM

        if (!startup_ramp_done)
        {
            target_left_pwm = limit_int16((int16)left_base_pwm, left_pwm_min, PWM_DUTY_MAX);
            target_right_pwm = limit_int16((int16)right_base_pwm, right_pwm_min, PWM_DUTY_MAX);

            startup_left_pwm = ramp_pwm_int16(startup_left_pwm, target_left_pwm, STARTUP_PWM_STEP);
            startup_right_pwm = ramp_pwm_int16(startup_right_pwm, target_right_pwm, STARTUP_PWM_STEP);

            left_pwm = startup_left_pwm;
            right_pwm = startup_right_pwm;

            if (target_ramp_reached &&
                startup_left_pwm == target_left_pwm &&
                startup_right_pwm == target_right_pwm)
            {
                startup_ramp_done = 1;
            }
        }
        else
        {
            left_pwm = limit_int16((int16)left_base_pwm, left_pwm_min, PWM_DUTY_MAX);
            right_pwm = limit_int16((int16)right_base_pwm, right_pwm_min, PWM_DUTY_MAX);
        }

        motor_control(left_pwm, right_pwm);

        // ==================== 串口调试 ====================

        print_count++;
        if (print_count >= 50)
        {
            uint8 i;
            float left_speed_mps = (float)left_encoder_count /
                                   (ENCODER_COUNT_PER_METER * PID_PERIOD_S);
            float right_speed_mps = (float)right_encoder_count /
                                    (ENCODER_COUNT_PER_METER * PID_PERIOD_S);

            print_count = 0;

            printf("ADC:");
            for (i = 0; i < SENSOR_NUM; i++)
            {
                printf(" %5d", adc_value[i]);
            }
            printf("\r\n");

            printf("flag=%2d pwmL=%5d pwmR=%5d targetL=%5d targetR=%5d encL=%5d encR=%5d speedL=%5.3f speedR=%5.3f\r\n",
                   yqj_flag,
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

void adc_all_read(void)
{
    uint8 i;
    for (i = 0; i < SENSOR_NUM; i++)
    {
        // 10 路全部用于循迹，3 次平均兼顾稳定和响应速度。
        adc_value[i] = adc_mean_filter_convert(adc_list[i], 3);
    }
}

// ==================== 20ms 速度 PID 中断 ====================

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);

    // ==================== 读取 20ms 内编码器增量 ====================
    // 左编码器前进时是负数，所以取反变成正数。
    left_encoder_count = -encoder_get_count(LEFT_ENCODER);

    // 右编码器前进时是正数，直接读取。
    right_encoder_count = encoder_get_count(RIGHT_ENCODER);

    // ==================== 读完立刻清空硬件编码器 ====================

    encoder_clear_count(LEFT_ENCODER);
    encoder_clear_count(RIGHT_ENCODER);

    // ==================== 保留：如需只看速度大小，可在这里取绝对值 ====================

    /*if(left_encoder_count < 0)
    {
        left_encoder_count = -left_encoder_count;
    }

    if(right_encoder_count < 0)
    {
        right_encoder_count = -right_encoder_count;
    }*/

    // ==================== 软件累计总路程 ====================

    left_encoder_total += left_encoder_count;
    right_encoder_total += right_encoder_count;

    car_distance_m = ((float)left_encoder_total + (float)right_encoder_total) /
                     (2.0f * ENCODER_COUNT_PER_METER);

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
    right_base_pwm += FEEDFORWARD_GAIN * right_target_count / (PID_PERIOD_S * ENCODER_COUNT_PER_METER);
}

// ==================== 限幅函数 ====================

float ramp_target_count(float current, float target)
{
    if (target <= TARGET_START_EPS_COUNT)
    {
        return 0.0f;
    }

    if (current < target)
    {
        current += TARGET_RAMP_STEP_COUNT;
        if (current > target)
        {
            current = target;
        }
        return current;
    }

    return target;
}

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

int16 ramp_pwm_int16(int16 current, int16 target, int16 step)
{
    if (current < target)
    {
        current += step;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= step;
        if (current < target)
        {
            current = target;
        }
    }

    return current;
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
