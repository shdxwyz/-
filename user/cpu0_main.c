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

// 任一电机实测速度绝对值超过该值时，锁存急停并关闭全部电机。
#define MOTOR_MAX_SAFE_SPEED_MPS (5.0f)
#define MOTOR_MAX_SAFE_COUNT \
    (MOTOR_MAX_SAFE_SPEED_MPS * ENCODER_COUNT_PER_METER * PID_PERIOD_S)

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

// 主循环周期
#define MAIN_LOOP_PERIOD_MS (2)

// ==================== 巡线参数 ====================

#define SENSOR_NUM (XUNJI_SENSOR_TOTAL)

// ==================== ADC 变量 ====================

// 从左到右：A0 A1 A2 A3 A4 A5 A6 A7 A8 A10 A11 A12 A13 A16 A17
// A0 A1 和 A16 A17 不用于巡线，用于转弯操作
uint16 adc_value[SENSOR_NUM];

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

// 20ms 内编码器增量，用于速度 PID
volatile int16 left_encoder_count = 0;
volatile int16 right_encoder_count = 0;

// 软件累计总计数，用于计算总路程
volatile int32 left_encoder_total = 0;
volatile int32 right_encoder_total = 0;

// 小车总路程，单位 m
volatile float car_distance_m = 1.5f;

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
    int16 left_pwm_min = 0;
    int16 right_pwm_min = 0;

    uint32 print_count = 0;
    float final_left_target = BASE_TARGET_COUNT;
    float final_right_target = BASE_TARGET_COUNT;
    float action_left_target = BASE_TARGET_COUNT;
    float action_right_target = BASE_TARGET_COUNT;

    uint8 yqj_condition = 0;
    uint8 yqj_case_trigger = 0;
    uint32 yqj_delay_ms = 0;
    float yqj_delay_distance_m = 0.0f;
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
        yqj_left_speed_mps = 0.0f;
        yqj_right_speed_mps = 0.0f;

        final_left_target = line_result.left_target_count;
        final_right_target = line_result.right_target_count;

        // 每个 case 可分别调整左右轮动作速度、动作前延迟时间/距离、
        // 动作执行时间，以及动作后的自锁时间/距离。
        switch (yqj_flag)
        {
      /* kaiguang case disabled
      case 1:
            // 开关
            yqj_condition = yqj_kaiguang0_1trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.2f;
            break;
      */
        case 1:
            // 电源方案（当前禁用）
           /*yqj_condition = yqj_dianyuan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 66;
            yqj_lock_distance_m = 0.2f;
            break;*/
            // 当前 case 1：左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 3.5f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 0;
            yqj_lock_distance_m = 0.0f;
            break;
        case 200:
            // 巡线走 1m
            yqj_condition = 1;
            yqj_case_trigger = 0;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 0;
            yqj_lock_ms = 0;
            yqj_lock_distance_m = 0.0f;
            break;
        case 300:
                    // 右转
                    yqj_condition = yqj_right_turn_trigger(adc_value);
                    yqj_case_trigger = 1;
                    yqj_left_speed_mps = 3.5f;
                    yqj_right_speed_mps = 0.0f;
                    yqj_delay_ms = 0;
                    yqj_delay_distance_m = 0.0f;
                    yqj_run_ms = 120;
                    yqj_lock_ms = 150;
                    yqj_lock_distance_m = 0.4f;
                    break;
        case 4:
            // 电阻
            yqj_condition = yqj_dianzu_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        case 5:
            // 左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 3.5f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
        case 6:
            // 左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 3.5f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
        case 7:
            // 二级管
            yqj_condition = yqj_erjiguan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        case 8:
            // 三极管
            yqj_condition = yqj_sanjiguan0_1trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 3.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 50;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        case 9:
            // 二级管
            yqj_condition = yqj_erjiguan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 100;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        case 10:
            // 线圈电阻
            yqj_condition = yqj_xianquandianzu_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.8f;
            yqj_right_speed_mps = 0.8f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.4f;
            break;
        case 11:
            // 右转
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 3.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
        case 12:
                   // 左电感
                   yqj_condition = yqj_ldiangan_trigger(adc_value);
                   yqj_case_trigger = 1;
                   yqj_left_speed_mps = 1.0f;
                   yqj_right_speed_mps = 1.0f;
                   yqj_delay_ms = 0;
                   yqj_delay_distance_m = 0.0f;
                   yqj_run_ms = 80;
                   yqj_lock_ms = 120;
                   yqj_lock_distance_m = 0.4f;
                   break;
        case 13:
            // 右转
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 3.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
        case 14:
            // 三极管
            yqj_condition = yqj_sanjiguan0_1trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 3.5f;
            yqj_delay_ms = 50;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        case 15:
                    // 电阻
                    yqj_condition = yqj_dianzu_trigger(adc_value);
                    yqj_case_trigger = 1;
                    yqj_left_speed_mps = 1.0f;
                    yqj_right_speed_mps = 1.0f;
                    yqj_delay_ms = 0;
                    yqj_delay_distance_m = 0.0f;
                    yqj_run_ms = 100;
                    yqj_lock_ms = 33;
                    yqj_lock_distance_m = 0.2f;
                    break;
        case 16:
                    // 右转
                    yqj_condition = yqj_right_turn_trigger(adc_value);
                    yqj_case_trigger = 1;
                    yqj_left_speed_mps = 3.5f;
                    yqj_right_speed_mps = 0.0f;
                    yqj_delay_ms = 0;
                    yqj_delay_distance_m = 0.0f;
                    yqj_run_ms = 120;
                    yqj_lock_ms = 150;
                    yqj_lock_distance_m = 0.4f;
                    break;
        /* kaiguang case disabled
        case 18:
                    // 开关
                    yqj_condition = yqj_kaiguang0_1trigger(adc_value);
                    yqj_case_trigger = 1;
                    yqj_left_speed_mps = 1.0f;
                    yqj_right_speed_mps = 1.0f;
                    yqj_delay_ms = 0;
                    yqj_delay_distance_m = 0.0f;
                    yqj_run_ms = 100;
                    yqj_lock_ms = 140;
                    yqj_lock_distance_m = 0.5f;
                    break;
        */
        case 17:
                    // 电源
                    yqj_condition = yqj_dianyuan_trigger(adc_value);
                    yqj_case_trigger = 1;
                    yqj_left_speed_mps = 1.0f;
                    yqj_right_speed_mps = 1.0f;
                    yqj_delay_ms = 0;
                    yqj_delay_distance_m = 0.0f;
                    yqj_run_ms = 100;
                    yqj_lock_ms = 66;
                    yqj_lock_distance_m = 0.2f;
                    break;

        case 18:
            // 左转
            yqj_condition = yqj_double_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 3.5f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 100;
            yqj_lock_distance_m = 0.2f;
            break;
        case 19:
            // 左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 3.5f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
        /*case 22:
            // 开关
            yqj_condition = yqj_kaiguang1_0trigger(adc_value);
                                yqj_case_trigger = 1;
                                yqj_left_speed_mps = 1.0f;
                                yqj_right_speed_mps = 1.0f;
                                yqj_delay_ms = 0;
                                yqj_delay_distance_m = 0.0f;
                                yqj_run_ms = 100;
                                yqj_lock_ms = 140;
                                yqj_lock_distance_m = 0.5f;
                                break;*/
        case 20:
                            // 电阻
                            yqj_condition = yqj_dianzu_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 1.0f;
                            yqj_right_speed_mps = 1.0f;
                            yqj_delay_ms = 0;
                            yqj_delay_distance_m = 0.0f;
                            yqj_run_ms = 100;
                            yqj_lock_ms = 33;
                            yqj_lock_distance_m = 0.2f;
                            break;
        case 21:
            //二极管
            yqj_condition = yqj_erji_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.5f;
            break;
        case 22:
            // 电容
            yqj_condition = yqj_dianrong_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.4f;
            break;
        case 23:
            // 左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 3.5f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
        case 24:
            // 电池
            yqj_condition = yqj_dianchi_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        case 25:
            //电容
            yqj_condition = yqj_dianrong_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        case 26:
            // 电容
            yqj_condition = yqj_dianrong_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        case 27:
            //左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 3.5f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
        case 28:
        
            //左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 0.0f;
            yqj_right_speed_mps = 3.5f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
        case 29:
            // 左电感
            yqj_condition = yqj_ldiangan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms =80;
            yqj_lock_distance_m = 0.4f;
            break;
        case 30:
            //非门
            yqj_condition = yqj_feimen_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 66;
            yqj_lock_distance_m = 0.1f;
            break;
        case 31:
            //右转
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 3.5f;
            yqj_right_speed_mps = 0.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
        case 32:
                    // 电阻
                    yqj_condition = yqj_dianzu_trigger(adc_value);
                    yqj_case_trigger = 1;
                    yqj_left_speed_mps = 1.0f;
                    yqj_right_speed_mps = 1.0f;
                    yqj_delay_ms = 0;
                    yqj_delay_distance_m = 0.0f;
                    yqj_run_ms = 100;
                    yqj_lock_ms = 33;
                    yqj_lock_distance_m = 0.2f;
                    break;
        case 33:
                    //不右转
                    yqj_condition = yqj_right_turn_trigger(adc_value);
                    yqj_case_trigger = 1;
                    yqj_left_speed_mps = 1.0f;
                    yqj_right_speed_mps = 1.0f;
                    yqj_delay_ms = 0;
                    yqj_delay_distance_m = 0.0f;
                    yqj_run_ms = 50;
                    yqj_lock_ms = 150;
                    yqj_lock_distance_m = 0.4f;
                    break;
        /* kaiguang case disabled
        case 36:
            // 开关
            yqj_condition = yqj_kaiguang1_0trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 66;
            yqj_lock_distance_m = 0.4f;
            break;
        */
        case 34:
            // 电源
            yqj_condition = yqj_dianyuan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_left_speed_mps = 1.0f;
            yqj_right_speed_mps = 1.0f;
            yqj_delay_ms = 0;
            yqj_delay_distance_m = 0.0f;
            yqj_run_ms = 100;
            yqj_lock_ms = 66;
            yqj_lock_distance_m = 0.4f;
            break;
        case 35:
                    //右转
                    yqj_condition = yqj_right_turn_trigger(adc_value);
                    yqj_case_trigger = 1;
                    yqj_left_speed_mps = 3.5f;
                    yqj_right_speed_mps = 0.0f;
                    yqj_delay_ms = 0;
                    yqj_delay_distance_m = 0.0f;
                    yqj_run_ms = 120;
                    yqj_lock_ms = 150;
                    yqj_lock_distance_m = 0.4f;
                    break;
        /* kaiguang case disabled
        case 39:
                    // 开关
                    yqj_condition = yqj_kaiguang_trigger(adc_value);
                    yqj_case_trigger = 1;
                    yqj_left_speed_mps = 1.0f;
                    yqj_right_speed_mps = 1.0f;
                    yqj_delay_ms = 0;
                    yqj_delay_distance_m = 0.0f;
                    yqj_run_ms = 100;
                    yqj_lock_ms = 66;
                    yqj_lock_distance_m = 0.4f;
                    break;
        */
                case 36:
                    // 电源
                    yqj_condition = yqj_dianyuan_trigger(adc_value);
                    yqj_case_trigger = 1;
                    yqj_left_speed_mps = 1.0f;
                    yqj_right_speed_mps = 1.0f;
                    yqj_delay_ms = 0;
                    yqj_delay_distance_m = 0.0f;
                    yqj_run_ms = 100;
                    yqj_lock_ms = 66;
                    yqj_lock_distance_m = 0.4f;
                    break;

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
            // 所有动作均由各 case 的 yqj_run_ms 决定执行时长。
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
            yqj_apply_action(yqj_left_speed_mps,
                             yqj_right_speed_mps,
                             &final_left_target,
                             &final_right_target);
        }

        // ==================== 直接使用巡线结果，不做软启动 ====================
        left_target_count = final_left_target;
        right_target_count = final_right_target;

        left_pwm_min = left_speed_decel_flag ? -PWM_DUTY_MAX : 0;
        right_pwm_min = right_speed_decel_flag ? -PWM_DUTY_MAX : 0;

        left_pwm = limit_int16((int16)left_base_pwm, left_pwm_min, PWM_DUTY_MAX);
        right_pwm = limit_int16((int16)right_base_pwm, right_pwm_min, PWM_DUTY_MAX);

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

            printf("flag=%2d motorFault=%d pwmL=%5d pwmR=%5d targetL=%5d targetR=%5d encL=%5d encR=%5d speedL=%5.3f speedR=%5.3f\r\n",
                   yqj_flag,
                   motor_emergency_is_latched(),
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
        // 读取全部 15 路 ADC，3 次平均兼顾稳定和响应速度。
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

    // ==================== 电机超速保护 ====================
    // 使用编码器实测速度而不是目标速度；正转、反转均按绝对值判断。
    // 20ms 内任一编码器计数超过 3m/s 对应阈值，立即锁存并关闭两侧电机。
    if(((float)left_encoder_count > MOTOR_MAX_SAFE_COUNT) ||
       ((float)left_encoder_count < -MOTOR_MAX_SAFE_COUNT) ||
       ((float)right_encoder_count > MOTOR_MAX_SAFE_COUNT) ||
       ((float)right_encoder_count < -MOTOR_MAX_SAFE_COUNT))
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
