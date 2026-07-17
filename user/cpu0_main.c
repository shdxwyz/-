#include "zf_common_headfile.h"
#include "isr_config.h"
#include "device.h"
#include "pid.h"
#include "../code/yqj.h"
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

// 实测（强磁电机）：小车走 0.5 米约 27000 个编码器计数
// 1 米 = 27000 * 2 = 54000
#define ENCODER_COUNT_PER_METER (54000.0f)

// 目标基础速度 1.0 m/s
#define TARGET_SPEED_MPS (1.5f)

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
#define SPEED_KP (2.8f)
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
    uint32 yqj_run_ms = 0;
    uint32 yqj_lock_ms = 0;
    float yqj_lock_distance_m = 0.0f;
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

    pwm_init(ATOM0_CH6_P02_6, 100, 1600);
    system_delay_ms(2000);

    // IMU660RC 初始化（120Hz 四元数输出）
    imu660rc_init(IMU660RC_QUARTERNION_120HZ);
   
    yqj_init(PID_PERIOD_S, ENCODER_COUNT_PER_METER);
        
    pit_ms_init(PIT0, PID_PERIOD_MS);
    while (TRUE)




    {
        // ==================== 读取 15 路 ADC ====================

        adc_all_read();
        // ==================== 巡线层 ====================
        // xunji 只根据 ADC 计算普通巡线目标，不处理特殊动作命令。
        // 传入 &adc_value[XUNJI_LINE_START_IDX] 跳过 A0,A1，只使用巡线用的 11 路
        xunji_update(&adc_value[XUNJI_LINE_START_IDX], BASE_TARGET_COUNT, &line_result);

        // ==================== 元器件顺序层 ====================
        // 总流程：正常巡线、判断当前 flag、延时、执行动作、自锁、flag 加一。
        yqj_condition = 0;
        yqj_case_trigger = 0;
        yqj_delay_ms = 0;
        yqj_run_ms = 0;
        yqj_lock_ms = 0;
        yqj_lock_distance_m = 0.0f;
        yqj_turn_base_speed = 0.0f;

        final_left_target = line_result.left_target_count;
        final_right_target = line_result.right_target_count;

        switch (yqj_flag)
        {
            /* kaiguang case disabled
            case 1:
                        // 电源
                        yqj_condition = yqj_dianyuan_trigger(adc_value);
                        yqj_case_trigger = 1;
                        yqj_turn_base_speed = 1.0f;
                        yqj_delay_ms = 0;
                        yqj_run_ms = 100;
                        yqj_lock_ms = 66;
                        yqj_lock_distance_m = 0.2f;
                        break;
                  */
        case 1:
            // 电源
            /*yqj_condition = yqj_dianyuan_trigger(adc_value);
             yqj_case_trigger = 1;
             yqj_turn_base_speed = 1.0f;
             yqj_delay_ms = 0;
             yqj_run_ms = 100;
             yqj_lock_ms = 66;
             yqj_lock_distance_m = 0.2f;
             break;*/
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = -1.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 0;
            yqj_lock_ms = 0;
            yqj_lock_distance_m = 0.0f;
            break;
        // case 2:
        //     //电阻
        //     yqj_condition = yqj_dianzu_trigger(adc_value);
        //     yqj_case_trigger = 1;
        //     yqj_turn_base_speed = 0.0f;
        //     yqj_delay_ms = 0;
        //     yqj_run_ms = 100;
        //     yqj_lock_ms = 33;
        //     yqj_lock_distance_m = 0.2f;
        //     break;
        case 2:
            // 巡线走 1m
            yqj_condition = 1;
            yqj_case_trigger = 0;
            yqj_turn_base_speed = 0.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 0;
            yqj_lock_ms = 0;
            yqj_lock_distance_m = 1.0f;
            break;
        case 3:
            // 右转
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = -1.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
            // case 4:
            // //feimen
            //         yqj_condition = yqj_feimen_trigger(adc_value);
            //         yqj_case_trigger = 1;
            //         yqj_turn_base_speed = 0.0f;
            //         yqj_delay_ms = 0;
            //         yqj_run_ms = 100;
            //         yqj_lock_ms = 33;
            //         yqj_lock_distance_m = 0.2f;
            //         break;

        case 4:
            // 左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = 1.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
            // case 5:
            //     //sanjiguan
            //     yqj_condition = yqj_sanjiguan0_1trigger(adc_value);
            //     yqj_case_trigger = 1;
            //     yqj_turn_base_speed = 0.0f;
            //     yqj_delay_ms = 0;
            //     yqj_run_ms = 100;
            //     yqj_lock_ms = 33;
            //     yqj_lock_distance_m = 0.2f;
            //     break;
            // case 6:
            //     //erjiguan
            //     yqj_condition = yqj_erjiguan_trigger(adc_value);
            //     yqj_case_trigger = 1;
            //     yqj_turn_base_speed = 0.0f;
            //     yqj_delay_ms = 0;
            //     yqj_run_ms = 100;
            //     yqj_lock_ms = 33;
            //     yqj_lock_distance_m = 0.2f;
            //     break;
            // case 7:
            //     //sanjiguan
            //     yqj_condition = yqj_sanjiguan0_2trigger(adc_value);
            //     yqj_case_trigger = 1;
            //     yqj_turn_base_speed = 1.0f;
            //     yqj_delay_ms = 0;
            //     yqj_run_ms = 120;
            //     yqj_lock_ms = 150;
            //     yqj_lock_distance_m = 0.4f;
            //     break;
            // case 8:
            //     //zuowan
            //     yqj_condition = yqj_left_turn_trigger(adc_value);
            //     yqj_case_trigger = 1;
            //     yqj_turn_base_speed = 0.0f;
            //     yqj_delay_ms = 0;
            //     yqj_run_ms = 100;
            //     yqj_lock_ms = 33;
            //     yqj_lock_distance_m = 0.2f;
            //     break;

        case 5:
            // 左转
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = 1.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
        case 6:
            // zuowan
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = 0.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        case 7:
            //     double trigger
            yqj_condition = yqj_double_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = 1.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_run_ms = 100;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        // case 8:
        //     // 二级管
        //     yqj_condition = yqj_erjiguan_trigger(adc_value);
        //     yqj_case_trigger = 1;
        //     yqj_turn_base_speed = 0.0f;
        //     yqj_delay_ms = 0;
        //     yqj_run_ms = 100;
        //     yqj_lock_ms = 33;
        //     yqj_lock_distance_m = 0.2f;
        //     break;
        // case 8:
        //     // dianrong
        //     yqj_condition = yqj_dianrong_trigger(adc_value);
        //     yqj_case_trigger = 1;
        //     yqj_turn_base_speed = 0.0f;
        //     yqj_delay_ms = 0;
        //     yqj_run_ms = 100;
        //     yqj_lock_ms = 33;
        //     yqj_lock_distance_m = 0.2f;
        //     break;
        case 8:
            yqj_condition = yqj_sanjiguan0_1trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = 1.0f;
            yqj_delay_ms = 50;
            yqj_run_ms = 120;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        case 9:
            // zuowan
            yqj_condition = yqj_left_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = 1.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;

        // case 10:
        //     // erjiguan
        //     yqj_condition = yqj_erjiguan_trigger(adc_value);
        //     yqj_case_trigger = 1;
        //     yqj_turn_base_speed = 0.0f;
        //     yqj_delay_ms = 100;
        //     yqj_run_ms = 100;
        //     yqj_lock_ms = 33;
        //     yqj_lock_distance_m = 0.2f;
        //     break;

        // case 11:
        //     // dianrong
        //     yqj_condition = yqj_dianrong_trigger(adc_value);
        //     yqj_case_trigger = 1;
        //     yqj_turn_base_speed = 0.0f;
        //     yqj_delay_ms = 100;
        //     yqj_run_ms = 100;
        //     yqj_lock_ms = 33;
        //     yqj_lock_distance_m = 0.2f;
        //     break;
        case 10:
            // double trigger
            yqj_condition = yqj_double_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = -1.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        // case 11:
        //     //youzhuan
        //     yqj_condition = yqj_right_turn_trigger(adc_value);
        //     yqj_case_trigger = 1;
        //     yqj_turn_base_speed = -1.0f;
        //     yqj_delay_ms = 0;
        //     yqj_run_ms = 120;
        //     yqj_lock_ms = 150;
        //     yqj_lock_distance_m = 0.4f;
        //     break;
        // case 12:
        //     //daingan
        //     yqj_condition = yqj_rdiangan_trigger(adc_value);
        //     yqj_case_trigger = 1;
        //     yqj_turn_base_speed = 0.0f;
        //     yqj_delay_ms = 0;
        //     yqj_run_ms = 100;
        //     yqj_lock_ms = 33;
        //     yqj_lock_distance_m = 0.2f;
        //     break;
        case 11:
            // youzhuan
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = -1.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
        case 12:
            // double trigger
            yqj_condition = yqj_double_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = 1.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        // case 13:
        //     //dianrong
        //     yqj_condition = yqj_dianrong_trigger(adc_value);
        // yqj_case_trigger = 1;
        //     yqj_turn_base_speed = 0.0f;
        //     yqj_delay_ms = 100;
        //     yqj_run_ms = 100;
        //     yqj_lock_ms = 33;
        //     yqj_lock_distance_m = 0.2f;
        //     break;
        case 13:
            // double trigger
            yqj_condition = yqj_double_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = -1.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
            // case 14:
            //     //youwan
            //     yqj_condition = yqj_right_turn_trigger(adc_value);
            //     yqj_case_trigger = 1;
            //     yqj_turn_base_speed = 0.0f;
            //     yqj_delay_ms = 0;
            //     yqj_run_ms = 120;
            //     yqj_lock_ms = 150;
            //     yqj_lock_distance_m = 0.4f;
            //     break;
            // case 15:
            //     //sanjiguan
            //     yqj_condition=yqj_sanjiguan1_0trigger(adc_value);
            //     yqj_case_trigger = 1;
            //     yqj_turn_base_speed = 0.0f;
            //     yqj_delay_ms = 0;
            //     yqj_run_ms = 100;
            //     yqj_lock_ms = 33;
            //     yqj_lock_distance_m = 0.2f;
            //     break;
            // case 16:
            //     //youwan
            //     yqj_condition = yqj_right_turn_trigger(adc_value);
            //     yqj_case_trigger = 1;
            //     yqj_turn_base_speed = 0.0f;
            //     yqj_delay_ms = 0;
            //     yqj_run_ms = 120;
            //     yqj_lock_ms = 150;
            //     yqj_lock_distance_m = 0.4f;
            //     break;

        case 14:
            // double trigger
            yqj_condition = yqj_double_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = -1.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 33;
            yqj_lock_distance_m = 0.2f;
            break;
        // case 15:
        //     //dianzu
        //     yqj_condition = yqj_dianzu_trigger(adc_value);
        //     yqj_case_trigger = 1;
        //     yqj_turn_base_speed = 0.0f;
        //     yqj_delay_ms = 0;
        //     yqj_run_ms = 100;
        //     yqj_lock_ms = 33;
        //     yqj_lock_distance_m = 0.2f;
        //     break;
        // case 16:
        //     //youwan
        //     yqj_condition = yqj_right_turn_trigger(adc_value);
        //     yqj_case_trigger = 1;
        //     yqj_turn_base_speed = 0.0f;
        //     yqj_delay_ms = 0;
        //     yqj_run_ms = 120;
        //     yqj_lock_ms = 150;
        //     yqj_lock_distance_m = 0.4f;
        //     break;
        // case 17:
        //     //xianquan'
        //     yqj_condition = yqj_xianquan_trigger(adc_value);
        //     yqj_case_trigger = 1;
        //     yqj_turn_base_speed = 0.0f;
        //     yqj_delay_ms = 0;
        //     yqj_run_ms = 100;
        //     yqj_lock_ms = 33;
        //     yqj_lock_distance_m = 0.2f;
        //     break;
        // case 18:
        //     //youwan
        //     yqj_condition = yqj_right_turn_trigger(adc_value);
        //     yqj_case_trigger = 1;
        //     yqj_turn_base_speed = 0.0f;
        //     yqj_delay_ms = 0;
        //     yqj_run_ms = 120;
        //     yqj_lock_ms = 150;
        //     yqj_lock_distance_m = 0.4f;
        //     break;
        case 19:
            // youzhuan
            yqj_condition = yqj_right_turn_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = -1.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 120;
            yqj_lock_ms = 150;
            yqj_lock_distance_m = 0.4f;
            break;
        case 20:
            // kaiguan
            yqj_condition = yqj_kaiguang0_1trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = 0.0f;
            yqj_delay_ms = 0;
            yqj_run_ms = 100;
            yqj_lock_ms = 140;
            yqj_lock_distance_m = 0.5f;
            break;
            // dainyuan

            /* kaiguang case disabled
            case 39:
                        // 开关
                        yqj_condition = yqj_kaiguang_trigger(adc_value);
                        yqj_case_trigger = 1;
                        yqj_turn_base_speed = 1.0f;
                        yqj_delay_ms = 0;
                        yqj_run_ms = 100;
                        yqj_lock_ms = 66;
                        yqj_lock_distance_m = 0.4f;
                        break;
            */
        case 36:
            // 电源
            yqj_condition = yqj_dianyuan_trigger(adc_value);
            yqj_case_trigger = 1;
            yqj_turn_base_speed = 1.0f;
            yqj_delay_ms = 0;
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
                yqj_start_case(yqj_case_trigger);
            }
        }
        else if (YQJ_STATE_DELAY == yqj_state)
        {
            if (yqj_time_reached(yqj_state_start_time, yqj_delay_ms))
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

                yqj_state = YQJ_STATE_RUN;
                yqj_state_start_time = system_getval();
            }
        }
        else if (YQJ_STATE_RUN == yqj_state)
        {
            uint8 action_done;

            if (yqj_get_action_trigger())
            {
                // 转向由角速度积分角度闭环结束；超时仅用于传感器异常或堵转保护。
                action_done = yqj_turn_target_reached() ||
                              yqj_time_reached(yqj_state_start_time,
                                               YQJ_TURN_TIMEOUT_MS);
            }
            else
            {
                action_done = yqj_time_reached(yqj_state_start_time, yqj_run_ms);
            }

            if (action_done)
            {
                if (yqj_get_action_trigger())
                {
                    // 转到目标角度后立即退出转弯，下一周期直接使用巡线目标。
                    left_speed_decel_flag = 0;
                    right_speed_decel_flag = 0;
                    yqj_finish_case();
                }
                else
                {
                    // 非转弯动作仍保留原有的时间/距离自锁流程。
                    yqj_start_lock(left_encoder_total + right_encoder_total);
                }
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
            yqj_apply_action(yqj_turn_base_speed,
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

            printf("flag=%2d motorFault=%d turnAngle=%7.2f pwmL=%5d pwmR=%5d targetL=%5d targetR=%5d encL=%5d encR=%5d speedL=%5.3f speedR=%5.3f\r\n",
                   yqj_flag,
                   motor_emergency_is_latched(),
                   yqj_integrated_angle,
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

    // ==================== 电机超速保护 ====================
    // 使用编码器实测速度而不是目标速度；正转、反转均按绝对值判断。
    // 20ms 内任一编码器计数超过 5m/s 对应阈值，立即锁存并关闭两侧电机。
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
