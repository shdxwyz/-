#include "zf_common_headfile.h"
#include "isr_config.h"
#include "device.h"
#include "pid.h"
#include "../code/yqj.h"
#include "../code/xunji.h"

#pragma section all "cpu0_dsram"

// ==================== PIT 与编码器配置 ====================

#define PIT0                            (CCU60_CH0)

// 左编码器
#define LEFT_ENCODER                    (TIM2_ENCODER)
#define LEFT_ENCODER_PULSE              (TIM2_ENCODER_CH1_P33_7)
#define LEFT_ENCODER_DIR                (TIM2_ENCODER_CH2_P33_6)

// 右编码器
#define RIGHT_ENCODER                   (TIM4_ENCODER)
#define RIGHT_ENCODER_PULSE             (TIM4_ENCODER_CH1_P02_8)
#define RIGHT_ENCODER_DIR               (TIM4_ENCODER_CH2_P00_9)


// ==================== 速度 PID 参数 ====================

// 实测：小车走 1 米约 12106 个编码器计数
#define ENCODER_COUNT_PER_METER         (12106.0f)

// 目标基础速度�?.6 m/s
#define TARGET_SPEED_MPS                (0.3f)

// PID 周期�?0ms
#define PID_PERIOD_MS                   (20)
#define PID_PERIOD_S                    (0.02f)

// 20ms 内基础目标计数�?.6 * 0.02 * 12106 �?145.3
#define BASE_TARGET_COUNT               (TARGET_SPEED_MPS * PID_PERIOD_S * ENCODER_COUNT_PER_METER)

// PID 输出范围
#define SPEED_PID_MAX_OUT               (5000.0f)
#define SPEED_PID_MAX_IOUT              (1000.0f)

// 速度 PID 参数
// 电机电源�?7.6V 升到 12V，电压提�?58%，系统增益变�?
// 降低 Kp 和前馈系�?
#define SPEED_KP                        (25.0f)
#define SPEED_KI                        (0.1f)
#define SPEED_KD                        (0.0f)

// 前馈系数：PWM = FEEDFORWARD_GAIN * target_speed
// 电压�?7.6V 升到 12V，同样的 PWM 转速更快，前馈系数按比例降�?
// 10000 * 7.6 / 12 �?6300
#define FEEDFORWARD_GAIN                (1000.0f)

// Start boost for static friction when target exists but encoder is nearly zero.
#define SPEED_START_MIN_PWM             (1800.0f)
#define SPEED_START_ENCODER_COUNT       (3)


// ==================== 巡线参数 ====================

#define SENSOR_NUM                      (XUNJI_SENSOR_NUM)


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
    ADC0_CH11_A11
};

// ==================== 编码器与 PID 变量 ====================

// 20ms 内编码器增量，用于速度 PID
volatile int16 left_encoder_count = 0;
volatile int16 right_encoder_count = 0;

// 软件累计总计数，用于算总路�?
volatile int32 left_encoder_total = 0;
volatile int32 right_encoder_total = 0;

// 小车总路程，单位 m
volatile float car_distance_m = 0.5f;

// 循迹算出来的左右目标计数
volatile float left_target_count = BASE_TARGET_COUNT;
volatile float right_target_count = BASE_TARGET_COUNT;

// PID 输出 PWM
volatile float left_base_pwm = 0;
volatile float right_base_pwm = 0;

PidTypeDef left_speed_pid;
PidTypeDef right_speed_pid;


// ==================== 函数声明 ====================

void adc_all_init(void);
void adc_all_read(void);

int16 limit_int16(int16 value, int16 min, int16 max);
float speed_pwm_start_compensate(float pwm, float target_count, int16 encoder_count);


// ==================== 主函�?====================

int core0_main(void)
{
    xunji_result_struct line_result = {0, 0, BASE_TARGET_COUNT, BASE_TARGET_COUNT};

    int16 left_pwm = 0;
    int16 right_pwm = 0;

    uint32 print_count = 0;
    float final_left_target = BASE_TARGET_COUNT;
    float final_right_target = BASE_TARGET_COUNT;

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

    // ADC 初始�?
    adc_all_init();

    // 编码器初始化
    encoder_dir_init(LEFT_ENCODER, LEFT_ENCODER_PULSE, LEFT_ENCODER_DIR);
    encoder_dir_init(RIGHT_ENCODER, RIGHT_ENCODER_PULSE, RIGHT_ENCODER_DIR);

    // 电机初始�?
    motor_init();

    // PID 初始�?
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

    // 20ms 速度 PID
    pit_ms_init(PIT0, PID_PERIOD_MS);

    cpu_wait_event_ready();
    yqj_init(PID_PERIOD_S, ENCODER_COUNT_PER_METER);

    while(TRUE)
    {
        // ==================== 读取 10 �?ADC ====================

        adc_all_read();
        // ==================== 巡线�?====================
        // xunji 只根�?ADC 计算普通巡线目标，不处理任何特殊命令�?
        xunji_update(adc_value, BASE_TARGET_COUNT, &line_result);

        // ==================== 元器件顺序层 ====================
        // 这里就是总流程：正常巡线、判断当�?flag、延时、执行动作、自锁、flag 加一�?
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

        switch(yqj_flag)
        {
            case 1:
                            // 电源
                            yqj_condition = yqj_dianyuan_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 1.6f;
                            yqj_right_speed_mps = 1.6f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 50;
                            yqj_lock_ms = 50;
                            yqj_lock_distance_m = 0.5f;
                            break;

            case 200:
                            // A10 �?A11 同时小于 500 后右转�?
                            yqj_condition = yqj_right_turn_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 1.5f;
                            yqj_right_speed_mps = 0.0f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 400;
                            yqj_lock_ms = 33;
                            yqj_lock_distance_m = 0.2f;
                            break;
            case 3:
                // A10 �?A11 同时小于 500 后右转�?
                yqj_condition = yqj_right_turn_trigger(adc_value);
                yqj_case_trigger = 1;
                yqj_left_speed_mps = 1.5f;
                yqj_right_speed_mps = 0.0f;
                yqj_delay_ms = 0;
                yqj_run_ms = 400;
                yqj_lock_ms = 66;
                yqj_lock_distance_m = 0.4f;
                break;

//            case 4:
//                // 电阻
//                yqj_condition = yqj_dianzu_trigger(adc_value);
//                yqj_case_trigger = 1;
//                yqj_left_speed_mps = 0.3f;
//                yqj_right_speed_mps = 0.3f;
//                yqj_delay_ms = 0;
//                yqj_run_ms = 350;
//                yqj_lock_ms = 1;
//                yqj_lock_distance_m = 0.1f;
//                break;
            case 4:
                // 支角弯左转：左边检测到白线，右边没有�?
                yqj_condition = yqj_left_turn_trigger(adc_value);
                yqj_case_trigger = 1;
                            yqj_left_speed_mps = 0.0f;
                yqj_right_speed_mps = 1.5f;
                yqj_delay_ms = 0;
                            yqj_run_ms = 400;
                yqj_lock_ms = 33;
                yqj_lock_distance_m = 0.4f;
                break;
            
        
            case 5:
                            // 双支角弯左转：左右两边同时检测到白线�?
                            yqj_condition = yqj_double_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 0.0f;
                            yqj_right_speed_mps = 1.5f;     
                            yqj_delay_ms = 0;
                            yqj_run_ms = 400;
                            yqj_lock_ms = 33;
                            yqj_lock_distance_m = 0.1f;
                            break;

            case 6:
                            // 三极�?_1
                            yqj_condition = yqj_sanjiguan0_1trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 1.5f;
                            yqj_right_speed_mps = 0.0f;
                            yqj_delay_ms = 100;
                            yqj_run_ms = 350;
                            yqj_lock_ms = 66;
                            yqj_lock_distance_m = 0.4f;
                            break;
            case 7:
                            // A10 �?A11 同时小于 500 后右转�?
                            yqj_condition = yqj_right_turn_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 1.5f;
                            yqj_right_speed_mps = 0.0f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 400;
                            yqj_lock_ms = 66;
                            yqj_lock_distance_m = 1.2f;
                            break;
            
//            case 8:
//                            //电阻
//                            yqj_condition = yqj_dianzu_trigger(adc_value);
//                            yqj_case_trigger = 1;
//                            yqj_left_speed_mps = 0.3f;
//                            yqj_right_speed_mps = 0.3f;
//                            yqj_delay_ms = 0;
//                            yqj_run_ms = 350;
//                            yqj_lock_ms = 200;
//                            yqj_lock_distance_m = 0.1f;
//                            break;

            case 8:
                            //A10 �?A11 同时小于 500 后右转�?
                            yqj_condition = yqj_right_turn_trigger(adc_value);
                            yqj_case_trigger = 1;               
                            yqj_left_speed_mps = 1.5f;
                            yqj_right_speed_mps = 0.0f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 400;
                            yqj_lock_ms = 66;
                            yqj_lock_distance_m = 0.1f;
                            break;
            case 9:
                            //三极�?_1
                            yqj_condition = yqj_sanjiguan0_1trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 0.0f;
                            yqj_right_speed_mps = 1.5f;
                            yqj_delay_ms = 100;
                            yqj_run_ms = 300;
                            yqj_lock_ms = 66;
                            yqj_lock_distance_m = 0.4f;
                            break;
            
            case 10:
                            // 双支角弯左转：左右两边同时检测到白线�?
                            yqj_condition = yqj_double_trigger(adc_value);
                            yqj_case_trigger = 1;               
                            yqj_left_speed_mps = 0.0f;
                            yqj_right_speed_mps = 1.5f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 400;
                            yqj_lock_ms = 66;
                            yqj_lock_distance_m = 0.3f;
                            break;
            case 11:
                            // A1 �?A2 同时小于 500 后左转�?
                            yqj_condition = yqj_left_turn_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 0.0f;
                            yqj_right_speed_mps = 1.5f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 350;
                            yqj_lock_ms = 66;
                            yqj_lock_distance_m = 2.0f;
                            break;
//            case 12:
//                                // 电阻
//                                yqj_condition = yqj_dianzu_trigger(adc_value);
//                                yqj_case_trigger = 1;
//                                yqj_left_speed_mps = 0.3f;
//                                yqj_right_speed_mps = 0.3f;
//                                yqj_delay_ms = 0;
//                                yqj_run_ms = 350;
//                                yqj_lock_ms = 200;
//                                yqj_lock_distance_m = 0.1f;
//                                break;
//            case 13:
//                            // 左拐�?
//                            yqj_condition = yqj_left_turn_trigger(adc_value);
//                            yqj_case_trigger = 1;
//                            yqj_left_speed_mps = 0.3f;
//                            yqj_right_speed_mps = 0.3f;
//                            yqj_delay_ms = 0;
//                            yqj_run_ms = 300;
//                            yqj_lock_ms = 200;
//                            yqj_lock_distance_m = 0.1f;
//                            break;
//            case 14:
//                            //电阻
//                            yqj_condition = yqj_dianzu_trigger(adc_value);
//                            yqj_case_trigger = 1;
//                            yqj_left_speed_mps = 0.3f;
//                            yqj_right_speed_mps = 0.3f;
//                            yqj_delay_ms = 0;
//                            yqj_run_ms = 350;
//                            yqj_lock_ms = 200;
//                            yqj_lock_distance_m = 0.1f;
//                            break;
            case 12 :       //双支角弯_right
                            yqj_condition=yqj_double_trigger(adc_value);
                            yqj_case_trigger=1;
                            yqj_left_speed_mps=1.5f;
                            yqj_right_speed_mps=0.0f;
                            yqj_delay_ms=0;
                            yqj_run_ms=400;
                            yqj_lock_ms=66;
                            yqj_lock_distance_m=0.6f;
                            break;
//            case 16 :
//                               //电阻
//                                yqj_condition = yqj_dianzu_trigger(adc_value);
//                                yqj_case_trigger = 1;
//                                yqj_left_speed_mps = 0.3f;
//                                yqj_right_speed_mps = 0.3f;
//                                yqj_delay_ms = 0;
//                                yqj_run_ms = 350;
//                                yqj_lock_ms = 200;
//                                yqj_lock_distance_m = 0.1f;
//                                break;
            case 13:
                           //A10 �?A11 同时小于 500 后右转�?
                            yqj_condition = yqj_right_turn_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 1.5f;
                            yqj_right_speed_mps = 0.0f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 400;
                            yqj_lock_ms = 66;
                            yqj_lock_distance_m = 0.1f;
                            break;

            case 14:
                            //二极�?
                            yqj_condition = yqj_erjiguan_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 0.8f;
                            yqj_right_speed_mps = 0.8f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 500;
                            yqj_lock_ms = 66;
                            yqj_lock_distance_m = 0.1f;
                            break;

            case 15:
                            //A11 �?A12 同时小于 500 后右转�?
                            yqj_condition = yqj_right_turn_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 1.5f;
                            yqj_right_speed_mps = 0.0f;    
                            yqj_delay_ms = 0;
                            yqj_run_ms = 400;
                            yqj_lock_ms = 66;
                            yqj_lock_distance_m = 0.8f;
                            break;
            case 16:
                            // 开�?_1
                            yqj_condition = yqj_kaiguang0_1trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 0.8f;
                            yqj_right_speed_mps = 0.8f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 500;
                            yqj_lock_ms = 66;
                            yqj_lock_distance_m = 0.4f;
                            break;

            case 17:
                            // A10 �?A11 同时小于 500 后右转�?
                            yqj_condition = yqj_right_turn_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 1.5f;
                            yqj_right_speed_mps = 0.0f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 400;
                            yqj_lock_ms = 66;
                            yqj_lock_distance_m = 0.1f;
                            break;

            case 18:
                            // 电源
                            yqj_condition = yqj_dianyuan_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 0.6f;
                            yqj_right_speed_mps = 0.6f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 500;
                            yqj_lock_ms = 66;
                            yqj_lock_distance_m = 0.1f;
                            break;

            default:
                // 停止
                //yqj_flag=2;
                motor_stop();
                system_delay_ms(20000);
                break;
        }

        if(YQJ_STATE_LINE == yqj_state)
        {
            if(yqj_condition)
            {
                yqj_start_case(yqj_case_trigger);
            }
        }
        else if(YQJ_STATE_DELAY == yqj_state)
        {
            if(yqj_time_reached(yqj_state_start_time, yqj_delay_ms))
            {
                yqj_state = YQJ_STATE_RUN;
                yqj_state_start_time = system_getval();
            }
        }
        else if(YQJ_STATE_RUN == yqj_state)
        {
            if(yqj_time_reached(yqj_state_start_time, yqj_run_ms))
            {
                yqj_start_lock(left_encoder_total + right_encoder_total);
            }
        }
        else if(YQJ_STATE_LOCK == yqj_state)
        {
            if(yqj_lock_done(left_encoder_total + right_encoder_total,
                             yqj_lock_ms,
                             yqj_lock_distance_m))
            {
                yqj_finish_case();
            }
        }
        else
        {
            yqj_set_flag(0);
        }

        if(YQJ_STATE_RUN == yqj_state)
        {
            yqj_apply_action(yqj_left_speed_mps, yqj_right_speed_mps, &final_left_target, &final_right_target);
        }

        left_target_count  = final_left_target;
        right_target_count = final_right_target;

        // ==================== 输出电机 ====================
        // PID �?20ms 中断里根据左右目标计数输�?PWM

        left_pwm  = (int16)left_base_pwm;
        right_pwm = (int16)right_base_pwm;

        left_pwm  = limit_int16(left_pwm,  -PWM_DUTY_MAX, PWM_DUTY_MAX);
        right_pwm = limit_int16(right_pwm, -PWM_DUTY_MAX, PWM_DUTY_MAX);

        motor_control(left_pwm, right_pwm);

        // ==================== 串口调试 ====================

        print_count++;
        if(print_count >= 50)
        {
            uint8 i;
            float left_speed_mps = (float)left_encoder_count /
                                   (ENCODER_COUNT_PER_METER * PID_PERIOD_S);
            float right_speed_mps = (float)right_encoder_count /
                                    (ENCODER_COUNT_PER_METER * PID_PERIOD_S);

            print_count = 0;

            printf("ADC:");

            for(i = 0; i < SENSOR_NUM; i++)
            {
                printf(" %4d", adc_value[i]);
            }

            printf("\r\n");
            printf("pwmL=%6d pwmR=%6d targetL=%5d targetR=%5d encL=%5d encR=%5d speedL=%6.3f speedR=%6.3f\r\n",
                    left_pwm,
                    right_pwm,
                    (int)left_target_count,
                    (int)right_target_count,
                    left_encoder_count,
                    right_encoder_count,
                    left_speed_mps,
                    right_speed_mps);
        }

        system_delay_ms(2);
    }
}


// ==================== ADC 初始�?====================

void adc_all_init(void)
{
    uint8 i;
    for(i = 0; i < SENSOR_NUM; i++)
    {
        adc_init(adc_list[i], ADC_12BIT);
    }
}


// ==================== ADC 读取 ====================

void adc_all_read(void)
{
    uint8 i;
    for(i = 0; i < SENSOR_NUM; i++)
    {
        // 10 路全部用于循迹，3 次平均保证响应较�?
        adc_value[i] = adc_mean_filter_convert(adc_list[i], 3);
    }
}


// ==================== 20ms 速度 PID 中断 ====================

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);

    // ==================== 读取 20ms 内编码器增量 ====================
    // 左编码器前进时是负数，所以取反变�?
    left_encoder_count = -encoder_get_count(LEFT_ENCODER);

    // 右编码器前进时是正数
    right_encoder_count =encoder_get_count(RIGHT_ENCODER);

    // ==================== 读完立刻清空硬件编码�?====================

    encoder_clear_count(LEFT_ENCODER);
    encoder_clear_count(RIGHT_ENCODER);

    // ==================== 防止偶发负数影响速度 PID ====================

    if(left_encoder_count < 0)
    {
        left_encoder_count = -left_encoder_count;
    }

    if(right_encoder_count < 0)
    {
        right_encoder_count = -right_encoder_count;
    }

    // ==================== 软件累计总路�?====================

    left_encoder_total += left_encoder_count;
    right_encoder_total += right_encoder_count;

    car_distance_m = ((float)left_encoder_total + (float)right_encoder_total) /
                     (2.0f * ENCODER_COUNT_PER_METER);

    // ==================== 速度 PID ====================
    // PID_Calc(pid, 实际�? 目标�?

    left_base_pwm = PID_Calc(&left_speed_pid,
                             (float)left_encoder_count,
                             left_target_count);

    right_base_pwm = PID_Calc(&right_speed_pid,
                              (float)right_encoder_count,
                              right_target_count);

    // ==================== 前馈控制 ====================
    // 根据目标速度直接给基础 PWM，减�?PID 负担
    // 前馈 = FEEDFORWARD_GAIN * target_speed
    // target_speed = left_target_count / (PID_PERIOD_S * ENCODER_COUNT_PER_METER)
    left_base_pwm  += FEEDFORWARD_GAIN * left_target_count / (PID_PERIOD_S * ENCODER_COUNT_PER_METER);
    right_base_pwm += FEEDFORWARD_GAIN * right_target_count / (PID_PERIOD_S * ENCODER_COUNT_PER_METER);

    left_base_pwm = speed_pwm_start_compensate(left_base_pwm,
                                               left_target_count,
                                               left_encoder_count);
    right_base_pwm = speed_pwm_start_compensate(right_base_pwm,
                                                right_target_count,
                                                right_encoder_count);
}


// ==================== 限幅函数 ====================

float speed_pwm_start_compensate(float pwm, float target_count, int16 encoder_count)
{
    if(target_count <= 0.0f)
    {
        return 0.0f;
    }

    if(encoder_count <= SPEED_START_ENCODER_COUNT &&
       pwm > 0.0f &&
       pwm < SPEED_START_MIN_PWM)
    {
        return SPEED_START_MIN_PWM;
    }

    if(pwm < 0.0f)
    {
        return 0.0f;
    }

    return pwm;
}


int16 limit_int16(int16 value, int16 min, int16 max)
{
    if(value > max)
    {
        return max;
    }
    else if(value < min)
    {
        return min;
    }
    else
    {
        return value;
    }
}

#pragma section all restore
