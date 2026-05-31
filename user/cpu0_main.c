#include "zf_common_headfile.h"
#include "isr_config.h"
#include "device.h"
#include "pid.h"
#include "pid_debug.h"

/* ==================== PIT 与编码器配置 ==================== */

#define PIT0                            (CCU60_CH0)

/* 左编码器 */
#define LEFT_ENCODER                    (TIM2_ENCODER)
#define LEFT_ENCODER_PULSE              (TIM2_ENCODER_CH1_P33_7)
#define LEFT_ENCODER_DIR                (TIM2_ENCODER_CH2_P33_6)

/* 右编码器 */
#define RIGHT_ENCODER                   (TIM4_ENCODER)
#define RIGHT_ENCODER_PULSE             (TIM4_ENCODER_CH1_P02_8)
#define RIGHT_ENCODER_DIR               (TIM4_ENCODER_CH2_P00_9)


/* ==================== 速度 PID 参数 ==================== */

/* 实测：小车走 1 米约 12106 个编码器计数 */
#define ENCODER_COUNT_PER_METER         (12106.0f)

/* 目标基础速度：0.3 m/s */
#define TARGET_SPEED_MPS                (0.3f)

/* PID 周期：20ms */
#define PID_PERIOD_MS                   (20)
#define PID_PERIOD_S                    (0.02f)

/* 20ms 内基础目标计数：0.3 * 0.02 * 12106 = 72.6 */
#define BASE_TARGET_COUNT               (TARGET_SPEED_MPS * PID_PERIOD_S * ENCODER_COUNT_PER_METER)

/* PID 输出范围 */
#define SPEED_PID_MAX_OUT               (10000.0f)
#define SPEED_PID_MAX_IOUT              (6000.0f)

/* 速度 PID 参数 */
#define SPEED_KP                        (30.0f)
#define SPEED_KI                        (0.2f)
#define SPEED_KD                        (0.0f)

/* 前馈系数：PWM = FEEDFORWARD_GAIN * target_speed */
/* 从实测数据推算：0.5m/s需要约5200PWM */
/* 斜率 ≈ 10000 */
#define FEEDFORWARD_GAIN                (10000.0f)


/* ==================== 编码器与 PID 变量 ==================== */

/* 20ms 内编码器增量，用于速度 PID */
volatile int16 left_encoder_count = 0;
volatile int16 right_encoder_count = 0;

/* 软件累计总计数，用于算总路程 */
volatile int32 left_encoder_total = 0;
volatile int32 right_encoder_total = 0;

/* 小车总路程，单位 m */
volatile float car_distance_m = 0.5f;

/* 循迹算出来的左右目标计数 */
volatile float left_target_count = BASE_TARGET_COUNT;
volatile float right_target_count = BASE_TARGET_COUNT;

/* PID 输出 PWM */
volatile float left_base_pwm = 0;
volatile float right_base_pwm = 0;

PidTypeDef left_speed_pid;
PidTypeDef right_speed_pid;



/* ==================== 函数声明 ==================== */

int16 limit_int16(int16 value, int16 min, int16 max);


/* ==================== 主函数 ==================== */

int core0_main(void)
{
    int16 left_pwm = 0;
    int16 right_pwm = 0;
    float test_speed = 0.2f;           /* 当前测试速度 */
    unsigned int test_speed_start_ms = 0;    /* 速度变化计时 */

    clock_init();
    debug_init();

    /* 编码器初始化 */
    encoder_dir_init(LEFT_ENCODER, LEFT_ENCODER_PULSE, LEFT_ENCODER_DIR);
    encoder_dir_init(RIGHT_ENCODER, RIGHT_ENCODER_PULSE, RIGHT_ENCODER_DIR);

    /* 电机初始化 */
    motor_init();

    /* PID 初始化 */
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

    /* 20ms 速度 PID */
    pit_ms_init(PIT0, PID_PERIOD_MS);

    cpu_wait_event_ready();

    /* PID 调试助手初始化 */
    pid_debug_init();

    system_delay_ms(2000);

    while(TRUE)
    {
        /* ==================== 速度测试：每10秒变化一次 ==================== */
        if(system_getval_ms() - test_speed_start_ms >= 10000)
        {
            test_speed_start_ms = system_getval_ms();
            test_speed += 0.1f;
            if(test_speed > 0.6f)
            {
                test_speed = 0.2f;
            }
            /* 更新目标速度 */
            left_target_count = test_speed * PID_PERIOD_S * ENCODER_COUNT_PER_METER;
            right_target_count = left_target_count;
            
            /* 速度变化时重置 PID，清零积分项 */
            PID_clear(&left_speed_pid);
            PID_clear(&right_speed_pid);
            
            printf("[TEST] target=%.2f m/s\r\n", test_speed);
        }

        /* ==================== PID 调试更新 ==================== */
        pid_debug_update();

        /* ==================== 输出电机 ==================== */
        left_pwm  = (int16)left_base_pwm;
        right_pwm = (int16)right_base_pwm;

        left_pwm  = limit_int16(left_pwm,  -PWM_DUTY_MAX, PWM_DUTY_MAX);
        right_pwm = limit_int16(right_pwm, -PWM_DUTY_MAX, PWM_DUTY_MAX);

        motor_control(left_pwm, right_pwm);

        system_delay_ms(2);
    }
}


/* ==================== 20ms 速度 PID 中断 ==================== */

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);

    /* 读取 20ms 内编码器增量 */
    left_encoder_count = encoder_get_count(LEFT_ENCODER);
    right_encoder_count = encoder_get_count(RIGHT_ENCODER);

    /* 读完立刻清空硬件编码器 */
    encoder_clear_count(LEFT_ENCODER);
    encoder_clear_count(RIGHT_ENCODER);

    /* 取绝对值，确保速度 PID 方向正确 */
    if(left_encoder_count < 0)
    {
        left_encoder_count = -left_encoder_count;
    }

    if(right_encoder_count < 0)
    {
        right_encoder_count = -right_encoder_count;
    }

    /* 软件累计总路程 */
    left_encoder_total += left_encoder_count;
    right_encoder_total += right_encoder_count;

    car_distance_m = ((float)left_encoder_total + (float)right_encoder_total) /
                     (2.0f * ENCODER_COUNT_PER_METER);

    /* 速度 PID */
    left_base_pwm = PID_Calc(&left_speed_pid,
                             (float)left_encoder_count,
                             left_target_count);

    right_base_pwm = PID_Calc(&right_speed_pid,
                              (float)right_encoder_count,
                              right_target_count);

    /* 前馈控制：根据目标速度直接给基础 PWM，减少 PID 负担 */
    /* 前馈 = FEEDFORWARD_GAIN * target_speed */
    /* target_speed = left_target_count / (PID_PERIOD_S * ENCODER_COUNT_PER_METER) */
    left_base_pwm  += FEEDFORWARD_GAIN * left_target_count / (PID_PERIOD_S * ENCODER_COUNT_PER_METER);
    right_base_pwm += FEEDFORWARD_GAIN * right_target_count / (PID_PERIOD_S * ENCODER_COUNT_PER_METER);
}


/* ==================== 限幅函数 ==================== */

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
