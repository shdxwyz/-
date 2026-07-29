#include "device.h"

// 一旦触发只能通过重新上电清除，防止控制程序反复重启电机。
static volatile uint8 motor_emergency_latched = 0;

static int16 motor_limit(int16 pwm)
{
    if(pwm > PWM_DUTY_MAX)
    {
        return PWM_DUTY_MAX;
    }
    else if(pwm < -PWM_DUTY_MAX)
    {
        return -PWM_DUTY_MAX;
    }
    else
    {
        return pwm;
    }
}


void motor_init(void)
{
    motor_emergency_latched = 0;
    gpio_init(LEFT_IN,  GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(RIGHT_IN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pwm_init(LEFT_PWM,  MOTOR_PWM_FREQ, 0);
    pwm_init(RIGHT_PWM, MOTOR_PWM_FREQ, 0);
}


void motor_set_left(int16 pwm)
{
    int16 duty = 0;

    if(motor_emergency_latched)
    {
        pwm = 0;
    }

    pwm = motor_limit(pwm);

    if(pwm > 0)
    {
        duty = pwm;

        gpio_set_level(LEFT_IN, GPIO_HIGH);
        pwm_set_duty(LEFT_PWM, duty);
    }
    else if(pwm < 0)
    {
        duty = -pwm;

        gpio_set_level(LEFT_IN, GPIO_LOW);
        pwm_set_duty(LEFT_PWM, duty);
    }
    else
    {
        gpio_set_level(LEFT_IN, GPIO_LOW);
        pwm_set_duty(LEFT_PWM, 0);
    }
}


void motor_set_right(int16 pwm)
{
    int16 duty = 0;

    if(motor_emergency_latched)
    {
        pwm = 0;
    }

    pwm = motor_limit(pwm);

    if(pwm > 0)
    {
        duty = pwm;

        gpio_set_level(RIGHT_IN, GPIO_HIGH);
        pwm_set_duty(RIGHT_PWM, duty);
    }
    else if(pwm < 0)
    {
        duty = -pwm;

        gpio_set_level(RIGHT_IN, GPIO_LOW);
        pwm_set_duty(RIGHT_PWM, duty);
    }
    else
    {
        gpio_set_level(RIGHT_IN, GPIO_LOW);
        pwm_set_duty(RIGHT_PWM, 0);
    }
}


void motor_control(int16 left_pwm, int16 right_pwm)
{
    uint32 interrupt_state;

    // 两路输出作为一个整体更新，避免超速中断在两次写入之间触发后，
    // 其他执行路径又用中断前的旧命令短暂重启其中一路电机。
    interrupt_state = interrupt_global_disable();
    motor_set_left(left_pwm);
    motor_set_right(right_pwm);
    interrupt_global_enable(interrupt_state);
}


void motor_stop(void)
{
    motor_control(0, 0);
}


void motor_emergency_stop(void)
{
#if MOTOR_LATCHED_STOP_ENABLE
    uint32 interrupt_state;

    // 先锁存，再关闭两侧输出；之后任何非零输出命令都会被拒绝。
    interrupt_state = interrupt_global_disable();
    motor_emergency_latched = 1;
    motor_set_left(0);
    motor_set_right(0);
    interrupt_global_enable(interrupt_state);
#else
    // 调试期间关闭锁存急停；保持锁存状态为未触发。
    motor_emergency_latched = 0;
#endif
}


uint8 motor_emergency_is_latched(void)
{
    return motor_emergency_latched;
}

