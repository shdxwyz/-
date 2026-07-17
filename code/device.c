#include "device.h"

// 一旦触发只能通过重新上电清除，防止控制程序反复重启电机。
static volatile uint8 motor_emergency_latched = 0;

// 将有符号 PWM 限制在 [-PWM_DUTY_MAX, PWM_DUTY_MAX] 内。
// 符号代表转向，绝对值代表占空比计数。
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
    // 重新上电/初始化时才清除紧急停车锁存。
    motor_emergency_latched = 0;

    // 方向引脚初始为低，PWM 初始为 0，避免上电误动。
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
        // 紧急停车锁存后，强制忽略上层的非零指令。
        pwm = 0;
    }

    pwm = motor_limit(pwm);

    if(pwm > 0)
    {
        duty = pwm;

        // DRV8701 方向脚为高电平时正转。
        gpio_set_level(LEFT_IN, GPIO_HIGH);
        pwm_set_duty(LEFT_PWM, duty);
    }
    else if(pwm < 0)
    {
        duty = -pwm;

        // 反转时方向脚置低，PWM 底层仍使用正的绝对值。
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
        // 左右轮共用同一个紧急停车锁存。
        pwm = 0;
    }

    pwm = motor_limit(pwm);

    if(pwm > 0)
    {
        duty = pwm;

        // 右电机的方向电平定义与左电机相同。
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
    // 两路输出作为一个整体更新，避免超速中断在两次写入之间触发后，
    // 主循环又用中断前的旧命令短暂重启其中一路电机。
    interrupt_global_disable();
    motor_set_left(left_pwm);
    motor_set_right(right_pwm);
    interrupt_global_enable(0);
}


void motor_stop(void)
{
    // 不设置 emergency latch，因此后续 motor_control() 仍然可以启动电机。
    motor_set_left(0);
    motor_set_right(0);
}


void motor_emergency_stop(void)
{
    // 先锁存，再关闭两侧输出；之后任何非零输出命令都会被拒绝。
    motor_emergency_latched = 1;
    motor_set_left(0);
    motor_set_right(0);
}


uint8 motor_emergency_is_latched(void)
{
    // 用于主循环或串口判断紧急停车是否已生效。
    return motor_emergency_latched;
}

