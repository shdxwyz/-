#include "device.h"


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
    gpio_init(LEFT_IN1,  GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LEFT_IN2,  GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(RIGHT_IN1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(RIGHT_IN2, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pwm_init(LEFT_PWM,  MOTOR_PWM_FREQ, 0);
    pwm_init(RIGHT_PWM, MOTOR_PWM_FREQ, 0);
}


void motor_set_left(int16 pwm)
{
    int16 duty = 0;

    pwm = motor_limit(pwm);

    if(pwm > 0)
    {
        // 正转
        duty = pwm;

        gpio_set_level(LEFT_IN1, GPIO_HIGH);
        gpio_set_level(LEFT_IN2, GPIO_LOW);
        pwm_set_duty(LEFT_PWM, duty);
    }
    else if(pwm < 0)
    {
        // 反转
        duty = -pwm;

        gpio_set_level(LEFT_IN1, GPIO_LOW);
        gpio_set_level(LEFT_IN2, GPIO_HIGH);
        pwm_set_duty(LEFT_PWM, duty);
    }
    else
    {
        gpio_set_level(LEFT_IN1, GPIO_LOW);
        gpio_set_level(LEFT_IN2, GPIO_LOW);
        pwm_set_duty(LEFT_PWM, 0);
    }
}


void motor_set_right(int16 pwm)
{
    int16 duty = 0;

    pwm = motor_limit(pwm);

    if(pwm > 0)
    {
        // 正转
        duty = pwm;

        gpio_set_level(RIGHT_IN1, GPIO_HIGH);
        gpio_set_level(RIGHT_IN2, GPIO_LOW);
        pwm_set_duty(RIGHT_PWM, duty);
    }
    else if(pwm < 0)
    {
        // 反转
        duty = -pwm;

        gpio_set_level(RIGHT_IN1, GPIO_LOW);
        gpio_set_level(RIGHT_IN2, GPIO_HIGH);
        pwm_set_duty(RIGHT_PWM, duty);
    }
    else
    {
        gpio_set_level(RIGHT_IN1, GPIO_LOW);
        gpio_set_level(RIGHT_IN2, GPIO_LOW);
        pwm_set_duty(RIGHT_PWM, 0);
    }
}


void motor_control(int16 left_pwm, int16 right_pwm)
{
    motor_set_left(left_pwm);
    motor_set_right(right_pwm);
}


void motor_stop(void)
{
    motor_set_left(0);
    motor_set_right(0);
}
