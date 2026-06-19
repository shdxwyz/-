#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "zf_common_headfile.h"

// ==================== DRV8701 actual pin map ====================

// 左电机
#define LEFT_IN         P21_4
#define LEFT_PWM        ATOM0_CH7_P20_8

// 右电机
#define RIGHT_IN        P21_5
#define RIGHT_PWM       ATOM1_CH5_P20_9

#define MOTOR_PWM_FREQ  17000

void motor_init(void);

void motor_set_left(int16 pwm);
void motor_set_right(int16 pwm);

void motor_control(int16 left_pwm, int16 right_pwm);
void motor_stop(void);

#endif
