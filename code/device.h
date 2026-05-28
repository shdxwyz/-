#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "zf_common_headfile.h"

// ==================== TB6612 实际引脚分配 ====================

// 左电机
#define LEFT_IN1        P20_6
#define LEFT_IN2        P20_7
#define LEFT_PWM        ATOM1_CH5_P20_9

// 右电机
#define RIGHT_IN1       P21_4
#define RIGHT_IN2       P21_5
#define RIGHT_PWM       ATOM0_CH7_P20_8

#define MOTOR_PWM_FREQ  50000

void motor_init(void);

void motor_set_left(int16 pwm);
void motor_set_right(int16 pwm);

void motor_control(int16 left_pwm, int16 right_pwm);
void motor_stop(void);

#endif
