#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "zf_common_headfile.h"

// DRV8701 电机驱动接口。
// 每路电机使用一个方向引脚和一个 PWM 引脚。

// 左电机：P21_4 控制方向，P20_8 输出 PWM。
#define LEFT_IN         P21_4
#define LEFT_PWM        ATOM0_CH7_P20_8

// 右电机：P21_5 控制方向，P20_9 输出 PWM。
#define RIGHT_IN        P21_5
#define RIGHT_PWM       ATOM1_CH5_P20_9

// 电机 PWM 载波频率，单位 Hz。
#define MOTOR_PWM_FREQ  17000

// 初始化方向引脚和 PWM，上电时占空比为 0。
void motor_init(void);

// 设置单路电机：pwm > 0 正转，pwm < 0 反转，pwm = 0 停止。
void motor_set_left(int16 pwm);
void motor_set_right(int16 pwm);

// 同时更新两路电机 PWM。
void motor_control(int16 left_pwm, int16 right_pwm);

// 普通停车：PWM 清零，之后仍可以重新启动。
void motor_stop(void);

// 紧急停车：锁存停车状态，重新上电前拒绝所有非零 PWM。
void motor_emergency_stop(void);

// 返回紧急停车是否已锁存。
uint8 motor_emergency_is_latched(void);

#endif
