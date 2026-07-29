#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "zf_common_headfile.h"

// ==================== DRV8701 actual pin map ====================

// ����
#define LEFT_IN         P21_4
#define LEFT_PWM        ATOM0_CH7_P20_8

// �ҵ��
#define RIGHT_IN        P21_5
#define RIGHT_PWM       ATOM1_CH5_P20_9

#define MOTOR_PWM_FREQ  17000

// 锁存急停总开关：0=关闭全部锁存急停，1=启用。
// 调试完成后应恢复为 1，避免传感器故障时电机持续运行。
#define MOTOR_LATCHED_STOP_ENABLE  (0u)

void motor_init(void);

void motor_set_left(int16 pwm);
void motor_set_right(int16 pwm);

void motor_control(int16 left_pwm, int16 right_pwm);
void motor_stop(void);
void motor_emergency_stop(void);
uint8 motor_emergency_is_latched(void);

#endif
