
#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "zf_common_headfile.h"

// ==================== DRV8701 actual pin map ====================

// 锟斤拷锟斤拷
#define LEFT_IN         P21_4
#define LEFT_PWM        ATOM0_CH7_P20_8

// 锟揭碉拷锟�
#define RIGHT_IN        P21_5
#define RIGHT_PWM       ATOM1_CH5_P20_9

#define MOTOR_PWM_FREQ  17000

// 閿佸瓨鎬ュ仠鎬诲紑鍏筹細0=鍏抽棴鍏ㄩ儴閿佸瓨鎬ュ仠锛�1=鍚敤銆�
// 璋冭瘯瀹屾垚鍚庡簲鎭㈠涓� 1锛岄伩鍏嶄紶鎰熷櫒鏁呴殰鏃剁數鏈烘寔缁繍琛屻��
#define MOTOR_LATCHED_STOP_ENABLE  (0u)

void motor_init(void);

void motor_set_left(int16 pwm);
void motor_set_right(int16 pwm);

void motor_control(int16 left_pwm, int16 right_pwm);
void motor_stop(void);
void motor_emergency_stop(void);
uint8 motor_emergency_is_latched(void);

#endif
