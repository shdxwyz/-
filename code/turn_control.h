#ifndef CODE_TURN_CONTROL_H_
#define CODE_TURN_CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"

// ==================== yaw 转向参数 ====================

// 角度环以 90 度为控制目标，保证到达停止角前仍有足够差速。
#define TURN_CONTROL_TARGET_ANGLE_DEG          (90.0f)

// 从转向起始 yaw 算起，相对 yaw 达到 85 度时结束转向。
#define TURN_CONTROL_STOP_YAW_ANGLE_DEG        (90.0f)

// yaw 异常或车辆堵转时的转向超时保护（ms）。
#define TURN_CONTROL_TIMEOUT_MS                (500u)

// ==================== 角度环 PID 参数 ====================

// PID 输出为左右轮速度差，单位 m/s。
#define TURN_CONTROL_ANGLE_PID_MAX_OUT         (6.5f)
#define TURN_CONTROL_ANGLE_PID_MAX_IOUT        (2.0f)
#define TURN_CONTROL_ANGLE_KP                  (0.016f)
#define TURN_CONTROL_ANGLE_KI                  (0.00f)
#define TURN_CONTROL_ANGLE_KD                  (0.00f)

// 左右轮最终目标速度的安全限幅，单位 m/s。
#define TURN_CONTROL_WHEEL_SPEED_LIMIT_MPS     (5.0f)

// ==================== 串口调试量 ====================

extern volatile float turn_control_angle_deg;
extern float turn_control_angle_pid_output;

// ==================== 对外接口 ====================

void turn_control_init(float pid_period_s, float encoder_count_per_meter);
void turn_control_stop(void);
uint8 turn_control_target_reached(void);

// turn_base_speed：正数左转，负数右转，绝对值为向前基础速度。
void turn_control_apply(float turn_base_speed,
                        float *left_target_count,
                        float *right_target_count);

// 在 IMU 数据就绪中断中调用。
void turn_control_update_yaw(float yaw_deg);

#endif
