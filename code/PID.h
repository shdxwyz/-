#ifndef __PID_H__
#define __PID_H__

#include "zf_common_headfile.h"

// PID_POSITION：位置式，每次计算完整输出。
// PID_DELTA：增量式，每次计算输出变化量并累加。
enum PID_MODE { PID_POSITION = 0, PID_DELTA };

// 每个闭环都需要独立的 PidTypeDef，否则历史误差和积分会相互干扰。
typedef struct {
  uint8_t mode;       // 计算模式：PID_POSITION 或 PID_DELTA。

  float Kp;           // 比例系数，直接响应当前误差。
  float Ki;           // 积分系数，用于消除稳态误差。
  float Kd;           // 微分系数，根据误差变化抑制过冲。

  float max_out;      // PID 总输出的最大绝对值。
  float max_iout;     // 积分项的最大绝对值，防止积分饱和。

  float set;          // 最近一次计算的目标值。
  float fdb;          // 最近一次计算的实际反馈值。

  float out;          // 最终 PID 输出。
  float Pout;         // 比例项输出。
  float Iout;         // 积分项输出。
  float Dout;         // 微分项输出。
  float Dbuf[3];      // 微分项的历史缓冲。
  float error[3];     // [0] 当前误差，[1] 上次误差，[2] 上上次误差。

} PidTypeDef;

// 设置 PID 参数和限幅，同时清空所有运行状态。
extern void PID_Init(PidTypeDef *pid, uint8_t mode, float maxout, float max_iout, float kp, float ki, float kd);

// ref 是实际值，set 是目标值，内部按 error = set - ref 计算。
extern float PID_Calc(PidTypeDef *pid, float ref, float set);

// 清空误差、积分和输出，但保留 Kp/Ki/Kd 和限幅参数。
extern void PID_clear(PidTypeDef *pid);

#endif  // !__PID_H__
