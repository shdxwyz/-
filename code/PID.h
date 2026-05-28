#ifndef __PID_H__
#define __PID_H__

#include "zf_common_headfile.h"

enum PID_MODE { PID_POSITION = 0, PID_DELTA };

typedef struct {
  uint8_t mode;

  float Kp;
  float Ki;
  float Kd;

  float max_out;
  float max_iout;

  float set;
  float fdb;

  float out;
  float Pout;
  float Iout;
  float Dout;
  float Dbuf[3];
  float error[3];

} PidTypeDef;
extern void PID_Init(PidTypeDef *pid, uint8_t mode, float maxout, float max_iout, float kp, float ki, float kd);
extern float PID_Calc(PidTypeDef *pid, float ref, float set);
extern void PID_clear(PidTypeDef *pid);

#endif  // !__PID_H__
