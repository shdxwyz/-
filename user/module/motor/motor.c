#include "motor.h"
#include <stdint.h>       // 添加标准整数类型头文件
#include <string.h>       // 添加字符串处理头文件
#include "../elrs/elrs.h" // 包含 ELRS_Data 的完整定义

/**
************************************************************************************************
* @brief    电机控制任务，运动算法
* @param    None
* @return   None
* @author   创源启明		2025.12.28
************************************************************************************************
**/
// 外部变量声明
WINGS_DATA Wings_Data;

/************ 运动速度变量定义 *************/
int motor_1_set_pwm;
int motor_2_set_pwm;
int motor_3_set_pwm;
int motor_4_set_pwm;

/************** 电机限幅设置 *************/
const int16_t MOTOR_MAX_OUT = 20000; // PWM最大输出值
const int16_t MOTOR_MIN_OUT = 0;     // PWM最小输出值

/**
 * @brief  4电机直接转速控制（凸轮连杆机构仿生蝴蝶）
 * @param  throttle: 油门值 (0-1000) 控制基础转速
 * @param  yaw: 偏航值 (-400 to 400) 控制左右差速（转向）
 * @param  roll: 横滚值 (-80 to 80) 控制前后差速（俯仰）
 * @retval None
 * @note   控制逻辑：
 *         1. 油门控制基础转速（所有电机同速）
 *         2. 偏航控制左右差速（左转/右转）
 *         3. 横滚控制前后差速（前倾/后倾）
 *         电机布局（凸轮连杆机构，电机持续正转）：
 *         左前(M1)  右前(M2)
 *         左后(M3)  右后(M4)
 *         
 *         由于是凸轮连杆机构，电机只需正转（0~MOTOR_MAX_OUT），
 *         通过改变转速差实现姿态控制。
 */
void Motor_Direct_Control(int16_t throttle, int16_t yaw, int16_t roll)
{
  // 油门值范围 0-1000，映射到PWM范围 0-MOTOR_MAX_OUT
  // 油门为0时电机停转，油门最大时全速
  int16_t base_speed = (throttle * MOTOR_MAX_OUT) / 1000;

  // 偏航差速：yaw范围 -400~400，映射到 -MOTOR_MAX_OUT/4 ~ MOTOR_MAX_OUT/4
  int16_t yaw_diff = (yaw * MOTOR_MAX_OUT) / (4 * 1000);

  // 横滚差速：roll范围 -80~80，映射到 -MOTOR_MAX_OUT/4 ~ MOTOR_MAX_OUT/4
  int16_t roll_diff = (roll * MOTOR_MAX_OUT) / (4 * 1000);

  // 计算各电机速度（凸轮机构只需正转，但允许差速导致某侧减速）
  // 左前电机: 基础速度 - 偏航差速(左转减速) + 横滚差速(前倾加速)
  motor_1_set_pwm = base_speed - yaw_diff + roll_diff;

  // 右前电机: 基础速度 + 偏航差速(右转减速) + 横滚差速(前倾加速)
  motor_2_set_pwm = base_speed + yaw_diff + roll_diff;

  // 左后电机: 基础速度 - 偏航差速(左转减速) - 横滚差速(后倾减速)
  motor_3_set_pwm = base_speed - yaw_diff - roll_diff;

  // 右后电机: 基础速度 + 偏航差速(右转减速) - 横滚差速(后倾减速)
  motor_4_set_pwm = base_speed + yaw_diff - roll_diff;

  // 限幅处理（凸轮机构电机只需正转，下限为0）
  if (motor_1_set_pwm < 0) motor_1_set_pwm = 0;
  if (motor_1_set_pwm > MOTOR_MAX_OUT) motor_1_set_pwm = MOTOR_MAX_OUT;
  
  if (motor_2_set_pwm < 0) motor_2_set_pwm = 0;
  if (motor_2_set_pwm > MOTOR_MAX_OUT) motor_2_set_pwm = MOTOR_MAX_OUT;
  
  if (motor_3_set_pwm < 0) motor_3_set_pwm = 0;
  if (motor_3_set_pwm > MOTOR_MAX_OUT) motor_3_set_pwm = MOTOR_MAX_OUT;
  
  if (motor_4_set_pwm < 0) motor_4_set_pwm = 0;
  if (motor_4_set_pwm > MOTOR_MAX_OUT) motor_4_set_pwm = MOTOR_MAX_OUT;

  // 设置PWM输出
  Set_Pwm(motor_1_set_pwm, motor_2_set_pwm, motor_3_set_pwm, motor_4_set_pwm);
}

/**
 * @brief  遥控器数据到电机控制
 * @param  elrs_data: ELRS接收机数据
 * @retval None
 */
void Remote_To_Motor_Control(void *elrs_data_void)
{
  ELRS_Data *elrs_data = (ELRS_Data *)elrs_data_void;
  if (elrs_data == NULL)
    return;

  // 如果开关关闭，停止所有电机
  if (elrs_data->Switch == 0)
  {
    Set_Pwm(0, 0, 0, 0);
    return;
  }

  // 获取遥控器通道值
  // Throttle: 0-1000 (已映射)
  // Yaw: -1000 to 1000
  // Roll: -1000 to 1000
  int16_t throttle = elrs_data->Throttle;
  int16_t yaw = elrs_data->Yaw;
  int16_t roll = elrs_data->Roll;

  // 调用直接控制函数
  Motor_Direct_Control(throttle, yaw, roll);
}

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : motor1_out - 电机1输出值, motor2_out - 电机2输出值
          motor3_out - 电机3输出值, motor4_out - 电机4输出值
Output  : none
函数功能：赋值给PWM寄存器，控制四个车轮转速与方向及舵机角度
入口参数：motor1_out - 电机1的输出值
          motor2_out - 电机2的输出值
          motor3_out - 电机3的输出值
          motor4_out - 电机4的输出值
返回  值：无
**************************************************************************/

// 分支消除的 int16 绝对值（两补码）
static inline uint16_t abs16_fast(int16_t x)
{
  int16_t m = x >> 15;            // x<0 → -1；x>=0 → 0
  return (uint16_t)((x ^ m) - m); // 等价于 abs(x)
}

void Set_Pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
  // -------- Motor 1 (左前) --------
  uint16_t pwm1 = abs16_fast(m1);
  uint16_t mask1 = (uint16_t)-(m1 > 0); // >0 → 0xFFFF；<=0 → 0x0000
  PWM_M1_2 = (uint16_t)(pwm1 & mask1);  // 正转：CH2=pwm
  PWM_M1_1 = (uint16_t)(pwm1 & ~mask1); // 反/零：CH1=pwm

  // -------- Motor 2 (右前) --------
  uint16_t pwm2 = abs16_fast(m2);
  uint16_t mask2 = (uint16_t)-(m2 > 0);
  PWM_M2_2 = (uint16_t)(pwm2 & mask2);
  PWM_M2_1 = (uint16_t)(pwm2 & ~mask2);

  // -------- Motor 3 (左后) --------
  uint16_t pwm3 = abs16_fast(m3);
  uint16_t mask3 = (uint16_t)-(m3 > 0);
  PWM_M3_2 = (uint16_t)(pwm3 & mask3);
  PWM_M3_1 = (uint16_t)(pwm3 & ~mask3);

  // -------- Motor 4 (右后) --------
  uint16_t pwm4 = abs16_fast(m4);
  uint16_t mask4 = (uint16_t)-(m4 > 0);
  PWM_M4_2 = (uint16_t)(pwm4 & mask4);
  PWM_M4_1 = (uint16_t)(pwm4 & ~mask4);
}

/**
 * @brief  电机初始化（兼容旧接口）
 * @retval None
 */
void Chassis_PID_Init(void)
{
  // 不再使用PID，保持空函数以兼容旧代码
}

/**
 * @brief  电机控制（兼容旧接口）
 * @retval None
 */
void Motor_PID_Control(void)
{
  // 不再使用PID控制，调用新的直接控制函数
  // 注意：需要在main.c中修改为调用Remote_To_Motor_Control
}
