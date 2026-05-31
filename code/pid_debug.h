#ifndef __PID_DEBUG_H__
#define __PID_DEBUG_H__

#include "zf_common_headfile.h"
#include "zf_driver_timer.h"
#include "pid.h"

/*
 * ============================================================
 * PID 调试助手
 * ============================================================
 * 功能：
 *   1. 通过 SeekFree Assistant 上位机虚拟示波器实时显示：
 *      - 左轮实际速度 vs 目标速度
 *      - 右轮实际速度 vs 目标速度
 *      - PID 输出 PWM 值
 *   2. 通过 SeekFree Assistant 上位机在线调参：
 *      - 通道 0: 左轮 Kp
 *      - 通道 1: 左轮 Ki
 *      - 通道 2: 左轮 Kd
 *      - 通道 3: 右轮 Kp
 *      - 通道 4: 右轮 Ki
 *      - 通道 5: 右轮 Kd
 *      - 通道 6: 基础目标速度 (m/s)
 *      - 通道 7: 巡线转向强度 (XUNJI_LINE_TURN_KP)
 *   3. 支持一键保存/恢复 PID 参数到 Flash
 *
 * 使用方法：
 *   1. 在 cpu0_main.c 中包含此头文件
 *   2. 在主循环中调用 pid_debug_update() 即可
 *   3. 打开 SeekFree Assistant 上位机连接查看
 * ============================================================
 */

// ==================== 调试开关 ====================
// 注释掉此行可关闭调试功能，恢复为原始代码
#define PID_DEBUG_ENABLE

// ==================== 示波器通道定义 ====================
#define OSC_CH_LEFT_ACTUAL      0   // 左轮实际速度（编码器计数/20ms）
#define OSC_CH_LEFT_TARGET      1   // 左轮目标速度（编码器计数/20ms）
#define OSC_CH_RIGHT_ACTUAL     2   // 右轮实际速度（编码器计数/20ms）
#define OSC_CH_RIGHT_TARGET     3   // 右轮目标速度（编码器计数/20ms）
#define OSC_CH_LEFT_PWM         4   // 左轮 PWM 输出
#define OSC_CH_RIGHT_PWM        5   // 右轮 PWM 输出
#define OSC_CH_LEFT_ERROR       6   // 左轮误差
#define OSC_CH_RIGHT_ERROR      7   // 右轮误差

// ==================== 参数调节通道定义 ====================
// SeekFree Assistant 上位机参数调节通道对应关系
// 上位机发送的参数会更新到 seekfree_assistant_parameter[channel]
// 同时 seekfree_assistant_parameter_update_flag[channel] 会被置1
#define PARAM_CH_LEFT_KP        0   // 左轮 Kp
#define PARAM_CH_LEFT_KI        1   // 左轮 Ki
#define PARAM_CH_LEFT_KD        2   // 左轮 Kd
#define PARAM_CH_RIGHT_KP       3   // 右轮 Kp
#define PARAM_CH_RIGHT_KI       4   // 右轮 Ki
#define PARAM_CH_RIGHT_KD       5   // 右轮 Kd
#define PARAM_CH_BASE_SPEED     6   // 基础目标速度 (m/s)
#define PARAM_CH_TURN_KP        7   // 巡线转向强度

// ==================== 默认 PID 参数（与 cpu0_main.c 保持一致） ====================
#define PID_DEBUG_DEFAULT_LEFT_KP       (20.0f)
#define PID_DEBUG_DEFAULT_LEFT_KI       (0.1f)
#define PID_DEBUG_DEFAULT_LEFT_KD       (0.0f)
#define PID_DEBUG_DEFAULT_RIGHT_KP      (20.0f)
#define PID_DEBUG_DEFAULT_RIGHT_KI      (0.1f)
#define PID_DEBUG_DEFAULT_RIGHT_KD      (0.0f)
#define PID_DEBUG_DEFAULT_BASE_SPEED    (0.3f)
#define PID_DEBUG_DEFAULT_TURN_KP       (0.01f)

// ==================== 示波器发送间隔 ====================
#define PID_DEBUG_SEND_INTERVAL_MS      (50)    // 50ms 发送一次示波器数据

// ==================== 外部变量声明 ====================
// 这些变量在 cpu0_main.c 中定义，这里引用
extern volatile int16  left_encoder_count;
extern volatile int16  right_encoder_count;
extern volatile float  left_target_count;
extern volatile float  right_target_count;
extern volatile float  left_base_pwm;
extern volatile float  right_base_pwm;
extern PidTypeDef      left_speed_pid;
extern PidTypeDef      right_speed_pid;
extern volatile float  car_distance_m;

// ==================== 函数声明 ====================

// 山外多功能调试助手响应曲线绘制函数
// data: 数据指针（float数组）
// size: 数据总字节数，sizeof(data)
void Draw_ResponseCurve(void *data, uint32 size);

// PID 调试初始化：设置参数调节通道的初始值
void pid_debug_init(void);

// PID 调试更新：在主循环中调用，处理参数更新和示波器发送
void pid_debug_update(void);

// 获取当前参数值（供外部读取调参后的值）
float pid_debug_get_param(uint8 channel);

// 检查参数是否被更新过
uint8 pid_debug_is_param_updated(uint8 channel);

// 清除参数更新标志
void pid_debug_clear_param_flag(uint8 channel);

#endif // __PID_DEBUG_H__
