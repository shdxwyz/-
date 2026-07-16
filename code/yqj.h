#ifndef CODE_YQJ_H_
#define CODE_YQJ_H_

#include "zf_common_headfile.h"
#include "pid.h"


// ADC 小于该阈值时认为传感器压到白线。
// 速度提高后需要更早触发转弯，因此阈值可以适当调高。
#define YQJ_TURN_TRIGGER_ADC_VALUE      (800)

// ==================== 陀螺仪转弯参数 ====================

// 直角转弯目标角度（度）
#define YQJ_TURN_TARGET_ANGLE          (90.0f)

// 转弯完成允许的角度误差（度），防止过冲
#define YQJ_TURN_ANGLE_TOLERANCE       (5.0f)

// 转弯完成时角速度阈值（°/s），角速度低于此值且角度在目标范围内才算稳定完成
#define YQJ_TURN_GYRO_STABLE_THRESHOLD  (1500.0f)//未用

// yaw 异常或车辆堵转时的转向超时保护（ms）
#define YQJ_TURN_TIMEOUT_MS             (2000u)

// ==================== 角度环 PID 参数 ====================

// 角度环 PID 最大输出（对应左右轮速度差，单位：m/s）
#define ANGLE_PID_MAX_OUT              (3.5f)
#define ANGLE_PID_MAX_IOUT             (1.0f)

// 角度环 PID 参数（角度误差 → 速度差）
#define ANGLE_KP                       (0.01f)
#define ANGLE_KI                       (0.00f)
#define ANGLE_KD                       (0.00f)

// ==================== 执行状态 ====================

typedef enum
{
    YQJ_STATE_LINE = 0,       // 正常巡线，同时只判断当前 flag 对应的触发条件。
    YQJ_STATE_DELAY,          // 条件触发后的延时阶段，仍然保持正常巡线。
    YQJ_STATE_RUN,            // 正在执行当前 flag 对应的动作。
    YQJ_STATE_LOCK            // 动作结束后的自锁阶段，继续巡线但不判断新条件。
} yqj_state_enum;


// ==================== 全局状态 ====================

extern uint16 yqj_flag;
extern yqj_state_enum yqj_state;
extern uint8 yqj_action_trigger;
extern uint32 yqj_state_start_time;
extern int32 yqj_lock_start_count;

// 角度环相关变量
extern volatile float yqj_integrated_angle; // 原始 Z 轴角速度积分得到的相对转角（度）
extern PidTypeDef yqj_angle_pid;         // 角度环 PID
extern float yqj_angle_pid_output;       // 角度环 PID 输出（速度差，m/s）


// ==================== 对外接口 ====================

void yqj_init(float pid_period_s, float encoder_count_per_meter);

uint8 yqj_left_turn_trigger(const uint16 adc_value[]);
uint8 yqj_right_turn_trigger(const uint16 adc_value[]);
uint8 yqj_dianzu_trigger(const uint16 adc_value[]);
uint8 yqj_dianyuan_trigger(const uint16 adc_value[]);
uint8 yqj_xianquandianzu_trigger(const uint16 adc_value[]);
uint8 yqj_kaiguang1_0trigger(const uint16 adc_value[]);
uint8 yqj_kaiguang0_1trigger(const uint16 adc_value[]);
uint8 yqj_kaiguang_trigger(const uint16 adc_value[]);
uint8 yqj_erjiguan_trigger(const uint16 adc_value[]);
uint8 yqj_sanjiguan1_2trigger(const uint16 adc_value[]);
uint8 yqj_sanjiguan2_1trigger(const uint16 adc_value[]);
uint8 yqj_sanjiguan0_1trigger(const uint16 adc_value[]);
uint8 yqj_sanjiguan0_2trigger(const uint16 adc_value[]);
uint8 yqj_sanjiguan1_0trigger(const uint16 adc_value[]);
uint8 yqj_sanjiguan2_0trigger(const uint16 adc_value[]);
uint8 yqj_double_trigger(const uint16 adc_value[]);
uint8 yqj_ldiangan_trigger(const uint16 adc_value[]);
uint8 yqj_rdiangan_trigger(const uint16 adc_value[]);
uint8 yqj_xianquan_trigger(const uint16 adc_value[]);
uint8 yqj_erji_trigger(const uint16 adc_value[]);
uint8 yqj_dianchi_trigger(const uint16 adc_value[]);
uint8 yqj_feimen_trigger(const uint16 adc_value[]);
uint8 yqj_dianrong_trigger(const uint16 adc_value[]);
uint8 yqj_time_reached(uint32 start_time, uint32 duration_ms);

void yqj_start_case(uint8 action_trigger);
void yqj_start_lock(int32 encoder_total_sum);
void yqj_finish_case(void);
uint8 yqj_lock_done(int32 encoder_total_sum, uint32 lock_ms, float lock_distance_m);
uint8 yqj_turn_target_reached(void);
void yqj_apply_action(float turn_base_speed, float *left_target_count, float *right_target_count);

// 角度环相关函数
void yqj_angle_pid_init(void);
void yqj_angle_pid_reset(void);
float yqj_angle_pid_calc(float target_angle, float current_angle);
void yqj_integrate_gyro_z(float gyro_z_dps, float dt_s);

uint16 yqj_get_flag(void);
void yqj_set_flag(uint16 flag);
yqj_state_enum yqj_get_state(void);
uint8 yqj_get_action_trigger(void);

#endif
