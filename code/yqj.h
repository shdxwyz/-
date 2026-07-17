#ifndef CODE_YQJ_H_
#define CODE_YQJ_H_

#include "zf_common_headfile.h"

// 元器件顺序与特殊动作模块。
// 状态机流程：LINE 等待触发 -> DELAY 延时 -> RUN 执行 -> LOCK 自锁 -> flag 加 1。
// 所有 ADC 触发函数均接收完整 adc_value[0]~[14]。

// ADC 小于该阈值时认为传感器压到白线。
// 速度提高后需要更早触发转弯，因此阈值可以适当调高。
#define YQJ_TURN_TRIGGER_ADC_VALUE      (500)

// ==================== 执行状态 ====================

typedef enum
{
    YQJ_STATE_LINE = 0,       // 正常巡线，同时只判断当前 flag 对应的触发条件。
    YQJ_STATE_DELAY,          // 条件触发后的延时阶段，仍然保持正常巡线。
    YQJ_STATE_RUN,            // 正在执行当前 flag 对应的动作。
    YQJ_STATE_LOCK            // 动作结束后的自锁阶段，继续巡线但不判断新条件。
} yqj_state_enum;


// ==================== 全局状态 ====================

extern uint16 yqj_flag;                    // 当前等待或执行的 case 编号。
extern yqj_state_enum yqj_state;            // 当前 LINE/DELAY/RUN/LOCK 状态。
extern uint8 yqj_action_trigger;            // 1 表示 RUN 阶段需要覆盖普通巡线目标。
extern uint32 yqj_state_start_time;         // 进入当前状态时的 system_getval() 计时值。
extern int32 yqj_delay_start_count;         // 进入 DELAY 时的左右编码器总计数之和。
extern int32 yqj_lock_start_count;          // 进入 LOCK 时的左右编码器总计数之和。


// ==================== 对外接口 ====================

// 初始化状态机以及速度、距离换算参数。
void yqj_init(float pid_period_s, float encoder_count_per_meter);

// 元器件和转弯触发条件：返回 1 表示对应 ADC 组合已达阈值。
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

// 判断从 start_time 起是否已过 duration_ms；duration_ms=0 时立即返回 1。
uint8 yqj_time_reached(uint32 start_time, uint32 duration_ms);

// 状态机切换以及延迟、自锁判断。
void yqj_start_case(uint8 action_trigger, int32 encoder_total_sum);
void yqj_start_lock(int32 encoder_total_sum);
void yqj_finish_case(void);
// delay_ms 和 delay_distance_m 必须同时达到；delay_distance_m 指左右轮距离之和。
uint8 yqj_delay_done(int32 encoder_total_sum, uint32 delay_ms, float delay_distance_m);
// lock_ms 和 lock_distance_m 必须同时达到；lock_distance_m 指左右轮距离之和。
uint8 yqj_lock_done(int32 encoder_total_sum, uint32 lock_ms, float lock_distance_m);

// RUN 阶段把 case 设置的左右轮速度转换为速度环目标计数。
void yqj_apply_action(float left_speed_mps,
                      float right_speed_mps,
                      float *left_target_count,
                      float *right_target_count);

uint16 yqj_get_flag(void);
void yqj_set_flag(uint16 flag);
yqj_state_enum yqj_get_state(void);
uint8 yqj_get_action_trigger(void);

#endif
