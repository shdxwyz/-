#ifndef CODE_YQJ_H_
#define CODE_YQJ_H_

#include "zf_common_headfile.h"


// ADC 小于这个值认为压到白线，后面实车调试时可以改
#define YQJ_TURN_TRIGGER_ADC_VALUE      (500)



// ==================== 执行状态 ====================

typedef enum
{
    YQJ_STATE_LINE = 0,       // 正常巡线，同时只判断当前 flag 的条件
    YQJ_STATE_DELAY,          // 条件触发后的延时，仍然正常巡线
    YQJ_STATE_RUN,            // 正在执行当前 flag 对应动作
    YQJ_STATE_LOCK            // 动作结束后的自锁，仍然正常巡线，但不判断新条件
} yqj_state_enum;


// ==================== 全局状态 ====================

extern uint16 yqj_flag;
extern yqj_state_enum yqj_state;
extern uint8 yqj_action_trigger;
extern uint32 yqj_state_start_time;
extern int32 yqj_lock_start_count;


// ==================== 对外接口 ====================

void yqj_init(float pid_period_s, float encoder_count_per_meter);

uint8 yqj_left_turn_trigger(const uint16 adc_value[]);
uint8 yqj_right_turn_trigger(const uint16 adc_value[]);
uint8 yqj_dianzu_trigger(const uint16 adc_value[]);
uint8 yqj_dianyuan_trigger(const uint16 adc_value[]);
uint8 yqj_kaiguang1_0trigger(const uint16 adc_value[]);
uint8 yqj_kaiguang0_1trigger(const uint16 adc_value[]);
uint8 yqj_erjiguan_trigger(const uint16 adc_value[]);
uint8 yqj_sanjiguan1_2trigger(const uint16 adc_value[]);
uint8 yqj_sanjiguan2_1trigger(const uint16 adc_value[]);
uint8 yqj_sanjiguan0_1trigger(const uint16 adc_value[]);
uint8 yqj_sanjiguan0_2trigger(const uint16 adc_value[]);
uint8 yqj_sanjiguan1_0trigger(const uint16 adc_value[]);
uint8 yqj_sanjiguan2_0trigger(const uint16 adc_value[]);
uint8 yqj_time_reached(uint32 start_time, uint32 duration_ms);

void yqj_start_case(uint8 action_trigger);
void yqj_start_lock(int32 encoder_total_sum);
void yqj_finish_case(void);
uint8 yqj_lock_done(int32 encoder_total_sum, uint32 lock_ms, float lock_distance_m);
void yqj_apply_action(float left_speed_mps, float right_speed_mps, float *left_target_count, float *right_target_count);

uint16 yqj_get_flag(void);
void yqj_set_flag(uint16 flag);
yqj_state_enum yqj_get_state(void);
uint8 yqj_get_action_trigger(void);

#endif
