#include "zf_common_headfile.h"
#include "isr_config.h"
#include "device.h"
#include "pid.h"
#include "../code/yqj.h"
#include "../code/xunji.h"
#include "../code/pid_debug.h"

#pragma section all "cpu0_dsram"

// ==================== PIT 涓庣紪鐮佸櫒閰嶇疆 ====================

#define PIT0                            (CCU60_CH0)

// 宸︾紪鐮佸櫒
#define LEFT_ENCODER                    (TIM2_ENCODER)
#define LEFT_ENCODER_PULSE              (TIM2_ENCODER_CH1_P33_7)
#define LEFT_ENCODER_DIR                (TIM2_ENCODER_CH2_P33_6)

// 鍙崇紪鐮佸櫒
#define RIGHT_ENCODER                   (TIM4_ENCODER)
#define RIGHT_ENCODER_PULSE             (TIM4_ENCODER_CH1_P02_8)
#define RIGHT_ENCODER_DIR               (TIM4_ENCODER_CH2_P00_9)


// ==================== 閫熷害 PID 鍙傛暟 ====================

// 瀹炴祴锛氬皬杞﹁蛋 1 绫崇害 12106 涓紪鐮佸櫒璁℃暟
#define ENCODER_COUNT_PER_METER         (12106.0f)

// 鐩爣鍩虹閫熷害锛�0.3 m/s
#define TARGET_SPEED_MPS                (0.3f)

// PID 鍛ㄦ湡锛�20ms
#define PID_PERIOD_MS                   (20)
#define PID_PERIOD_S                    (0.02f)

// 20ms 鍐呭熀纭�鐩爣璁℃暟锛�0.3 * 0.02 * 12106 鈮� 72.6
#define BASE_TARGET_COUNT               (TARGET_SPEED_MPS * PID_PERIOD_S * ENCODER_COUNT_PER_METER)

// PID 杈撳嚭鑼冨洿
#define SPEED_PID_MAX_OUT               (10000.0f)
#define SPEED_PID_MAX_IOUT              (6000.0f)

// 閫熷害 PID 鍙傛暟
#define SPEED_KP                        (50.0f)
#define SPEED_KI                        (5.0f)
#define SPEED_KD                        (0.0f)


// ==================== 宸＄嚎鍙傛暟 ====================

#define SENSOR_NUM                      (XUNJI_SENSOR_NUM)


// ==================== ADC 鍙橀噺 ====================

// 浠庡乏鍒板彸锛欰1 A2 A3 A4 A5 A6 A7 A8 A10 A11
uint16 adc_value[SENSOR_NUM];

adc_channel_enum adc_list[SENSOR_NUM] =
{
    ADC0_CH1_A1,
    ADC0_CH2_A2,
    ADC0_CH3_A3,
    ADC0_CH4_A4,
    ADC0_CH5_A5,
    ADC0_CH6_A6,
    ADC0_CH7_A7,
    ADC0_CH8_A8,
    ADC0_CH10_A10,
    ADC0_CH11_A11
};

// ==================== 缂栫爜鍣ㄤ笌 PID 鍙橀噺 ====================

// 20ms 鍐呯紪鐮佸櫒澧為噺锛岀敤浜庨�熷害 PID
volatile int16 left_encoder_count = 0;
volatile int16 right_encoder_count = 0;

// 杞欢绱鎬昏鏁帮紝鐢ㄤ簬绠楁�昏矾绋�
volatile int32 left_encoder_total = 0;
volatile int32 right_encoder_total = 0;

// 灏忚溅鎬昏矾绋嬶紝鍗曚綅 m
volatile float car_distance_m = 0.5f;

// 寰抗绠楀嚭鏉ョ殑宸﹀彸鐩爣璁℃暟
volatile float left_target_count = BASE_TARGET_COUNT;
volatile float right_target_count = BASE_TARGET_COUNT;

// PID 杈撳嚭 PWM
volatile float left_base_pwm = 0;
volatile float right_base_pwm = 0;

PidTypeDef left_speed_pid;
PidTypeDef right_speed_pid;


// ==================== 鍑芥暟澹版槑 ====================

void adc_all_init(void);
void adc_all_read(void);

int16 limit_int16(int16 value, int16 min, int16 max);


// ==================== 涓诲嚱鏁� ====================

int core0_main(void)
{
    xunji_result_struct line_result = {0, 0, BASE_TARGET_COUNT, BASE_TARGET_COUNT};

    int16 left_pwm = 0;
    int16 right_pwm = 0;

    uint32 print_count = 0;
    float final_left_target = BASE_TARGET_COUNT;
    float final_right_target = BASE_TARGET_COUNT;

    uint8 yqj_condition = 0;
    uint8 yqj_case_trigger = 0;
    uint32 yqj_delay_ms = 0;
    uint32 yqj_run_ms = 0;
    uint32 yqj_lock_ms = 0;
    float yqj_lock_distance_m = 0.0f;
    float yqj_left_speed_mps = 0.0f;
    float yqj_right_speed_mps = 0.0f;

    clock_init();
    debug_init();

    // ADC 鍒濆鍖�
    adc_all_init();

    // 缂栫爜鍣ㄥ垵濮嬪寲
    encoder_dir_init(LEFT_ENCODER, LEFT_ENCODER_PULSE, LEFT_ENCODER_DIR);
    encoder_dir_init(RIGHT_ENCODER, RIGHT_ENCODER_PULSE, RIGHT_ENCODER_DIR);

    // 鐢垫満鍒濆鍖�
    motor_init();

    // PID 鍒濆鍖�
    PID_Init(&left_speed_pid,
             PID_POSITION,
             SPEED_PID_MAX_OUT,
             SPEED_PID_MAX_IOUT,
             SPEED_KP,
             SPEED_KI,
             SPEED_KD);

    PID_Init(&right_speed_pid,
             PID_POSITION,
             SPEED_PID_MAX_OUT,
             SPEED_PID_MAX_IOUT,
             SPEED_KP,
             SPEED_KI,
             SPEED_KD);

    // 20ms 閫熷害 PID
    pit_ms_init(PIT0, PID_PERIOD_MS);

    cpu_wait_event_ready();
    yqj_init(PID_PERIOD_S, ENCODER_COUNT_PER_METER);

    // PID 璋冭瘯鍔╂墜鍒濆鍖�
    pid_debug_init();

    // ==================== 閫熷害娴嬭瘯鍙橀噺 ====================
    float test_speed = 0.2f;           // 褰撳墠娴嬭瘯閫熷害
    uint32 test_speed_start_ms = 0;    // 閫熷害鍙樺寲璁℃椂
    float test_speed_get = 0.0f;       // 瀹為檯閫熷害锛堢紪鐮佸櫒鎹㈢畻锛�
    float test_speed_set = 0.0f;       // 鐩爣閫熷害

    system_delay_ms(2000);

    while(TRUE)
    {
        // ==================== 閫熷害娴嬭瘯锛氭瘡10绉掑彉鍖栦竴娆� ====================
        if(system_getval_ms() - test_speed_start_ms >= 10000)
        {
            test_speed_start_ms = system_getval_ms();
            test_speed += 0.1f;
            if(test_speed > 0.6f)
            {
                test_speed = 0.2f;
            }
            // 鏇存柊鐩爣閫熷害
            left_target_count = test_speed * PID_PERIOD_S * ENCODER_COUNT_PER_METER;
            right_target_count = left_target_count;
            test_speed_set = test_speed;
            printf("[TEST] 鐩爣閫熷害 = %.2f m/s\r\n", test_speed);
        }

        // 瀹為檯閫熷害锛堢紪鐮佸櫒璁℃暟鎹㈢畻涓� m/s锛�
        test_speed_get = ((float)left_encoder_count + (float)right_encoder_count) / 2.0f
                         / ENCODER_COUNT_PER_METER / PID_PERIOD_S;

        // ==================== 鍙戦�佺ず娉㈠櫒鏁版嵁 ====================
        {
            float send_data[2];
            send_data[0] = test_speed_get;   // 瀹為檯閫熷害
            send_data[1] = test_speed_set;   // 鐩爣閫熷害
            Draw_ResponseCurve(send_data, sizeof(send_data));
        }

        // ==================== 璇诲彇 10 璺� ADC ====================

        adc_all_read();
        // ==================== 宸＄嚎灞� ====================
        // xunji 鍙牴鎹� ADC 璁＄畻鏅�氬贰绾跨洰鏍囷紝涓嶅鐞嗕换浣曠壒娈婂懡浠ゃ��
        xunji_update(adc_value, BASE_TARGET_COUNT, &line_result);

        // ==================== 鍏冨櫒浠堕『搴忓眰 ====================
        // 杩欓噷灏辨槸鎬绘祦绋嬶細姝ｅ父宸＄嚎銆佸垽鏂綋鍓� flag銆佸欢鏃躲�佹墽琛屽姩浣溿�佽嚜閿併�乫lag 鍔犱竴銆�
        yqj_condition = 0;
        yqj_case_trigger = 0;
        yqj_delay_ms = 0;
        yqj_run_ms = 0;
        yqj_lock_ms = 0;
        yqj_lock_distance_m = 0.0f;
        yqj_left_speed_mps = 0.0f;
        yqj_right_speed_mps = 0.0f;

        final_left_target = line_result.left_target_count;
        final_right_target = line_result.right_target_count;

        switch(yqj_flag)
        {
            case 1:
                            // 鐢垫簮
                            yqj_condition = yqj_dianyuan_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 0.3f;
                            yqj_right_speed_mps = 0.3f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 300;
                            yqj_lock_ms = 500;
                            yqj_lock_distance_m = 0.5f;
                            break;

            case 2:
                            // A10 鍜� A11 鍚屾椂灏忎簬 500 鍚庡彸杞��
                            yqj_condition = yqj_right_turn_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 1.2f;
                            yqj_right_speed_mps = -0.2f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 300;
                            yqj_lock_ms = 500;
                            yqj_lock_distance_m = 0.8f;
                            break;

            case 3:
                            // 涓夋瀬绠�2_1
                            yqj_condition = yqj_sanjiguan2_1trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 0.3f;
                            yqj_right_speed_mps = 0.3f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 800;
                            yqj_lock_ms = 500;
                            yqj_lock_distance_m = 0.5f;
                            break;

            case 4:
                            // 鐢甸樆
                            yqj_condition = yqj_dianzu_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 0.3f;
                            yqj_right_speed_mps = 0.3f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 350;
                            yqj_lock_ms = 500;
                            yqj_lock_distance_m = 0.5f;
                            break;
            case 5:
                            // A10 鍜� A11 鍚屾椂灏忎簬 500 鍚庡彸杞��
                            yqj_condition = yqj_right_turn_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 1.2f;
                            yqj_right_speed_mps = -0.2f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 300;
                            yqj_lock_ms = 500;
                            yqj_lock_distance_m = 0.5f;
                            break;

            case 6:
                           // 浜岀骇绠�
                           yqj_condition = yqj_erjiguan_trigger(adc_value);
                           yqj_case_trigger = 1;
                           yqj_left_speed_mps = 0.3f;
                           yqj_right_speed_mps = 0.3f;
                           yqj_delay_ms = 0;
                           yqj_run_ms = 350;
                           yqj_lock_ms = 500;
                           yqj_lock_distance_m = 0.5f;
                           break;

            case 7:
                            // A10 鍜� A11 鍚屾椂灏忎簬 500 鍚庡彸杞��
                            yqj_condition = yqj_right_turn_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 1.2f;
                            yqj_right_speed_mps = -0.2f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 300;
                            yqj_lock_ms = 500;
                            yqj_lock_distance_m = 0.5f;
                            break;

            case 8:
                            // 寮�鍏�0_1
                            yqj_condition = yqj_kaiguang0_1trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 0.3f;
                            yqj_right_speed_mps = 0.3f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 350;
                            yqj_lock_ms = 500;
                            yqj_lock_distance_m = 0.5f;
                            break;

            case 9:
                            // A10 鍜� A11 鍚屾椂灏忎簬 500 鍚庡彸杞��
                            yqj_condition = yqj_right_turn_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 1.2f;
                            yqj_right_speed_mps = -0.2f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 300;
                            yqj_lock_ms = 300;
                            yqj_lock_distance_m = 0.3f;
                            break;

            case 10:
                            // 鐢垫簮
                            yqj_condition = yqj_dianyuan_trigger(adc_value);
                            yqj_case_trigger = 1;
                            yqj_left_speed_mps = 0.3f;
                            yqj_right_speed_mps = 0.3f;
                            yqj_delay_ms = 0;
                            yqj_run_ms = 300;
                            yqj_lock_ms = 500;
                            yqj_lock_distance_m = 0.5f;
                            break;

            default:
                // 鍋滄
                motor_stop();
                system_delay_ms(20000);
                break;
        }

        if(YQJ_STATE_LINE == yqj_state)
        {
            if(yqj_condition)
            {
                yqj_start_case(yqj_case_trigger);
            }
        }
        else if(YQJ_STATE_DELAY == yqj_state)
        {
            if(yqj_time_reached(yqj_state_start_time, yqj_delay_ms))
            {
                yqj_state = YQJ_STATE_RUN;
                yqj_state_start_time = system_getval();
            }
        }
        else if(YQJ_STATE_RUN == yqj_state)
        {
            if(yqj_time_reached(yqj_state_start_time, yqj_run_ms))
            {
                yqj_start_lock(left_encoder_total + right_encoder_total);
            }
        }
        else if(YQJ_STATE_LOCK == yqj_state)
        {
            if(yqj_lock_done(left_encoder_total + right_encoder_total,
                             yqj_lock_ms,
                             yqj_lock_distance_m))
            {
                yqj_finish_case();
            }
        }
        else
        {
            yqj_set_flag(0);
        }

        if(YQJ_STATE_RUN == yqj_state)
        {
            yqj_apply_action(yqj_left_speed_mps, yqj_right_speed_mps, &final_left_target, &final_right_target);
        }

        left_target_count  = final_left_target;
        right_target_count = final_right_target;

        // ==================== PID 璋冭瘯鏇存柊 ====================
        // 鍙戦�佺ず娉㈠櫒鏁版嵁鍒板北澶栧鍔熻兘璋冭瘯鍔╂墜
        // 澶勭悊 SeekFree Assistant 涓婁綅鏈虹殑鍦ㄧ嚎璋冨弬
        pid_debug_update();

        // ==================== 杈撳嚭鐢垫満 ====================
        // PID 鍦� 20ms 涓柇閲屾牴鎹乏鍙崇洰鏍囪鏁拌緭鍑� PWM

        left_pwm  = (int16)left_base_pwm;
        right_pwm = (int16)right_base_pwm;

        left_pwm  = limit_int16(left_pwm,  -PWM_DUTY_MAX, PWM_DUTY_MAX);
        right_pwm = limit_int16(right_pwm, -PWM_DUTY_MAX, PWM_DUTY_MAX);

        motor_control(left_pwm, right_pwm);

        // ==================== 涓插彛璋冭瘯 ====================

        print_count++;
        if(print_count >= 50)
        {
            print_count = 0;

            printf("flag=%d state=%d trigger=%d err=%d turn=%d targetL=%d targetR=%d encL=%d encR=%d totalL=%d totalR=%d dist=%.3f pwmL=%d pwmR=%d ADC:",
                    (int)yqj_get_flag(),
                    (int)yqj_get_state(),
                    (int)yqj_get_action_trigger(),
                    (int)line_result.line_error,
                    (int)line_result.turn_count,
                    (int)left_target_count,
                    (int)right_target_count,
                    left_encoder_count,
                    right_encoder_count,
                    (int)left_encoder_total,
                    (int)right_encoder_total,
                    car_distance_m,
                    left_pwm,
                    right_pwm);

            uint8 i;
            for(i = 0; i < SENSOR_NUM; i++)
            {
                printf(" %d", adc_value[i]);
            }

            printf("\r\n");
        }

        system_delay_ms(2);
    }
}


// ==================== ADC 鍒濆鍖� ====================

void adc_all_init(void)
{
    uint8 i;
    for(i = 0; i < SENSOR_NUM; i++)
    {
        adc_init(adc_list[i], ADC_12BIT);
    }
}


// ==================== ADC 璇诲彇 ====================

void adc_all_read(void)
{
    uint8 i;
    for(i = 0; i < SENSOR_NUM; i++)
    {
        // 10 璺叏閮ㄧ敤浜庡惊杩癸紝3 娆″钩鍧囦繚璇佸搷搴旇緝蹇�
        adc_value[i] = adc_mean_filter_convert(adc_list[i], 3);
    }
}


// ==================== 20ms 閫熷害 PID 涓柇 ====================

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);

    // ==================== 璇诲彇 20ms 鍐呯紪鐮佸櫒澧為噺 ====================
    // 宸︾紪鐮佸櫒鍓嶈繘鏃舵槸璐熸暟锛屾墍浠ュ彇鍙嶅彉姝�
    left_encoder_count = -encoder_get_count(LEFT_ENCODER);

    // 鍙崇紪鐮佸櫒鍓嶈繘鏃舵槸姝ｆ暟
    right_encoder_count = encoder_get_count(RIGHT_ENCODER);

    // ==================== 璇诲畬绔嬪埢娓呯┖纭欢缂栫爜鍣� ====================

    encoder_clear_count(LEFT_ENCODER);
    encoder_clear_count(RIGHT_ENCODER);

    // ==================== 闃叉鍋跺彂璐熸暟褰卞搷閫熷害 PID ====================

    if(left_encoder_count < 0)
    {
        left_encoder_count = -left_encoder_count;
    }

    if(right_encoder_count < 0)
    {
        right_encoder_count = -right_encoder_count;
    }

    // ==================== 杞欢绱鎬昏矾绋� ====================

    left_encoder_total += left_encoder_count;
    right_encoder_total += right_encoder_count;

    car_distance_m = ((float)left_encoder_total + (float)right_encoder_total) /
                     (2.0f * ENCODER_COUNT_PER_METER);

    // ==================== 閫熷害 PID ====================
    // PID_Calc(pid, 瀹為檯鍊�, 鐩爣鍊�)

    left_base_pwm = PID_Calc(&left_speed_pid,
                             (float)left_encoder_count,
                             left_target_count);

    right_base_pwm = PID_Calc(&right_speed_pid,
                              (float)right_encoder_count,
                              right_target_count);
}


// ==================== 闄愬箙鍑芥暟 ====================

int16 limit_int16(int16 value, int16 min, int16 max)
{
    if(value > max)
    {
        return max;
    }
    else if(value < min)
    {
        return min;
    }
    else
    {
        return value;
    }
}

#pragma section all restore
