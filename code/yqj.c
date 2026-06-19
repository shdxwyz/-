#include "yqj.h"
#include "device.h"

#define YQJ_MS_TO_10NS(ms) ((uint32)((ms) * 100000UL))

uint16 yqj_flag = 1;
yqj_state_enum yqj_state = YQJ_STATE_LINE;
uint8 yqj_action_trigger = 0;
uint32 yqj_state_start_time = 0;
int32 yqj_lock_start_count = 0;

static float yqj_pid_period_s = 0.02f;
static float yqj_encoder_count_per_meter = 12106.0f;

// 鍒ゆ柇宸﹁浆瑙﹀彂鏉′欢锛氬乏杈逛紶鎰熷櫒妫�娴嬪埌鐧界嚎锛堜綆浜庨槇鍊硷級锛�
// 鍚屾椂鍙宠竟浼犳劅鍣ㄦ病鏈夋娴嬪埌鐧界嚎锛堥珮浜庨槇鍊硷級锛岄伩鍏嶇洿閬撹瑙﹀彂銆�
uint8 yqj_left_turn_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] >= YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] >= YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 鍒ゆ柇鍙宠浆瑙﹀彂鏉′欢锛氬彸杈逛紶鎰熷櫒妫�娴嬪埌鐧界嚎锛堜綆浜庨槇鍊硷級锛�
// 鍚屾椂宸﹁竟浼犳劅鍣ㄦ病鏈夋娴嬪埌鐧界嚎锛堥珮浜庨槇鍊硷級锛岄伩鍏嶇洿閬撹瑙﹀彂銆�
uint8 yqj_right_turn_trigger(const uint16 adc_value[])
{
    return (adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[0] >= YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[1] >= YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 鐢甸樆
uint8 yqj_dianzu_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 鐢垫簮
uint8 yqj_dianyuan_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 寮�鍏�1_0
uint8 yqj_kaiguang1_0trigger(const uint16 adc_value[])
{
    return (adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 寮�鍏�0_1
uint8 yqj_kaiguang0_1trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 浜屾瀬绠�
uint8 yqj_erjiguan_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 涓夋瀬绠�1_2
uint8 yqj_sanjiguan1_2trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 涓夋瀬绠�2_1
uint8 yqj_sanjiguan2_1trigger(const uint16 adc_value[])
{
    return (adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 涓夋瀬绠�2_0
uint8 yqj_sanjiguan2_0trigger(const uint16 adc_value[])
{
    return (adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}
// 涓夋瀬绠�0_1
uint8 yqj_sanjiguan0_1trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}
// 涓夋瀬绠�1_0
uint8 yqj_sanjiguan1_0trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE);
}
// 涓夋瀬绠�0_2
uint8 yqj_sanjiguan0_2trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}
// 鍙岃Е鍙�
uint8 yqj_double_trigger(const uint16 adc_value[])
{
    return (adc_value[0] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 鐢垫劅
uint8 yqj_ldiangan_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 鐢垫劅
uint8 yqj_rdiangan_trigger(const uint16 adc_value[])
{
    return (adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 绾垮湀
uint8 yqj_xianquan_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// erji
uint8 yqj_erji_trigger(const uint16 adc_value[])
{
    return (adc_value[9] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 鐢垫睜
uint8 yqj_dianchi_trigger(const uint16 adc_value[])
{
    return (
        adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
        adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
        adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
        adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 闈為棬
uint8 yqj_feimen_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}

// 电容
uint8 yqj_dianrong_trigger(const uint16 adc_value[])
{
    return (adc_value[1] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[2] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[3] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[4] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[5] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[6] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[7] < YQJ_TURN_TRIGGER_ADC_VALUE &&
            adc_value[8] < YQJ_TURN_TRIGGER_ADC_VALUE);
}
// ==================== 鍐呴儴宸ュ叿鍑芥暟 ====================

// 灏嗛�熷害 m/s 鎹㈢畻鎴愪竴涓� PID 鍛ㄦ湡鍐呯殑缂栫爜鍣ㄧ洰鏍囪鏁般��
static float yqj_speed_to_target_count(float speed_mps)
{
    return speed_mps * yqj_pid_period_s * yqj_encoder_count_per_meter;
}

// 灏嗚嚜閿佽窛绂� m 鎹㈢畻鎴愬乏鍙宠疆閲岀▼鍜岄渶瑕佸鍔犵殑缂栫爜鍣ㄨ鏁般��
static int32 yqj_meter_to_count(float distance_m)
{
    if (distance_m <= 0.0f)
    {
        return 0;
    }

    return (int32)(distance_m * yqj_encoder_count_per_meter);
}

// 鍒囨崲鐘舵�侊紝骞惰褰曡繘鍏ヨ鐘舵�佺殑鏃堕棿銆�
static void yqj_enter_state(yqj_state_enum state)
{
    yqj_state = state;
    yqj_state_start_time = system_getval();
}

// 鍒ゆ柇鍔ㄤ綔鍚庣殑鑷攣璺濈鏄惁宸茬粡璧板銆�
static uint8 yqj_lock_distance_reached(int32 encoder_total_sum, float lock_distance_m)
{
    int32 need_count = yqj_meter_to_count(lock_distance_m);

    if (need_count <= 0)
    {
        return 1;
    }

    return ((encoder_total_sum - yqj_lock_start_count) >= need_count);
}

// ==================== 瀵瑰宸ュ叿鍑芥暟 ====================

// 鍒濆鍖栧厓鍣ㄤ欢椤哄簭妗嗘灦锛屾妸 flag銆佺姸鎬併�佸姩浣滈兘澶嶄綅鍒拌捣鐐广��
void yqj_init(float pid_period_s, float encoder_count_per_meter)
{
    yqj_pid_period_s = pid_period_s;
    yqj_encoder_count_per_meter = encoder_count_per_meter;
    yqj_flag = 1;
    yqj_state = YQJ_STATE_LINE;
    yqj_action_trigger = 0;
    yqj_state_start_time = 0;
    yqj_lock_start_count = 0;

    system_start();
}

// 鍒ゆ柇浠� start_time 寮�濮嬫槸鍚﹀凡缁忚揪鍒版寚瀹氭绉掓暟锛宒uration_ms 涓� 0 鏃惰〃绀轰笉鐢ㄧ瓑鏃堕棿銆�
uint8 yqj_time_reached(uint32 start_time, uint32 duration_ms)
{
    if (0 == duration_ms)
    {
        return 1;
    }

    return ((uint32)(system_getval() - start_time) >= YQJ_MS_TO_10NS(duration_ms));
}

// 褰撳墠 case 鐨勬潯浠舵垚绔嬪悗锛岃褰曟槸鍚︽墽琛屽姩浣滐紝骞惰繘鍏ヨЕ鍙戝欢鏃躲��
void yqj_start_case(uint8 action_trigger)
{
    yqj_action_trigger = action_trigger;
    yqj_enter_state(YQJ_STATE_DELAY);
}

// 鍔ㄤ綔鎵ц瀹屾垚鍚庯紝璁板綍宸﹀彸杞噷绋嬪拰锛屽苟杩涘叆鑷攣鐘舵�併��
void yqj_start_lock(int32 encoder_total_sum)
{
    yqj_lock_start_count = encoder_total_sum;
    yqj_enter_state(YQJ_STATE_LOCK);
}

// 褰撳墠 case 瀹屽叏缁撴潫鍚庯紝flag 鍔� 1锛屽紑濮嬬瓑寰呬笅涓�涓厓鍣ㄤ欢鏉′欢銆�
void yqj_finish_case(void)
{
    if (yqj_flag < 65535u)
    {
        yqj_flag++;
    }
    yqj_action_trigger = 0;
    yqj_enter_state(YQJ_STATE_LINE);

    // motor_stop();
    // system_delay_ms(20000);
}

// 鍒ゆ柇褰撳墠 case 鐨勮嚜閿佹槸鍚︾粨鏉燂紱鏃堕棿鍜岃窛绂讳袱涓潯浠堕兘婊¤冻鎵嶈В閿併��
uint8 yqj_lock_done(int32 encoder_total_sum, uint32 lock_ms, float lock_distance_m)
{
    return (yqj_time_reached(yqj_state_start_time, lock_ms) &&
            yqj_lock_distance_reached(encoder_total_sum, lock_distance_m));
}

// 鏍规嵁褰撳墠 case 缁欏嚭鐨勫乏鍙宠疆閫熷害瑕嗙洊鐩爣锛涜Е鍙戞爣蹇椾负 0 鏃朵笉瑕嗙洊锛岀户缁贰绾裤��
// 杞集鏃跺湪杞集閫熷害鍩虹涓婂彔鍔犲贰绾夸慨姝ｉ噺锛岃灏忚溅杈硅浆杈瑰贰绾裤��
void yqj_apply_action(float left_speed_mps,
                      float right_speed_mps,
                      float *left_target_count,
                      float *right_target_count)
{
    if (yqj_action_trigger)
    {
        float base_left = yqj_speed_to_target_count(left_speed_mps);
        float base_right = yqj_speed_to_target_count(right_speed_mps);

        // 璁＄畻宸＄嚎淇閲忥細褰撳墠宸＄嚎鐩爣涓庡熀纭�閫熷害鐨勫樊鍊�
        float line_correction_left = *left_target_count - base_left;
        float line_correction_right = *right_target_count - base_right;

        // 鍙犲姞淇閲忥紝闄愬箙鍒� 卤50% 闃叉淇杩囧ぇ
        if (line_correction_left > 0)
        {
            line_correction_left = line_correction_left > base_left * 0.5f ? base_left * 0.5f : line_correction_left;
        }
        else
        {
            line_correction_left = line_correction_left < -base_left * 0.5f ? -base_left * 0.5f : line_correction_left;
        }

        if (line_correction_right > 0)
        {
            line_correction_right = line_correction_right > base_right * 0.5f ? base_right * 0.5f : line_correction_right;
        }
        else
        {
            line_correction_right = line_correction_right < -base_right * 0.5f ? -base_right * 0.5f : line_correction_right;
        }

        *left_target_count = base_left + line_correction_left;
        *right_target_count = base_right + line_correction_right;
    }
}

// 鑾峰彇褰撳墠姝ｅ湪绛夊緟鎴栨墽琛岀殑鍏冨櫒浠剁紪鍙凤紝涓插彛璋冭瘯鏃剁湅杩欎釜鍊笺��
uint16 yqj_get_flag(void)
{
    return yqj_flag;
}

// 鎵嬪姩璁剧疆褰撳墠鍏冨櫒浠剁紪鍙凤紝鏂逛究浣犱粠鏌愪竴涓� case 寮�濮嬭皟杞︺��
void yqj_set_flag(uint16 flag)
{
    yqj_flag = flag;
    yqj_action_trigger = 0;
    yqj_enter_state(YQJ_STATE_LINE);
}

// 鑾峰彇褰撳墠鐘舵�侊細宸＄嚎銆佸欢鏃躲�佹墽琛屽姩浣溿�佽嚜閿併��
yqj_state_enum yqj_get_state(void)
{
    return yqj_state;
}

// 鑾峰彇褰撳墠鏄惁姝ｅ湪鎵ц瑙﹀彂鍔ㄤ綔锛�0 琛ㄧず涓嶆墽琛岋紝1 琛ㄧず鎵ц銆�
uint8 yqj_get_action_trigger(void)
{
    return yqj_action_trigger;
}
