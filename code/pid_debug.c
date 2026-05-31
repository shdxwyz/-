#include "pid_debug.h"
#include "pid.h"

#ifdef PID_DEBUG_ENABLE

// ==================== 本地常量定义 ====================
// 与 cpu0_main.c 保持一致
#define ENCODER_COUNT_PER_METER         (12106.0f)
#define PID_PERIOD_S                    (0.02f)

// ==================== 本地变量 ====================

// 保存上一次的参数值，用于检测变化
static float last_param[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT] = {0};

// 上次打印详细PID数据的时间（ms）
static uint32 last_print_time_ms = 0;

// 上次发送示波器数据的时间（ms）
static uint32 last_osc_time_ms = 0;

// ==================== Draw_ResponseCurve 函数 ====================
// 山外多功能调试助手的上位机响应曲线绘制函数
// 严格按照山外调试助手协议格式
// data: 数据指针（float数组）
// size: 数据总字节数

void Draw_ResponseCurve(void *data, uint32 size)
{
    uint8 cmdf[2] = {3, (uint8)(~3)};
    uint8 cmdr[2] = {(uint8)(~3), 3};

    uart_write_buffer(DEBUG_UART_INDEX, cmdf, sizeof(cmdf));
    uart_write_buffer(DEBUG_UART_INDEX, (uint8 *)data, size);
    uart_write_buffer(DEBUG_UART_INDEX, cmdr, sizeof(cmdr));
}

// ==================== 初始化 ====================

void pid_debug_init(void)
{
    // 设置参数调节通道的初始值
    // 上位机连接后，这些值会显示在参数调节界面
    seekfree_assistant_parameter[PARAM_CH_LEFT_KP]    = PID_DEBUG_DEFAULT_LEFT_KP;
    seekfree_assistant_parameter[PARAM_CH_LEFT_KI]    = PID_DEBUG_DEFAULT_LEFT_KI;
    seekfree_assistant_parameter[PARAM_CH_LEFT_KD]    = PID_DEBUG_DEFAULT_LEFT_KD;
    seekfree_assistant_parameter[PARAM_CH_RIGHT_KP]   = PID_DEBUG_DEFAULT_RIGHT_KP;
    seekfree_assistant_parameter[PARAM_CH_RIGHT_KI]   = PID_DEBUG_DEFAULT_RIGHT_KI;
    seekfree_assistant_parameter[PARAM_CH_RIGHT_KD]   = PID_DEBUG_DEFAULT_RIGHT_KD;
    seekfree_assistant_parameter[PARAM_CH_BASE_SPEED] = PID_DEBUG_DEFAULT_BASE_SPEED;
    seekfree_assistant_parameter[PARAM_CH_TURN_KP]    = PID_DEBUG_DEFAULT_TURN_KP;

    // 初始化上一次参数值
    for(uint8 i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
    {
        last_param[i] = seekfree_assistant_parameter[i];
    }

    last_print_time_ms = system_getval_ms();

    printf("\r\n");
    printf("========================================\r\n");
    printf("  PID Debug v1.0\r\n");
    printf("========================================\r\n");
    printf("  Format:\r\n");
    printf("  target  : target speed (m/s)\r\n");
    printf("  get     : actual speed (m/s)\r\n");
    printf("  err     : error = target - get\r\n");
    printf("  Pout    : proportional term\r\n");
    printf("  Iout    : integral term\r\n");
    printf("  Dout    : derivative term\r\n");
    printf("  out     : PID total output\r\n");
    printf("  enc     : encoder count / 20ms\r\n");
    printf("  pwm     : final PWM output\r\n");
    printf("========================================\r\n");
    printf("  Current PID params:\r\n");
    printf("  Left  Kp=%.1f Ki=%.1f Kd=%.1f\r\n",
           PID_DEBUG_DEFAULT_LEFT_KP,
           PID_DEBUG_DEFAULT_LEFT_KI,
           PID_DEBUG_DEFAULT_LEFT_KD);
    printf("  Right Kp=%.1f Ki=%.1f Kd=%.1f\r\n",
           PID_DEBUG_DEFAULT_RIGHT_KP,
           PID_DEBUG_DEFAULT_RIGHT_KI,
           PID_DEBUG_DEFAULT_RIGHT_KD);
    printf("========================================\r\n");
    printf("  Speed test: 0.2->0.6 m/s every 10s\r\n");
    printf("========================================\r\n");
    printf("\r\n");
}

// ==================== 参数更新处理 ====================

static void pid_debug_process_parameters(void)
{
    // 检查每个参数通道是否有更新
    for(uint8 i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
    {
        if(seekfree_assistant_parameter_update_flag[i])
        {
            // 清除更新标志
            seekfree_assistant_parameter_update_flag[i] = 0;

            // 值发生变化时打印
            if(seekfree_assistant_parameter[i] != last_param[i])
            {
                last_param[i] = seekfree_assistant_parameter[i];

                switch(i)
                {
                    case PARAM_CH_LEFT_KP:
                        left_speed_pid.Kp = seekfree_assistant_parameter[i];
                        printf("[PID] 左轮 Kp = %.2f\r\n", left_speed_pid.Kp);
                        break;

                    case PARAM_CH_LEFT_KI:
                        left_speed_pid.Ki = seekfree_assistant_parameter[i];
                        printf("[PID] 左轮 Ki = %.2f\r\n", left_speed_pid.Ki);
                        break;

                    case PARAM_CH_LEFT_KD:
                        left_speed_pid.Kd = seekfree_assistant_parameter[i];
                        printf("[PID] 左轮 Kd = %.2f\r\n", left_speed_pid.Kd);
                        break;

                    case PARAM_CH_RIGHT_KP:
                        right_speed_pid.Kp = seekfree_assistant_parameter[i];
                        printf("[PID] 右轮 Kp = %.2f\r\n", right_speed_pid.Kp);
                        break;

                    case PARAM_CH_RIGHT_KI:
                        right_speed_pid.Ki = seekfree_assistant_parameter[i];
                        printf("[PID] 右轮 Ki = %.2f\r\n", right_speed_pid.Ki);
                        break;

                    case PARAM_CH_RIGHT_KD:
                        right_speed_pid.Kd = seekfree_assistant_parameter[i];
                        printf("[PID] 右轮 Kd = %.2f\r\n", right_speed_pid.Kd);
                        break;

                    case PARAM_CH_BASE_SPEED:
                        printf("[PID] 基础目标速度 = %.2f m/s\r\n",
                               seekfree_assistant_parameter[i]);
                        break;

                    case PARAM_CH_TURN_KP:
                        printf("[PID] 巡线转向强度 = %.4f\r\n",
                               seekfree_assistant_parameter[i]);
                        break;

                    default:
                        break;
                }
            }
        }
    }
}

// ==================== 主更新函数 ====================

void pid_debug_update(void)
{
    uint32 current_time_ms = system_getval_ms();

    // 每50ms发送一次示波器数据
    if((current_time_ms - last_osc_time_ms) >= PID_DEBUG_SEND_INTERVAL_MS)
    {
        last_osc_time_ms = current_time_ms;

        // 计算实际速度 (m/s)
        float left_speed  = (float)left_encoder_count / ENCODER_COUNT_PER_METER / PID_PERIOD_S;
        float right_speed = (float)right_encoder_count / ENCODER_COUNT_PER_METER / PID_PERIOD_S;
        float left_target_speed  = left_target_count / ENCODER_COUNT_PER_METER / PID_PERIOD_S;
        float right_target_speed = right_target_count / ENCODER_COUNT_PER_METER / PID_PERIOD_S;

        // 左轮误差
        float left_err  = left_target_speed - left_speed;
        float right_err = right_target_speed - right_speed;

        // 示波器数据：8个通道
        // ch0: 左轮实际速度  ch1: 左轮目标速度
        // ch2: 右轮实际速度  ch3: 右轮目标速度
        // ch4: 左轮PWM       ch5: 右轮PWM
        // ch6: 左轮误差      ch7: 右轮误差
        float osc_data[8] = {
            left_speed,           // ch0
            left_target_speed,    // ch1
            right_speed,          // ch2
            right_target_speed,   // ch3
            left_base_pwm,        // ch4
            right_base_pwm,       // ch5
            left_err,             // ch6
            right_err             // ch7
        };

        Draw_ResponseCurve(osc_data, sizeof(osc_data));
    }

    // 每2000ms打印一次串口摘要
    if((current_time_ms - last_print_time_ms) >= 2000)
    {
        last_print_time_ms = current_time_ms;

        // 计算实际速度 (m/s)
        float left_speed  = (float)left_encoder_count / ENCODER_COUNT_PER_METER / PID_PERIOD_S;
        float right_speed = (float)right_encoder_count / ENCODER_COUNT_PER_METER / PID_PERIOD_S;
        float left_target_speed  = left_target_count / ENCODER_COUNT_PER_METER / PID_PERIOD_S;
        float right_target_speed = right_target_count / ENCODER_COUNT_PER_METER / PID_PERIOD_S;

        printf("L: tgt=%.3f get=%.3f err=%+.3f P=%+.1f I=%+.1f out=%+.1f pwm=%d\r\n",
               left_target_speed, left_speed, left_target_speed - left_speed,
               left_speed_pid.Pout, left_speed_pid.Iout, left_base_pwm, (int16)left_base_pwm);
        printf("R: tgt=%.3f get=%.3f err=%+.3f P=%+.1f I=%+.1f out=%+.1f pwm=%d\r\n",
               right_target_speed, right_speed, right_target_speed - right_speed,
               right_speed_pid.Pout, right_speed_pid.Iout, right_base_pwm, (int16)right_base_pwm);
    }

    // 处理上位机下发的参数更新
    pid_debug_process_parameters();
}

// ==================== 参数查询接口 ====================

float pid_debug_get_param(uint8 channel)
{
    if(channel < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT)
    {
        return seekfree_assistant_parameter[channel];
    }
    return 0.0f;
}

uint8 pid_debug_is_param_updated(uint8 channel)
{
    if(channel < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT)
    {
        return seekfree_assistant_parameter_update_flag[channel];
    }
    return 0;
}

void pid_debug_clear_param_flag(uint8 channel)
{
    if(channel < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT)
    {
        seekfree_assistant_parameter_update_flag[channel] = 0;
    }
}

#else
// ==================== 调试关闭时的空函数 ====================

void pid_debug_init(void)
{
    // 空函数，不执行任何操作
}

void pid_debug_update(void)
{
    // 空函数，不执行任何操作
}

void Draw_ResponseCurve(void *data, uint32 size)
{
    (void)data;
    (void)size;
}

float pid_debug_get_param(uint8 channel)
{
    (void)channel;
    return 0.0f;
}

uint8 pid_debug_is_param_updated(uint8 channel)
{
    (void)channel;
    return 0;
}

void pid_debug_clear_param_flag(uint8 channel)
{
    (void)channel;
}

#endif // PID_DEBUG_ENABLE
