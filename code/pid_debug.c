#include "pid_debug.h"

#ifdef PID_DEBUG_ENABLE

// ==================== 本地变量 ====================

// 保存上一次的参数值，用于检测变化
static float last_param[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT] = {0};

// 上次发送示波器数据的时间（ms）
static uint32 last_send_time_ms = 0;

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

    last_send_time_ms = system_getval_ms();

    printf("[PID_DEBUG] PID 调试助手已初始化\r\n");
    printf("[PID_DEBUG] 左轮 Kp=%.1f Ki=%.1f Kd=%.1f\r\n",
           PID_DEBUG_DEFAULT_LEFT_KP,
           PID_DEBUG_DEFAULT_LEFT_KI,
           PID_DEBUG_DEFAULT_LEFT_KD);
    printf("[PID_DEBUG] 右轮 Kp=%.1f Ki=%.1f Kd=%.1f\r\n",
           PID_DEBUG_DEFAULT_RIGHT_KP,
           PID_DEBUG_DEFAULT_RIGHT_KI,
           PID_DEBUG_DEFAULT_RIGHT_KD);
    printf("[PID_DEBUG] 基础速度=%.2f m/s  转向强度=%.3f\r\n",
           PID_DEBUG_DEFAULT_BASE_SPEED,
           PID_DEBUG_DEFAULT_TURN_KP);
    printf("[PID_DEBUG] 打开山外多功能调试助手连接串口即可查看响应曲线\r\n");
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
                        printf("[PID_DEBUG] 左轮 Kp = %.2f\r\n", left_speed_pid.Kp);
                        break;

                    case PARAM_CH_LEFT_KI:
                        left_speed_pid.Ki = seekfree_assistant_parameter[i];
                        printf("[PID_DEBUG] 左轮 Ki = %.2f\r\n", left_speed_pid.Ki);
                        break;

                    case PARAM_CH_LEFT_KD:
                        left_speed_pid.Kd = seekfree_assistant_parameter[i];
                        printf("[PID_DEBUG] 左轮 Kd = %.2f\r\n", left_speed_pid.Kd);
                        break;

                    case PARAM_CH_RIGHT_KP:
                        right_speed_pid.Kp = seekfree_assistant_parameter[i];
                        printf("[PID_DEBUG] 右轮 Kp = %.2f\r\n", right_speed_pid.Kp);
                        break;

                    case PARAM_CH_RIGHT_KI:
                        right_speed_pid.Ki = seekfree_assistant_parameter[i];
                        printf("[PID_DEBUG] 右轮 Ki = %.2f\r\n", right_speed_pid.Ki);
                        break;

                    case PARAM_CH_RIGHT_KD:
                        right_speed_pid.Kd = seekfree_assistant_parameter[i];
                        printf("[PID_DEBUG] 右轮 Kd = %.2f\r\n", right_speed_pid.Kd);
                        break;

                    case PARAM_CH_BASE_SPEED:
                        printf("[PID_DEBUG] 基础目标速度 = %.2f m/s\r\n",
                               seekfree_assistant_parameter[i]);
                        break;

                    case PARAM_CH_TURN_KP:
                        printf("[PID_DEBUG] 巡线转向强度 = %.4f\r\n",
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

    // 按固定间隔发送示波器数据
    if((current_time_ms - last_send_time_ms) >= PID_DEBUG_SEND_INTERVAL_MS)
    {
        last_send_time_ms = current_time_ms;

        // 准备示波器数据
        // send_data[0] = motor.speed_get;   -> 左轮实际速度
        // send_data[1] = motor.speed_set;   -> 左轮目标速度
        // send_data[2] = 右轮实际速度
        // send_data[3] = 右轮目标速度
        float send_data[4];
        send_data[0] = (float)left_encoder_count;        // 左轮实际速度
        send_data[1] = left_target_count;                // 左轮目标速度
        send_data[2] = (float)right_encoder_count;       // 右轮实际速度
        send_data[3] = right_target_count;               // 右轮目标速度

        // 调用 Draw_ResponseCurve 发送数据
        Draw_ResponseCurve(send_data, sizeof(send_data));
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
