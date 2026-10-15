#include "Monitor.hpp"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"
#include "System.hpp"
#include "bsp_log.hpp"

Monitor& APP::monit = Monitor::GetInstance();

const static uint8_t LogPort = 0x01;    // 日志使用的端口号
const static uint8_t WatchPort = 0x02;  // 监视使用的端口号
const static uint8_t TrackPort = 0x03;  // 跟踪使用的端口号

const static uint8_t justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7f};  // VOFA+ JustFloat 协议的帧尾 (Little Endian: 00 00 80 7f)


template<typename T>
static void ConcatToBuf(byte* buf, size_t& used_bytes, void* value)
{
    T targ_var = *(T*)value;
    size_t char_use = snprintf((char*)buf + used_bytes, 63 - used_bytes, "%d", (int)targ_var);
    used_bytes += char_use;
}

static byte track_send_buf[64];
/**
 * @brief 发送监控信息
 * @note 先用着，写得草率点，后面再改
 */
void Monitor::TrackLog()
{
    if (track_count < 1)   return; // 没有跟踪变量，直接返回

    memset(track_send_buf, 0, 64);
    //track_send_buf[0] = '[';
    size_t used_bytes = 0;

    for(int i = 0; i < track_count; i++)
    {
        switch (track_type[i])
        {
            case track_uint8:
            {
                ConcatToBuf<uint8_t>(track_send_buf, used_bytes, track_list[i]);
                break;
            }
            case track_int8:
            {
                ConcatToBuf<int8_t>(track_send_buf, used_bytes, track_list[i]);
                break;
            }
            case track_uint16:
            {
                ConcatToBuf<uint16_t>(track_send_buf, used_bytes, track_list[i]);
                break;
            }
            case track_int16:
            {
                ConcatToBuf<int16_t>(track_send_buf, used_bytes, track_list[i]);
                break;
            }
            case track_uint32:
            {
                ConcatToBuf<uint32_t>(track_send_buf, used_bytes, track_list[i]);
                break;
            }
            case track_int32:
            {
                ConcatToBuf<int32_t>(track_send_buf, used_bytes, track_list[i]);
                break;
            }
            case track_float:
            {
                float var = *(float*)track_list[i];
                uint8_t char_use = snprintf((char*)track_send_buf + used_bytes, 63 - used_bytes, "%.2f", var);
                used_bytes += char_use;
                break;
            }
        }
        // 添加逗号","
        if(i != track_count - 1)
        {
            track_send_buf[used_bytes] = ',';
            track_send_buf[used_bytes + 1] = '\0';
            used_bytes += 1;
        }
        else // 结束时添加换行符"\n"
        {
            size_t len = strlen((char*)track_send_buf);
            track_send_buf[len] = '\n';
            track_send_buf[len + 1] = '\0';
        }
    }

    // 发送编码后的数据
    if (track_uart.IsValid())
    {
        track_uart.Transmit(track_send_buf, strlen((char*)track_send_buf));
    }
}




void Monitor::TrackLogJustFloat()
{
    if (track_count < 1) return;

    // 清空缓冲区 (复用 vofa_buf，因为它在头文件里定义了但没被用)
    // 你的 Monitor.hpp 里定义了 byte vofa_buf[64]，这里正好用上
    // 8个变量 * 4字节 + 4字节帧尾 = 36字节，64字节足够了
    memset(vofa_buf, 0, 64);
    // vofa_buf[0] = '[';
    size_t used_bytes = 0;

    for(int i = 0; i < track_count; i++)
    {
        float temp_val = 0.0f;

        // 统一类型转换为 float
        // JustFloat 协议要求通道数据必须是 32bit float
        switch (track_type[i])
        {
            case track_uint8:  temp_val = (float)(*(uint8_t*)track_list[i]);  break;
            case track_int8:   temp_val = (float)(*(int8_t*)track_list[i]);   break;
            case track_uint16: temp_val = (float)(*(uint16_t*)track_list[i]); break;
            case track_int16:  temp_val = (float)(*(int16_t*)track_list[i]);  break;
            case track_uint32: temp_val = (float)(*(uint32_t*)track_list[i]); break;
            case track_int32:  temp_val = (float)(*(int32_t*)track_list[i]);  break;
            case track_float:  temp_val = *(float*)track_list[i];             break;
            default: break;
        }

        // 内存拷贝 (4字节)
        // STM32 是小端序(Little Endian)，VOFA+ 也是小端序，直接拷贝即可
        memcpy(&vofa_buf[used_bytes], &temp_val, 4);
        used_bytes += 4;
    }

    // 追加帧尾 (00 00 80 7f)
    memcpy(&vofa_buf[used_bytes], justfloat_tail, 4);
    used_bytes += 4;

    // 发送原始二进制数据
    if (track_uart.IsValid())
    {
        track_uart.Transmit(vofa_buf, used_bytes);
    }
}



static char err_log_buf[72] = {0};
/**
 * @brief 发送错误日志
 * @note 默认不向遥控器发送错误日志，只向上位机发送
 * @warning 总发送长度不超过72字节
 */
void Monitor::LogError(const char* format, ...)
{
    // 解析可变参数列表
    va_list args;
    va_start(args, format);
    
    // 清空缓冲区
    memset(err_log_buf, 0, sizeof(err_log_buf));
    
    // 填充错误头和时间戳
    uint8_t used_bytes = snprintf(err_log_buf, 12, "[%.2f]", System.runtime_tick);
    vsnprintf(err_log_buf + used_bytes, 72 - used_bytes, format, args);
    va_end(args);

    // 发送日志到上位机，BspLog会自动加换行
    BspLog_LogError("%s", err_log_buf);
}

static char wrn_log_buf[72] = {0};
/**
 * @brief 发送警告日志
 * @note 同上
 */
void Monitor::LogWarning(const char* format, ...)
{
    // 解析可变参数列表
    va_list args;
    va_start(args, format);
    
    // 清空缓冲区
    memset(wrn_log_buf, 0, sizeof(wrn_log_buf));
    
    // 填充警告头和时间戳
    uint8_t used_bytes = snprintf(wrn_log_buf, 12, "[%.2f]", System.runtime_tick);
    vsnprintf(wrn_log_buf + used_bytes, 72 - used_bytes, format, args);
    va_end(args);

    // 发送日志到上位机
    BspLog_LogWarning("%s", wrn_log_buf);
}

static char nrm_log_buf[72] = {0};
/**
 * @brief 发送日志
 * @note 同上
 */
void Monitor::LogInfo(const char* format, ...)
{
    // 解析可变参数列表
    va_list args;
    va_start(args, format);
    
    // 清空缓冲区
    memset(nrm_log_buf, 0, sizeof(nrm_log_buf));
    
    // 填充日志头和时间戳
    uint8_t used_bytes = snprintf(nrm_log_buf, 12, "[%.2f]", System.runtime_tick);
    vsnprintf(nrm_log_buf + used_bytes, 72 - used_bytes, format, args);
    va_end(args);

    // 发送日志到上位机
    BspLog_LogInfo("%s", nrm_log_buf);
}

static char ok_log_buf[72] = {0};
/**
 * @brief 发送完成日志
 * @note 同上
 */
void Monitor::LogOK(const char* format, ...)
{
    // 解析可变参数列表
    va_list args;
    va_start(args, format);

    // 清空缓冲区
    memset(ok_log_buf, 0, sizeof(ok_log_buf));

    // 填充日志头和时间戳
    uint8_t used_bytes = snprintf(ok_log_buf, 12, "[%.2f]", System.runtime_tick);
    vsnprintf(ok_log_buf + used_bytes, 72 - used_bytes, format, args);
    va_end(args);

    // 发送日志到上位机
    BspLog_LogOK("%s", ok_log_buf);
}

static char spec_log_buf[72] = {0};
/**
 * @brief 发送说明日志
 * @note 同上
 */
void Monitor::LogSpec(const char* format, ...)
{
    // 解析可变参数列表
    va_list args;
    va_start(args, format);

    // 清空缓冲区
    memset(spec_log_buf, 0, sizeof(spec_log_buf));

    // 填充日志头和时间戳
    uint8_t used_bytes = snprintf(spec_log_buf, 12, "[%.2f]", System.runtime_tick);
    vsnprintf(spec_log_buf + used_bytes, 72 - used_bytes, format, args);
    va_end(args);

    // 发送日志到上位机
    BspLog_LogSpec("%s", spec_log_buf);
}

static char rsp_log_buf[72] = {0};
/**
 * @brief 发送响应日志
 * @note 同上
 */
void Monitor::LogRespond(const char* format, ...)
{
    // 解析可变参数列表
    va_list args;
    va_start(args, format);

    // 清空缓冲区
    memset(rsp_log_buf, 0, sizeof(rsp_log_buf));

    // 填充日志头和时间戳
    uint8_t used_bytes = snprintf(rsp_log_buf, 12, "[%.2f]", System.runtime_tick);
    vsnprintf(rsp_log_buf + used_bytes, 72 - used_bytes, format, args);
    va_end(args);

    // 发送日志到上位机
    BspLog_LogRespond("%s", rsp_log_buf);
}


void Monitor::Perflize()
{
    high_performance_mode = true;
}


/**
 * @brief 初始化监视器
 * 
 * @param huart_track 用于发送跟踪数据的串口，传入 nullptr 则不启用跟踪功能；
 * 请注意，huart_track 只负责track的数据，日志数据取决于bsp_log的配置！！
 * @param huart_farc 暂未启用
 * @param vofa_mode 
 */
void Monitor::Init(BSP::UART::UartID huart_track, BSP::UART::UartID huart_farc, bool vofa_mode)
{
    if (huart_track != nullptr)
    {
        track_uart = BSP::UART::Apply(huart_track);
    }

    if (huart_farc != nullptr)
    {
        farcon_uart = BSP::UART::Apply(huart_farc);
    }
}