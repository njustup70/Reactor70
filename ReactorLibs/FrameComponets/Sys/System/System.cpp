#include "System.hpp"
#include "bsp_dwt.hpp"

// #include "Chassis.hpp"   // 暂时不作为系统库内容
#include "led_ws2812.hpp"
#include "Monitor.hpp"
#include "farcon.hpp"
#include "bsp_log.hpp"
#include "freertos.h"
#include "task.h"
#include "RtosCpp.hpp"

SystemType& System = SystemType::GetInstance();
LedWs2812 sys_ledband;

void SystemType::Init(bool Sc)
{
  // 输出系统启动信息
  BspLog_LogInfo("\n\n");
  BspLog_LogSpec("/----^---^-- Welcome to REACTOR SYSTEM --^---^----/");
  BspLog_LogInfo("Waiting for system initialization...\n\n");

    DWT_Init(CPU_HERT_A_BOARD_MHZ);

    // 初始化Monitor监视器
    Monitor::GetInstance().Init(Hardware::huart_log, nullptr, false);

    // 初始化系统灯带
    // sys_ledband.Init(Hardware::htim_led, TIM_CHANNEL_1, 13);
    // 颜色偏置因子（用于校正颜色）
    // sys_ledband.BiasFactor = Vec3(0.843f, 1.0f, 0.843f); 

  // 自动开始自检
  if (Sc)
    status = Systems::SELF_CHECK;
}

/**
 * @brief 运行机器人系统主进程
 * @note 该方法应被周期性调用，以处理系统任务
 */
void SystemType::Run()
{
  static float delta_time = 0;
  static uint32_t cnt = 0;

  // 自检
  _Update_SelfCheck();

    /*--<       正式运行        >--*/

    // 管理 主灯带 状态（50Hz分频）
    _Update_LedBand();

    // 提供位置
    if (pos_source != nullptr)
    {
        position = *pos_source;
    }

  // 更新全局时间
  runtime_tick = DWT_GetTimeline_Sec();
  delta_time = DWT_GetDeltaTime(&cnt);

  // 判定计时器是否停止连续六次（30ms）
  static uint32_t stop_cnt = 0;
  if (delta_time < 0.002f)
  {
    stop_cnt++;
    if (stop_cnt == 6)
    {
      out_from_debugmode = true;
      monit.LogSpec("--- --- Exit of Debug Detected! --- ---\n");
    }
  }
  else
  {
    out_from_debugmode = false;
    stop_cnt = 0;
  }

  // RTT接收命令处理
  // BspLog_RecvCMD();

    // 零开销巡检所有 App 状态
    for (int i = 0; i < 24; i++) {
        if (app_list[i] != nullptr) {
            App::Status s = app_list[i]->status;
            if (s == App::Error) {
                // TODO: 用一种不会刷屏的方法打印 Error，或者放到别处
                // monit.LogError("App [%s] Fatal Error!", app_list[i]->name);
                if (status == Systems::WORKING || status == Systems::READY) {
                    status = Systems::STOP; 
                    // Display.ErrorBlink(24, 200);
                }
            } else if (s == App::Warning) {
                // WarningBlink 等提示
                Display.WarningBlink(3, 300);
            }
        }
    }

    static int temp_cnt = 0;
    // 跟踪变量（非高性能模式下）
    if (!Monitor::GetInstance().high_performance_mode && temp_cnt++ >= 1)
    {
        Monitor::GetInstance().TrackLog();
        temp_cnt = 0;
    }
}

void SystemType::PerformanceRun()
{
    // 高性能模式下，1000Hz跟踪变量
    if (Monitor::GetInstance().high_performance_mode)
    {
        Monitor::GetInstance().TrackLogJustFloat();
    }
}



void SystemType::_Update_LedBand()
{
    static uint32_t prescaler_cnt = 0;
    prescaler_cnt++;

    if (prescaler_cnt >= ledband_prescaler)
    {
        prescaler_cnt = 0;

        if (Display.display_overlay)
        {
            // 覆盖显示接口
            _LedBandDisplayControl();
        }
        else
        {
            // 根据不同的要求来控制系统灯
            _LedBandControl();
        }

        sys_ledband.Upload();
    }
}



/**
 * @brief 自检（检查所有注册的 Application 的 WatchPoint 是否正常）
 */
void SystemType::_Update_SelfCheck()
{
    static uint16_t check_cnt = 0;
    static bool error_list[24] = {false};

    if (start_selfcheck_flag && status == Systems::ORIGIN)
    {
        start_selfcheck_flag = false;
        status = Systems::SELF_CHECK;
    }

    // 处于自检状态时，进行自检
    if (status != Systems::SELF_CHECK)   return;

    /**     确保所有 App 的 WatchPoint 在一秒内都持续为 true   **/
    for (uint8_t i = 0; i < 24; i++) 
    {
        if (app_list[i] != nullptr)
        {
            if(!app_list[i]->WatchPoint() && check_cnt > 100)    // 遇到有不在线的
            {
                error_list[i] = true;
            }
        }
    }

    check_cnt++;
    if (check_cnt >= 300)   // 先沉默半秒，再持续一秒，总共1.5秒
    {
        // 检查是否有错误
        bool have_error = false;
        for (uint8_t i = 0; i < 24; i++)
        {
            if (error_list[i])
            {
                // 严重错误，直接报错
                monit.LogError("App [%s] Self-Check Failed!\n", app_list[i]->name);
                have_error = true;
            }
        }

        if (have_error)
        {
            // 返回初始状态，看有没有机会修好
            status = Systems::ORIGIN;
            check_cnt = 0;

            // 重置错误列表
            memset(error_list, 0, sizeof(error_list));
            return;
        }
        else
        {
            // 自检完成，进入READY状态
            memset(error_list, 0, sizeof(error_list));
            status = Systems::READY;
            monit.LogOK("System Self-Check Passed!\n");
            check_cnt = 0;
        }
    }
}



/**
 * @brief 默认情况下，系统的灯带控制逻辑
 * @note 可被Display接口覆盖
 */
void SystemType::_LedBandControl()
{
    switch (status)
    {
        case Systems::ORIGIN:
        {
            // 正常：呼吸白灯
            if (!is_retrying)
                sys_ledband.Breath(Color(255.0f, 255.0f, 255.0f), 1.0f);
            // 重试：呼吸橙灯
            else
                sys_ledband.Breath(Color(255.0f, 127.0f, 0.0f), 1.0f);
            break;
        }
        case Systems::SELF_CHECK:
        {
            // 正常自检中：白色滚动
            if (!is_retrying)
                sys_ledband.Running(Color(255.0f, 255.0f, 255.0f), 0.2, 0.3);
            // 重试自检中：橙色滚动
            else
                sys_ledband.GradientFlow(Color(255.0f, 127.0f, 0.0f), Color(0, 0, 0), 0.3);
            break;
        }
        case Systems::READY:
        {
            // 准备好了：呼吸阵营灯
            if (camp == Systems::Camp_Blue)
                sys_ledband.Breath(Color(0.0f, 0.0f, 255.0f), 1.0f);
            else
                sys_ledband.Breath(Color(255.0f, 0.0f, 0.0f), 1.0f);
            
            break;
        }
        case Systems::WORKING:
        {
            // 分红蓝区
            // 蓝区：常亮蓝灯
            if (camp == Systems::Camp_Blue)     sys_ledband.Lit(Color(0.0f, 0.0f, 255.0f));
            // 红区：常亮红灯
            else    sys_ledband.Lit(Color(255.0f, 0.0f, 0.0f));
            break;
        }
    }
}












/*****--<       灯带显示接口        >--*****/
/**
 * @brief 系统灯带显示控制接口（可覆盖常态显示）
 */
void SystemType::_LedBandDisplayControl()
{
    switch (Display.display_type)
    {
        case SystemType::_LedDisplayAPI::SysLEDDisp_WarningBlink:
        {
            // 闪烁逻辑（在周期的前一半亮，后一半灭）
            if ((Display.blink_cnt * 5) % (Display.blink_interval) < (Display.blink_interval / 2))
            {
                sys_ledband.Lit(Color(255.0f, 255.0f, 0.0f));   // 黄色
            }
            else
            {
                sys_ledband.Lit(Color(0.0f, 0.0f, 0.0f));       // 熄灭
            }
            Display.blink_cnt++;
            break;
        }
        case SystemType::_LedDisplayAPI::SysLEDDisp_ErrorBlink:
        {
            // 闪烁逻辑（在周期的前一半亮，后一半灭）
            if ((Display.blink_cnt * 5) % (Display.blink_interval) < (Display.blink_interval / 2))
            {
                sys_ledband.Lit(Color(255.0f, 255.0f, 0.0f));   // 紫色
            }
            else
            {
                sys_ledband.Lit(Color(0.0f, 0.0f, 0.0f));     // 熄灭
            }
            Display.blink_cnt++;
            break;
        }
    }

    // 检查是否完成，如果闪烁时间大于要求次数乘以间隔时间，则结束
    if (Display.blink_cnt * 5 >= Display.blink_times * Display.blink_interval)
    {
        // 结束覆盖显示
        Display.display_overlay = false;
        Display.display_type = SystemType::_LedDisplayAPI::SysLEDDisp_None;
        Display.blink_cnt = 0;
    }
}

/**
 * @brief 警告闪烁LED
 */
void SystemType::_LedDisplayAPI::WarningBlink(uint8_t times, uint16_t interval)
{
    // 正在执行的话，就直接退出
    if (display_overlay) return;
    // 启用覆盖显示
    display_overlay = true;

    // 执行闪烁
    display_type = SysLEDDisp_WarningBlink;
    
    // 设置参数
    blink_times = times;
    blink_interval = interval;
    blink_cnt = 0;
}


/**
 * @brief 
 */
void SystemType::SetPositionSource(Vec3& pos)
{
    pos_source = &pos;
}

template <typename T>
T *SystemType::FindApp(const char *name)
{
    for (uint8_t i = 0; i < 24; i++)
    {
        if (app_list[i] != nullptr)
        {
            // 首先对比类型
            if (typeid(app_list[i]->GetType()) == typeid(T))
            {
                // 再对比名字
                if (strncmp(app_list[i]->name, name, 24) == 0)
                {
                    return dynamic_cast<T *>(app_list[i]);
                }
            }
        }
    }
    return nullptr;
}

bool SystemType::RegistApp(Application &app_inst)
{
    for (uint8_t i = 0; i < 24; i++)
    {
        if (app_list[i] == nullptr)
        {
            // 添加到应用实例列表中
            app_list[i] = &app_inst;

            // 运行其Start函数
            app_inst.Start();
            return true;
        }
    }
    return false;
}

bool SystemType::RegistSpiSamp(SpiSamp &sampler)
{
    if (sampler.poll_full_frame == nullptr)
    {
        BspLog_LogWarning("SpiSamp [%s] register failed: callback is null.\n",
                          (sampler.name != nullptr) ? sampler.name : "Unknown");
        return false;
    }

    // 查找空槽并注册
    for (uint8_t i = 0; i < 24; i++)
    {
        if (spi_sampler_list[i] == nullptr)
        {
            spi_sampler_list[i] = &sampler;
            return true;
        }
    }

    BspLog_LogWarning("SpiSamp [%s] register failed: list full.\n",
                      (sampler.name != nullptr) ? sampler.name : "Unknown");
    return false;
}

/**
 * @brief 更新所有 SPI 采样器（轮询模式）
 */
void SystemType::_Update_SpiSamps()
{
    for (uint8_t i = 0; i < 24; i++)
    {
        // 获取对应的采样器实例
        SpiSamp *sampler = spi_sampler_list[i];

        // 确保实例存在
        if (sampler == nullptr) continue;

        if (sampler->poll_full_frame == nullptr) continue;

        // 与 App 类似，按循环节拍直接轮询。
        // 一旦这一轮确实产出了完整新帧，并且实例声明了消费回调，
        // 就立刻唤醒消费线程，避免把算法耗时塞进 SpiRead 线程。
        bool has_new_frame = sampler->poll_full_frame(sampler->owner);
        if (has_new_frame && sampler->consume_full_frame != nullptr)
        {
            Reactor46H_NotifySpiConsume();
        }
    }
}

void SystemType::_Update_SpiConsumes()
{
    for (uint8_t i = 0; i < 24; i++)
    {
        SpiSamp *sampler = spi_sampler_list[i];

        if (sampler == nullptr) continue;
        if (sampler->consume_full_frame == nullptr) continue;

        // 一次唤醒就把实例内部已经准备好的样本全部消费掉。
        // 这样 System 只管“唤醒”，不需要在系统层维护额外的样本队列状态。
        while (sampler->consume_full_frame(sampler->owner))
        {
        }
    }
}

/**
 * @brief 运行所有应用实例并更新健康状态缓存
 */
void SystemType::_Update_Applications()
{
    for (uint8_t i = 0; i < 24; i++)
    {
        if (app_list[i] != nullptr)
        {
            if (app_list[i]->CntFull())
            {
                // 自动更新应用状态缓存，供零开销跨库查询
                app_list[i]->status = app_list[i]->GetStatus();
                app_list[i]->Update();
            }
        }
    }
}


/**
 * @brief 监测预分频计数器是否满了
 */
bool Application::CntFull()
{
    _prescaler_cnt++;
    if (_prescaler_cnt >= prescaler)
    {
        _prescaler_cnt = 0;
        return true;
    }
    return false;
}
