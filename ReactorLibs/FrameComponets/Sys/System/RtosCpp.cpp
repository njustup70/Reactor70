#include "RtosCpp.hpp"
#include "std_cpp.h"

#include "freertos.h"
#include "task.h"
#include "cmsis_os.h"
#include "semphr.h"

#include "bsp_dwt.hpp"
#include "bsp_log.hpp"
#include "motor_dji.hpp"
#include "System.hpp"
#include "RtosCpp.hpp"
// #include "Chassis.hpp"   // 暂时不作为系统库内容
#include "Monitor.hpp"

/* ================= 任务句柄定义 ================= */
// 如果需要在其他地方引用（如挂起任务），可以在头文件 extern
TaskHandle_t ApplicationTaskHandle;
TaskHandle_t RobotSystemTaskHandle;
TaskHandle_t SpiReadTaskHandle;
TaskHandle_t SpiConsumeTaskHandle;
TaskHandle_t ControlTaskHandle;
TaskHandle_t StateCoreTaskHandle;
TaskHandle_t CoroutineHandles[4]; // 协程数组，管理更方便
static SemaphoreHandle_t SpiConsumeSemaphore = nullptr;
void Reactor46H_TakeOverRTOS();
void ApplicationCpp();
void RobotSystemCpp();
void StateCoreCpp();
void ControlCpp();
void SpiReadCpp();
void SpiConsumeCpp();


/* ================= 内部适配器 ================= */
/**
 * @brief  通用任务适配器
 * @note   将 FreeRTOS 的 void(*)(void*) 转换为无参的 void(*)(void)
 * 这样你就不用修改 ApplicationCpp 等函数的签名了
 */
static void TaskWrapper(void *function_ptr)
{
    // 将参数强转为函数指针并调用
    auto real_function = (void (*)())function_ptr;
    real_function();
    
    // 正常情况下不会执行到这里，如果函数退出了，删除任务以防报错
    vTaskDelete(NULL);
}

/**
 * @brief 协程占位函数（对应原来的 Coroutine_x）
 */
static void CoroutineStub(void *arg)
{
    while(1) { osDelay(1000); } // 默认空闲，防止跑飞
}

/**
 * @brief 预初始化函数
 * @warning 为什么要搞一个这个，而不是在RTOS启动的线程初始化呢
 * 主要是因为怕线程爆栈，主函数的栈复用一下
 */
void Reactor46H_Initialize()
{
    Hardware::Config_Hardwares();
    // 初始化日志系统
    BspLog_Init();
    BspLog_LogInfo("\n\n");

    // 必须要，因为很多外部设备都要时间初始化
    // 尤其是C620 / C610，第一次上电时如果不等它们自检完就发指令，可能会冲爆
    HAL_Delay(4000);

    System.Init();
    MainFrameCpp();

    HAL_Delay(500);

    // 接管RTOS控制权
    Reactor46H_TakeOverRTOS();
    osKernelStart();
}

/**
 * @brief 接管RTOS控制权
 * @note 本函数同样在main函数中被调用，用于生成本框架所需的各个任务
 */
void Reactor46H_TakeOverRTOS()
{
    // SPI 消费线程平时阻塞在这个二值信号量上。
    // 读线程只负责“叫醒它”，不直接承担消费算法。
    if (SpiConsumeSemaphore == nullptr)
    {
        SpiConsumeSemaphore = xSemaphoreCreateBinary();
    }

    xTaskCreate(TaskWrapper, "Control", 256, (void*)ControlCpp, 
                osPriorityNormal, &ControlTaskHandle);

    xTaskCreate(TaskWrapper, "RobotSys", 256, (void*)RobotSystemCpp, 
                osPriorityNormal, &RobotSystemTaskHandle);

    xTaskCreate(TaskWrapper, "SpiRead", 128, (void*)SpiReadCpp, 
                osPriorityNormal, &SpiReadTaskHandle);

    xTaskCreate(TaskWrapper, "SpiConsume", 256, (void*)SpiConsumeCpp,
                osPriorityNormal, &SpiConsumeTaskHandle);

    xTaskCreate(TaskWrapper, "App", 512, (void*)ApplicationCpp, 
                osPriorityNormal, &ApplicationTaskHandle);

    xTaskCreate(TaskWrapper, "StateCore", 512, (void*)StateCoreCpp, 
                osPriorityNormal, &StateCoreTaskHandle);

    // 如果你还需要原来的 4 个协程，可以用循环批量创建
    // for(int i = 0; i < 4; i++) {
    //     // 如果你有 CoroutineCpp 这样的函数，把 CoroutineStub 换掉即可
    //     xTaskCreate(CoroutineStub, "Coroutine", 128, NULL, 
    //                 tskIDLE_PRIORITY + 1, &CoroutineHandles[i]);
    // }
}

void Reactor46H_NotifySpiConsume()
{
    // 这里只做最直接的唤醒，不在这里判断是谁的新数据。
    // 被唤醒后，消费线程会自己把各实例邮箱里的样本全部吃完。
    if (SpiConsumeSemaphore != nullptr)
    {
        xSemaphoreGive(SpiConsumeSemaphore);
    }
}

/**
 * @brief   机器人应用层任务
 * @note    200Hz运行
 */
void ApplicationCpp()
{
    uint32_t AppTick = xTaskGetTickCount();

    while (1)
    {
        // 更新所有应用
        System._Update_Applications();

        /***    最大循环频率：200Hz     ***/
        osDelayUntil(&AppTick, 5); // 200Hz
    }
}

/**
 * @brief       机器人系统主进程
 * @note        负载 `较低`，以200Hz运行，自行分频
 * @warning     这意味着，系统无法分辨200Hz以上的事件
 */
void RobotSystemCpp()
{
    uint32_t AppTick = xTaskGetTickCount();

    while (1)
    {
        // 维护DWT计时器
        DWT_CntUpdate();
        System.Run();

        /***    最大循环频率：200Hz     ***/
        osDelayUntil(&AppTick, 5);
    }
}

/**
 * @brief   机器人的低频循环任务
 * @warning 只承担控制类任务而非逻辑类任务
 * @note    负载 `较低`，以200Hz运行，自行分频
 */
void StateCoreCpp()
{
    uint32_t AppTick = xTaskGetTickCount();
    StateCore &core = StateCore::GetInstance();
    while (1)
    {
        core.Run();
        /***    最大循环频率：250Hz     ***/
        osDelayUntil(&AppTick, 4);
    }
}

/**
 * @brief   机器人的高频循环任务
 * @warning 只承担控制类任务而非逻辑类任务
 * @note    负载 `最高`，以最高1000Hz运行，自行分频
 */
void ControlCpp()
{
    while (1)
    {
        MotorDJI_Driver::ControlAllMotors();
        System.PerformanceRun();
        /***    最大循环频率：1000Hz     ***/
        osDelay(1); // FreeRTOS的极限，1ms喂狗
    }
}


/**
 * @brief 读取SPI数据的任务
 * 
 */
void SpiReadCpp()
{
    uint32_t AppTick = xTaskGetTickCount();

    while (1)
    {
        System._Update_SpiSamps();
        osDelayUntil(&AppTick, 2);
    }
}

void SpiConsumeCpp()
{
    while (1)
    {
        // 平时阻塞等待；一旦有任意 SpiSamp 产出了完整新帧，就起来把当前可消费样本全部处理掉。
        if (SpiConsumeSemaphore != nullptr)
        {
            xSemaphoreTake(SpiConsumeSemaphore, portMAX_DELAY);
        }

        System._Update_SpiConsumes();
    }
}
