#ifndef _STATEMACHINE_HPP_
#define _STATEMACHINE_HPP_

#include "bsp_uart.hpp"
#include "string.h"

class StateCore;        // 为了能引用，进行前向声明
class StateBlock;

#define SINGLETON(x)                \
public:                             \
static x& GetInstance()             \
{                                   \
    static x instance;              \
    return instance;                \
}                                   \
private:                            \
x(const x&) = delete;               \
x& operator=(const x&) = delete;    \
x()


#define PASTE2(a,b) a##b
#define NEW_STATE(core, name) \
    StateBlock PASTE2(St_, name)(#name, PASTE2(Robo, name)); \
    core.AddState(&PASTE2(St_, name));



/**
 * @brief 状态链接，代表了状态之间的转换关系
 */
typedef struct 
{
    bool *condition;                // 条件
    StateBlock *nextState;         // 下一个状态
}StateLink;


/**
 * @brief 状态块，代表了一个状态
 * @warning 状态块是少有的在构造函数中初始化的类，因此绝不能被定义成全局变量
 * 因为这会使得其在HAL库初始化之前就被初始化，从而引发不可预知的错误
 */
class StateBlock
{
    public:
    StateBlock(){};
    StateBlock(const char *name)
    {
        strncpy(this->name, name, 15);
        this->name[15] = 0; // 确保字符串结尾
        linkNums = 0;
    };
    char name[16];                      // 状态名称
    uint8_t id;                         // 状态ID，由StateGraph在创建时赋值
    void (*StateAction)(StateCore *core);          // 状态函数
    StateLink links[16];                 // 状态链接
    uint8_t linkNums;                   // 状态链接数量

    bool Complete;                      // 运行完成标志

    bool LinkTo(bool *condition, StateBlock& nextState);
    uint8_t Transition();
};


/**
 * @brief 状态图，包含了一系列状态块 
 */
class StateGraph
{
    friend class StateCore;
    private:
    void (*GlobalAction)(StateCore *core);    // 全局状态函数

    public:
    char name[16];                  // 状态图名称
    StateBlock states[24];          // 状态块数组（上限24个）
    uint8_t stateNums;              // 状态块数量

    uint8_t executor_at_id = 0;     // 当前执行的状态ID
    // StateBlock& current_state = states[0]; // 当前状态块引用
    StateBlock* current_state = &states[0]; // 当前状态块引用，修改为指针 
       
    public:
    StateGraph(const char *name){
        stateNums = 0;
        strncpy(this->name, name, 16);
        this->name[15] = 0;          // 确保字符串结尾
    };
    void SetGlobalAct(void (*GlobalAction)(StateCore *core));
    bool Degenerate(void (*DegenAction)(StateCore *core));

    StateBlock& AddState(const char *name);
};



/**
 * @brief 状态机核心，即状态机本体
 */
class StateCore
{
    SINGLETON(StateCore){};

    private:
    /// @brief 状态机核心是否启动
    bool _enabled = false;
    /// @brief dwt计时器用句柄
    uint32_t _dwt_tick;
    /// @brief 两次状态切换的时间间隔，单位秒
    float _dt;                   

    /// @brief 在每个状态执行周期开始时，响应对应的状态图切换请求
    void Apply_SwitchGraph();

    /// @brief 是否存在待处理的状态图切换请求
    bool _switch_graph_pending = false;
    /// @brief 待切换目标状态图ID
    uint8_t _switch_graph_targ_graph_id = 0;
    /// @brief 待切换目标状态ID（在目标图中的ID）
    uint8_t _switch_graph_targ_state_id = 0;

    public:
    /// @brief 循环运行状态机核心 
    void Run(void);

    /// @brief 启动状态机核心，可指定初始状态图
    void Enable(uint8_t first_graph = 0);
    
    /// @brief 注册状态图
    /// @return 注册成功返回图ID，失败返回-1
    int RegistGraph(StateGraph& graph);

    /**
     * @brief 转移到目标状态图的指定状态
     * 
     * @param targ_graph_id 目标的状态图的ID
     * @param targ_state_id 在下个状态函数执行周期开始，即将转换到的状态块，在其对应图中的ID
     * @note 这个函数并不会立即切换状态图，而是会在下一个状态函数执行周期开始时切换。
     */
    void SwitchGraph(uint8_t targ_graph_id, uint8_t targ_state_id);

    /**
     * @brief 上方的字符串查询重载版本
     */
    void SwitchGraph(const char* graph_name, const char* state_name);

    /// @brief 获得当前状态的引用
    StateBlock& GetCurState();
    
    /// @brief 所掌控的状态图
    StateGraph* graphs[16];
    /// @brief 当前所处状态图
    uint8_t at_graph_id;
    /// @brief 注册的状态图数量
    uint8_t graph_nums = 0;
    
    /// @brief 根据状态图输出Mermaid代码
    static void CoreGraph(const StateGraph& graph);
};


namespace Seq
{
    void Wait(float sec);
    void WaitUntil(bool& condition, float timeout_sec = 3600.0f);
    
    using CheckFunctionPtr = bool(*)(void*);

    namespace _Private 
    {
        /**
         * @brief 等待直到（私有代理函数）
         * @param check_func_ptr: 实际检查函数的地址
         * @param context: 传递给检查函数的Lambda闭包
         * @param timeout_sec: 超时时间，单位秒
         * @note 注意：Lambda 对象 'condition' 是在栈上创建的，它的生命周期仅在 WaitUntil 期间有效。
         */
        void WaitUntil_Impl(CheckFunctionPtr check_func_ptr, void* context, float timeout_sec);
    }
    

    /**
     * @brief 等待直到（表达式重载）
     * @param condition 布尔条件表达式 (Lambda/Functor)
     * @param timeout_sec 超时时间，单位秒
     */
    template<typename ConditionFunc>
    void WaitUntil(ConditionFunc condition, float timeout_sec = 3600.0f)
    {
        // 注意：Lambda 闭包对象会被传递到这里作为 context
        static auto callback_adapter = [](void* context) -> bool {
            // 将 void* 转换回原始的 Lambda 指针
            ConditionFunc* lambda_ptr = static_cast<ConditionFunc*>(context);
            // 执行 Lambda 返回结果
            return (*lambda_ptr)();
        };

        // 调用私有代理
        _Private::WaitUntil_Impl(callback_adapter, &condition, timeout_sec);
    }
}

namespace APP
{
    extern StateCore& state_core;
}



#endif
