#include "StateCore.hpp"
#include "bsp_dwt.hpp"
#include "stdio.h"
#include "Monitor.hpp"
#include "bsp_log.hpp"
#include "cmsis_os.h"
#include "cstring"

StateCore& APP::state_core = StateCore::GetInstance();

/**
 * @brief 设置全局状态函数，其会先于每个状态函数执行
 * 
 * @param GlobalAction 
 */
void StateGraph::SetGlobalAct(void (*GlobalAction)(StateCore *core))
{
    this->GlobalAction = GlobalAction;
}

/**
 * @brief 为状态块添加状态链接
 */
bool StateBlock::LinkTo(bool *condition, StateBlock& nextState)
{
    if (linkNums < 16)
    {
        links[linkNums].condition = condition;
        links[linkNums].nextState = &nextState;
        linkNums++;
        return true;
    }
    return false;
}


/**
 * @brief 进行状态转移，返回下一个状态ID
 * @return 如果没有状态转移则返回当前状态ID，否则返回下一个状态ID
 */
uint8_t StateBlock::Transition()
{
    for (uint8_t i = 0; i < linkNums; i++)
    {
        if (*(links[i].condition))
        {
            return links[i].nextState->id;
        }
    }
    return id;
}

/**
 * @brief 向状态图中添加状态块
 */
StateBlock& StateGraph::AddState(const char *name)
{
    if (stateNums < 24)
    {
        // 获取目标状态
        StateBlock& targ = states[stateNums];
        
        // 初始化
        targ = StateBlock(name);
        targ.id = stateNums;
        targ.Complete = false;

        // 增加状态数量
        stateNums++;
        return targ;
    }

    // 返回第一个状态，表示添加失败
    return states[0];
}


/**
 * @brief 状态机的简并初始化，一般用于调试
 * @details 只有两个状态：working和end
 */
bool StateGraph::Degenerate(void (*DegenAction)(StateCore *core))
{
    // 固定两个状态

    // 第一个状态：工作状态
    StateBlock& state_work = AddState("working");
    state_work.StateAction = DegenAction;
    state_work.LinkTo(&(states[0].Complete), states[1]);

    // 第二个状态：结束状态
    StateBlock& state_end = AddState("end");
    state_end.StateAction = nullptr; // 什么都不做

    return true;
}


/**
 * @brief 运行状态机
 */
void StateCore::Run()
{
    // 避免空运行
    if (graph_nums == 0 || !_enabled) return;
    
    // 计算时间间隔
    _dt = DWT_GetDeltaTime(&_dwt_tick);

    // 如果有图切换请求，在每个状态执行周期开始时进行响应
    Apply_SwitchGraph();

    // 执行 当前状态图 的 对应状态 的 状态函数
    StateGraph& graph = *graphs[at_graph_id];
    StateBlock* state = graph.current_state;

    // 执行当前状态图的全局状态函数
    if (graph.GlobalAction != nullptr) graph.GlobalAction(this);
    
    /**
     * @note 这个写法代表着，一般只有状态函数完全执行完了才会进行状态转移
     * 所以后面应该会加入 在中间打断动作 的机制（确保动作打断是经过作者设计的）
     * @warning 只有非空状态函数才会被执行 
     */
    if (state->StateAction != nullptr) state->StateAction(this);

    // 进行状态转移
    uint8_t next_id = state->Transition();
    if (next_id != graph.executor_at_id) 
    {
        // 存储名称用于日志输出
        char current_state_name[16];
        strncpy(current_state_name, graph.current_state->name, 15);
        current_state_name[15] = '\0';

        graph.executor_at_id = next_id;
        graph.current_state = &graph.states[next_id]; // 指针重定向

        // 输出日志
        APP::monit.LogInfo("[State] Transform From %s to %s", current_state_name, graph.current_state->name);
    }
}

void StateCore::Apply_SwitchGraph()
{
    // 如果没有待处理的图切换请求，直接返回
    if (!_switch_graph_pending) return;

    // 重置图切换请求标志，开始处理请求
    _switch_graph_pending = false;

    // 防越界
    if (_switch_graph_targ_graph_id >= graph_nums)
    {
        APP::monit.LogWarning(
            "[State] can't switch, graph_id:%d out of range!",
            _switch_graph_targ_graph_id, graph_nums
        );
        return;
    }
    StateGraph& targ_graph = *graphs[_switch_graph_targ_graph_id];
    if (_switch_graph_targ_state_id >= targ_graph.stateNums)
    {
        APP::monit.LogWarning(
            "[State] can't switch, state_id:%d out of range in graph %s",
            _switch_graph_targ_state_id, targ_graph.name
        );
        return;
    }

    uint8_t old_graph_id = at_graph_id;
    StateGraph& old_graph = *graphs[old_graph_id];
    
    // 转到新的状态图
    at_graph_id = _switch_graph_targ_graph_id;
    targ_graph.executor_at_id = _switch_graph_targ_state_id;
    targ_graph.current_state = &targ_graph.states[_switch_graph_targ_state_id];

    // 转换成功日志
    APP::monit.LogInfo(
        "[State] switch %s -> %s, enter %s(%d)",
        old_graph.name, targ_graph.name,
        targ_graph.current_state->name, _switch_graph_targ_state_id
    );
}

void StateCore::Enable(uint8_t first_graph)
{
    if (graph_nums == 0)
    {
        APP::monit.LogWarning("StateCore: no graph registered, enable failed.");
        _enabled = false;
        return;
    }

    if (first_graph >= graph_nums)
    {
        APP::monit.LogWarning(
            "StateCore: first_graph id %d out of range (graph_nums=%d), fallback to 0.",
            first_graph, graph_nums
        );
        first_graph = 0;
    }

    at_graph_id = first_graph;
    _enabled = true;
}

StateBlock& StateCore::GetCurState()
{
    StateGraph& graph = *graphs[at_graph_id];
    // return graph.current_state; 
    return *graph.current_state; 
}


/**
 * @brief 绘制状态机图
 * @details 通过遍历整个状态机，将状态机的状态和状态转换关系以图的形式发送到指定串口上 'Mermaid' 
 * @warning 该函数会阻塞程序运行！！因此禁止在线程中调用，仅做Debug用途
 */
void StateCore::CoreGraph(const StateGraph& graph)
{
    BspLog_LogSpec("StateGraph");
    for (uint8_t i = 0; i < graph.stateNums; i++)
    {
        // 发送状态转换关系（mermaid格式）
        for (uint8_t j = 0; j < graph.states[i].linkNums; j++)
        {
            BspLog_LogSpec("%s --> %s",
                graph.states[i].name,
                graph.states[i].links[j].nextState->name);
        }
    }
}


/**
 * @brief 注册状态图
 */
int StateCore::RegistGraph(StateGraph& graph)
{
    if (graph_nums < 16)
    {
        int graph_id = graph_nums;
        graphs[graph_nums] = &graph;
        graph_nums++;
        return graph_id;
    }
    else
    {
        Monitor::GetInstance().LogWarning("StateCore: Too much state graph!");
        return -1;
    }
}

void StateCore::SwitchGraph(uint8_t targ_graph_id, uint8_t targ_state_id)
{
    _switch_graph_targ_graph_id = targ_graph_id;
    _switch_graph_targ_state_id = targ_state_id;
    _switch_graph_pending = true;
}

void StateCore::SwitchGraph(const char* graph_name, const char* state_name)
{
    if (graph_name == nullptr || state_name == nullptr)
    {
        APP::monit.LogWarning("[State] SwitchGraph rejected: null graph/state name.");
        return;
    }

    for (uint8_t gid = 0; gid < graph_nums; gid++)
    {
        StateGraph& graph = *graphs[gid];
        if (std::strcmp(graph.name, graph_name) != 0) continue;

        for (uint8_t sid = 0; sid < graph.stateNums; sid++)
        {
            if (std::strcmp(graph.states[sid].name, state_name) == 0)
            {
                SwitchGraph(gid, sid);
                return;
            }
        }

        APP::monit.LogWarning(
            "[State] SwitchGraph rejected: state %s not found in graph %s.",
            state_name, graph_name
        );
        return;
    }

    APP::monit.LogWarning(
        "[State] SwitchGraph rejected: graph %s not found.",
        graph_name
    );
}


namespace Seq
{
    /** 
     * @brief 等待指定时间
     * @param sec 等待时间，单位秒
     * @details 利用osDelay实现
     */
    void Wait(float sec)
    {
        osDelay((uint32_t)(sec * 1000));
    }
    
    /**
     * @brief 等待直到条件满足或超时
     * @param condition 指向布尔条件的引用
     * @param timeout_sec 超时时间，单位秒，默认300秒
     * @details 利用阻塞+让步实现
     */
    void WaitUntil(bool& condition, float timeout_sec)
    {
        uint32_t start_tick = xTaskGetTickCount();
        uint32_t timeout_ticks = (uint32_t)(timeout_sec * 1000);

        // 存储当前任务优先级
        osPriority original_priority = osThreadGetPriority(osThreadGetId());
        // 降低任务优先级，避免死循环占用过多CPU时间
        osThreadSetPriority(osThreadGetId(), osPriorityIdle);

        while (!condition)
        {
            // 检查超时
            uint32_t current_tick = xTaskGetTickCount();
            if ((current_tick - start_tick) >= timeout_ticks)
            {
                break; // 超时退出
            }

            // 让出CPU时间片，避免死循环占用过多资源
            osThreadYield();
        }

        // 恢复任务优先级
        osThreadSetPriority(osThreadGetId(), original_priority);
    }

    namespace _Private
    {
        // 私有代理函数的实现
        void WaitUntil_Impl(CheckFunctionPtr check_func_ptr, void* context, float timeout_sec)
        {
            uint32_t start_tick = xTaskGetTickCount();
            uint32_t timeout_ticks = (uint32_t)(timeout_sec * 1000);

            // 存储当前任务优先级
            osThreadId current_thread_id = osThreadGetId();
            osPriority original_priority = osThreadGetPriority(current_thread_id);
            // 降低任务优先级，避免死循环占用过多CPU时间
            osThreadSetPriority(osThreadGetId(), osPriorityIdle);
            
            // 使用传入的函数指针和上下文来检查条件
            while (!check_func_ptr(context)) 
            {
                // 检查超时
                uint32_t current_tick = xTaskGetTickCount();
                if ((current_tick - start_tick) >= timeout_ticks)
                {
                    break; 
                }

                osThreadYield();
            }

            // 恢复任务优先级
            osThreadSetPriority(current_thread_id, original_priority);
        }
    }
}
