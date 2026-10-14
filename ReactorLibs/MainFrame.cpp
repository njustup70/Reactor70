#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "IMUOdom_Test.hpp"
#include "GM6020_Test.hpp"

StateGraph example_graph("graph_name");
void Action_of_Dege(StateCore *core);

GM6020_Test &gm6020_test = GM6020_Test::GetInstance();

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  // 配置状态图为简并模式
  example_graph.Degenerate(Action_of_Dege);

  System.RegistApp(gm6020_test);

  // 向状态机核心注册
  APP::state_core.RegistGraph(example_graph);
  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

void Action_of_Dege(StateCore *core)
{
}