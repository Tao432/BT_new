#include <behaviortree_cpp/behavior_tree.h>
#include <iostream>

using namespace BT;

class ClimbOffR1 : public BT::SyncActionNode
{
public:
    ClimbOffR1(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        std::cout << "[ClimbOffR1] 开始执行下降动作 (Climbing off R1)..." << std::endl;
    
    // TODO: 之后这里可以调用下位机服务或发布特定 cmd_vel
    
    // 模拟动作耗时 (可选)
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    
        std::cout << "[ClimbOffR1] 下降动作完成！" << std::endl;
        return NodeStatus::SUCCESS;
    }
};