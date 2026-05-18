#include <behaviortree_cpp/behavior_tree.h>
#include <iostream>

using namespace BT;

class PutKfs : public BT::SyncActionNode
{
public:
    PutKfs(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        std::cout << "[PutKfs] 准备放置 KFS..." << std::endl;
    
        // TODO: 调用机械臂或释放机构的控制代码
    
        std::cout << "[PutKfs] 成功放置 KFS！" << std::endl;
        return NodeStatus::SUCCESS;
    }
};