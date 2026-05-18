#include <behaviortree_cpp/behavior_tree.h>
#include <iostream>

using namespace BT;

// 判断是否足够高的条件节点
class IsHighEnough : public BT::ConditionNode
{
public:
    IsHighEnough(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {}; // 如果需要从黑板读取高度阈值，可以在这里定义 InputPort
    }

    BT::NodeStatus tick() override {
        std::cout << "[IsHighEnough] 正在检测机器人当前高度..." << std::endl;
    
        // 测试逻辑：假设当前高度不够，返回 FAILURE
        // 这样外层的 <reverse> 就会得到 SUCCESS，进而去执行 <ClimbOnR1>
        bool high_enough = false;

        if (high_enough) {
            std::cout << "[IsHighEnough] 高度达标！" << std::endl;
            return NodeStatus::SUCCESS;
        } else {
            std::cout << "[IsHighEnough] 高度不足！需要爬升。" << std::endl;
            return NodeStatus::FAILURE;
        }
    }
};