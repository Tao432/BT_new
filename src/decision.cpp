#include <behaviortree_cpp/behavior_tree.h>
#include <iostream>

using namespace BT;

// 决策节点 
class Dicision : public BT::SyncActionNode
{
public:
    Dicision(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        // 定义一个输出端口，对应 XML 中的变量
        return { BT::OutputPort<std::string>("kfs_location") };
    }

    BT::NodeStatus tick() override {
        std::cout << "\n[Dicision] 正在执行决策计算..." << std::endl;
    
        // 测试逻辑：假设经过视觉或传感器判断，我们有 kfs
        std::string has_kfs = "YES"; 
    
        // 将结果写入黑板
        setOutput("have_kfs", has_kfs);
    
        std::cout << "[Dicision] 决策完成，have_kfs = " << has_kfs << std::endl;
    
        return NodeStatus::SUCCESS;
    }
};