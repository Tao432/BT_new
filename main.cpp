#include <iostream>
#include <behaviortree_cpp/bt_factory.h> // 必须包含这个
#include <rclcpp/rclcpp.hpp>

// 包含你所有自定义节点的头文件
// 注意：如果你的头文件在 include 目录下，CMakeLists.txt 里的 include_directories(include) 会起作用
#include "nevigation.cpp"
#include "decision.cpp"
#include "isHighEnough.cpp"
#include "climbOnR1.cpp"
#include "putKfs.cpp"

using namespace BT;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("bt_node");

    BT::BehaviorTreeFactory factory;

    // 1. 注册所有的自定义节点
    factory.registerBuilder<Nevigation>("Nevigation", 
    [ros_node](const std::string& name, const NodeConfiguration& config) {
        return std::make_unique<Nevigation>(name, config, ros_node);
    });
    factory.registerNodeType<Dicision>("Dicision");
    factory.registerNodeType<IsHighEnough>("IsHighEnough");
    factory.registerNodeType<ClimbOnR1>("ClimbOnR1");
    factory.registerNodeType<PutKfs>("PutKfs");

    // 2. 从 XML 加载树形结构 (假设文件名为 subtree_area3.xml)
    // 注意：如果有子树 "return_area2"，也需要确保它被加载或注册
    try {
        auto tree = factory.createTreeFromFile("../include/subtree_area3.xml");

        // 3. 运行行为树
        std::cout << "--- 开始运行行为树 ---" << std::endl;
        rclcpp::WallRate loop_rate(10); // 10Hz
        while (rclcpp::ok()) {
        //    std::cout << "--- Ticking ---" << std::endl;
        //    auto status = tree.tickOnce();
    
          // 如果根节点返回 SUCCESS 或 FAILURE，树就执行完了
        //    if (status != BT::NodeStatus::RUNNING) {
        //    std::cout << "行为树运行结束，最终状态: " << status << std::endl;
        //    break; 
        //    }

        //    rclcpp::spin_some(ros_node);
        //    loop_rate.sleep();
        auto status = tree.tickOnce();
    
        if (status == BT::NodeStatus::SUCCESS) {
            std::cout << ">>> 任务全部成功完成！" << std::endl;
            break; 
        } 
        else if (status == BT::NodeStatus::FAILURE) {
            std::cout << ">>> 任务执行失败，3秒后重试..." << std::endl;
            tree.haltTree(); // 关键：重置所有节点状态，准备下一次 tick
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }

        rclcpp::spin_some(ros_node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } 
    catch (const std::exception& e) {
        std::cerr << "加载或运行行为树时出错: " << e.what() << std::endl;
    }

    return 0;
}