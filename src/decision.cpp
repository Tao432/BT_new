#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>

class DecisionNode : public BT::StatefulActionNode
{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ai_sub_;
    
    bool has_decision_ = false;
    int target_row_ = -1;
    int target_col_ = -1;

    void aiCallback(const std_msgs::msg::String::SharedPtr msg) {
        // 如果当前周期已经锁定了决策，忽略新消息，直到下一轮开始
        if (has_decision_) return; 

        std::istringstream iss(msg->data);
        std::string action;
        int row, col;
        
        // 解析字符串 "place 0 0"
        if (iss >> action >> row >> col) {
            if (action == "place" && row >= 0 && row < 2) { // 过滤规则：本机器人只能 place
                target_row_ = row;
                target_col_ = col;
                has_decision_ = true; // 锁定！
            }
        }
    }

public:
    DecisionNode(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node)
    {
        ai_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/ai/best_move", 10, std::bind(&DecisionNode::aiCallback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts() {
        return { 
            BT::OutputPort<double>("out_target_y"), 
            BT::OutputPort<int>("out_row") // 将行号输出给行为树
        };
    }

    BT::NodeStatus onStart() override {
        has_decision_ = false; // 每次执行到这里时，重置锁定状态，准备接收新决策
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!has_decision_) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "等待合法 place 决策...");
            return BT::NodeStatus::RUNNING;
        }

        // --- 坐标转换逻辑 ---
        // 假设 Y 轴向左为正，中心列 col=1 的 y=0
        // col=0 -> y = (1-0)*0.54 = 0.54
        // col=1 -> y = (1-1)*0.54 = 0.0
        // col=2 -> y = (1-2)*0.54 = -0.54
        double target_y = (1 - target_col_) * 0.54;

        setOutput("out_target_y", target_y);
        setOutput("out_row", target_row_);
        
        RCLCPP_INFO(node_->get_logger(), "🎯 锁定决策: row=%d, col=%d, 对应 Y=%.2fm", 
                    target_row_, target_col_, target_y);
        
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override { has_decision_ = false; }
};