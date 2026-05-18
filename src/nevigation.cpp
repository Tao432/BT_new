#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>

using namespace BT;

class Nevigation : public BT::StatefulActionNode
{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    std::vector<pid_t> pids_; // 存儲所有終端的 PID

    double target_x_ = 0.0, target_y_ = 0.0, target_z_ = 0.0;
    double current_x_ = 0.0, current_y_ = 0.0;
    bool has_odom_ = false;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        has_odom_ = true;
    }

    // 封裝啟動函數
    pid_t launchTerminal(const std::string& script_path) {
        pid_t pid = fork();
        if (pid == 0) { // 子進程
            // 方案 A：完全在后台静默运行（推荐，实车运行最稳定）
            // execlp("ros2", "ros2", "launch", "ego_planner", script_path.c_str(), NULL);
            
            // 方案 B：弹出一个真实的终端窗口（适合调试，需要安装 xterm: sudo apt install xterm）
            execlp("xterm", "xterm", "-hold", "-e", script_path.c_str(), NULL);
            
            // 如果 execlp 失败，子进程直接退出
            perror("execlp failed");
            exit(1);
        }
        return pid; // 返回給父進程記錄
    }

    void stopAllProcesses() {
        if (pids_.empty()) return;

        RCLCPP_INFO(node_->get_logger(), "正在關閉所有導航相關進程...");
        for (pid_t pid : pids_) {
            if (pid > 0) {
                kill(pid, SIGINT); // 溫柔地關閉
            }
        }

        // 給進程 0.5 秒的時間自我清理
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        for (pid_t pid : pids_) {
            if (pid > 0) {
                int status;
                // 檢查是否還活著，如果還活著就強制殺死
                if (waitpid(pid, &status, WNOHANG) == 0) {
                    kill(pid, SIGKILL);
                    waitpid(pid, &status, 0); // 阻塞等待回收
                }
            }
        }
        pids_.clear();
    }
    //xterm关不掉：
    //pkill -9 xterm
    //pkill -9 -f ros2
    // 或者杀掉具体的 python/cpp 进程
    //pkill -9 -f ego_planner
    //pkill -9 -f topic_relay

public:
    Nevigation(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node)
    {
        goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/drone_0_odom", 10, std::bind(&Nevigation::odomCallback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<double>("target_x"), 
            BT::InputPort<double>("target_y"), 
            BT::InputPort<double>("target_z") 
        };
    }

    BT::NodeStatus onStart() override {
        std::cout << "\033[32m[Executing]\033[0m -> " << this->name() << std::endl;
        
        if (!getInput("target_x", target_x_) || !getInput("target_y", target_y_) || !getInput("target_z", target_z_)) {
            RCLCPP_ERROR(node_->get_logger(), "Nevigation 端口输入缺失！");
            return BT::NodeStatus::FAILURE;
        }

        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        msg.header.frame_id = "world";
        msg.pose.position.x = target_x_;
        msg.pose.position.y = target_y_;
        msg.pose.position.z = target_z_;
        msg.pose.orientation.w = 1.0;

        const char* script_path_2 = "/home/tao/BT/ego-planner-realtime-only/start_terminal2.sh";
        const char* script_path_3 = "/home/tao/BT/ego-planner-realtime-only/start_terminal3.sh";
        const char* script_path_4 = "/home/tao/BT/ego-planner-realtime-only/start_terminal4.sh";

        if (pids_.empty()) { // 如果還沒啟動過
            
            // 依次啟動 4 個終端
            pids_.push_back(launchTerminal(script_path_2));
            pids_.push_back(launchTerminal(script_path_3));
            pids_.push_back(launchTerminal(script_path_4));
        
            RCLCPP_INFO(node_->get_logger(), "已啟動全套導航組件，共 %zu 個進程", pids_.size());
        
            // 給系統一點初始化時間
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        }

        // ================= 新增：等待订阅者连接 =================
        int retries = 0;
        // 循环等待，直到检测到至少有 1 个订阅者，最多等 10 次（0.5秒）
        while (goal_pub_->get_subscription_count() == 0 && retries < 10) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            retries++;
        }

        if (goal_pub_->get_subscription_count() == 0) {
            RCLCPP_WARN(node_->get_logger(), "警告：没有检测到 EGO-Planner 订阅目标话题，消息可能会丢失！");
        }
        // ========================================================

        // 稳妥起见，连续发两次，确保万无一失
        goal_pub_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        goal_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), "已向 Area 3 下发目标: [%.2f, %.2f, %.2f]", target_x_, target_y_, target_z_);
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!has_odom_) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "等待里程计...");
            return BT::NodeStatus::RUNNING;
        }

        // 计算当前位置与目标的距离 (XY 平面)
        double dist = std::hypot(target_x_ - current_x_, target_y_ - current_y_);
        std::cout<<dist<<std::endl;

        if (dist < 0.3) {
            RCLCPP_INFO(node_->get_logger(), "[%s] 成功到达目标点附近 (距离: %.2f)", name().c_str(), dist);
            stopAllProcesses(); // 成功时关闭
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        RCLCPP_WARN(node_->get_logger(), "[%s] 导航节点被外部终止", name().c_str());
        stopAllProcesses(); // 中断时关闭
    }
};