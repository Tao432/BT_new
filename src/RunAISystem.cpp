#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>

class RunAISystem : public BT::StatefulActionNode
{
private:
    pid_t ai_pid_ = -1;
    rclcpp::Node::SharedPtr node_;

    void stopAIProcess() {
        if (ai_pid_ > 0) {
            RCLCPP_INFO(node_->get_logger(), "正在关闭 AI 视觉节点 (PID: %d)...", ai_pid_);
            // 杀掉整个进程组 (必须加负号)
            kill(-ai_pid_, SIGINT); 
            
            // 给它一点时间释放摄像头
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            int status;
            if (waitpid(ai_pid_, &status, WNOHANG) == 0) {
                kill(-ai_pid_, SIGKILL);
                waitpid(ai_pid_, &status, 0);
            }
            ai_pid_ = -1;
        }
    }

public:
    RunAISystem(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override {
        if (ai_pid_ == -1) {
            ai_pid_ = fork();
            if (ai_pid_ == 0) { // 子进程
                // 建立新的进程组
                setpgid(0, 0); 
                
                // 执行启动脚本 (这里写你的 start_terminal5.sh 路径)
                execlp("xterm", "xterm", "-e", "/home/tao/BT/scripts/start_terminal5.sh", NULL);
                exit(1);
            }
            RCLCPP_INFO(node_->get_logger(), "已在此子树拉起 AI 视觉系统...");
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        // 这个节点永远返回 RUNNING，作为后台守护进程，直到被别人 Halt
        return BT::NodeStatus::RUNNING; 
    }

    void onHalted() override {
        // 当右边的 Sequence 成功或失败导致 Parallel 结束时，会自动调用这里
        stopAIProcess();
    }
};