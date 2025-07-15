#include "dobot_msgs_v4/srv/run_script.hpp"
#include "rclcpp/rclcpp.hpp"

using RunScript = dobot_msgs_v4::srv::RunScript;
using namespace std::chrono_literals;

class ScriptRunnerNode : public rclcpp::Node {
public:
    ScriptRunnerNode()
        : Node("script_runner")
    {
        client_ = this->create_client<RunScript>("/dobot_bringup_ros2/srv/RunScript");

        timer_ = this->create_wall_timer(2s, [this]() {
            if (!client_->wait_for_service(1s)) {
                RCLCPP_WARN(this->get_logger(), "Service not available...");
                return;
            }

            auto request = std::make_shared<RunScript::Request>();
            request->project_name = "straightmove";

            auto result = client_->async_send_request(request);

            // Wait for the future to complete (blocking)
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result.future) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Service called successfully!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to call service");
            }
        });
    }

private:
    rclcpp::Client<RunScript>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScriptRunnerNode>());
    rclcpp::shutdown();
    return 0;
}
