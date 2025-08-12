#include "dobot_msgs_v4/srv/run_script.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using RunScript = dobot_msgs_v4::srv::RunScript;
using namespace std::chrono_literals;

class ScriptRunnerNode : public rclcpp::Node
{
public:
    ScriptRunnerNode()
        : Node("script_runner"), service_called_(false)
    {
        client_ = this->create_client<RunScript>("/dobot_bringup_ros2/srv/RunScript");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&ScriptRunnerNode::delayed_start, this));
    }

private:
    void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !service_called_)
        {
            RCLCPP_INFO(this->get_logger(), "Trigger received! Calling RunScript service...");

            if (!client_->wait_for_service(2s))
            {
                RCLCPP_WARN(this->get_logger(), "RunScript service not available...");
                return;
            }

            auto request = std::make_shared<RunScript::Request>();
            request->project_name = "straightmoveupdate";

            auto result = client_->async_send_request(request);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result.future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "Service called successfully!");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call RunScript service.");
            }

            service_called_ = true; // prevent repeated calls
        }
    }

    void delayed_start()
    {
        // Subscribe to trigger topic
        sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start_repair",
            10,
            std::bind(&ScriptRunnerNode::trigger_callback, this, std::placeholders::_1));
        timer_->cancel();
    }

    rclcpp::Client<RunScript>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool service_called_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScriptRunnerNode>());
    rclcpp::shutdown();
    return 0;
}
