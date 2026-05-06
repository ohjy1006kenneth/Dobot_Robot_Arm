#include "dobot_msgs_v4/srv/run_script.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using RunScript = dobot_msgs_v4::srv::RunScript;
using Trigger = std_srvs::srv::Trigger;
using namespace std::chrono_literals;

class ScriptRunnerNode : public rclcpp::Node
{
public:
    ScriptRunnerNode()
        : Node("script_runner")
    {
        service_name_ = this->declare_parameter<std::string>(
            "service_name", "/dobot_bringup_ros2/srv/RunScript");
        project_name_ = this->declare_parameter<std::string>("project_name", "Scanner");
        retry_period_ms_ = this->declare_parameter<int>("retry_period_ms", 1000);
        trigger_cooldown_ms_ = this->declare_parameter<int>("trigger_cooldown_ms", 3000);

        client_ = this->create_client<RunScript>(service_name_);
        scanner_client_ = this->create_client<Trigger>("/scanner/trigger_scan");
        arm_started_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/arm_script_started", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start_repair",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&ScriptRunnerNode::trigger_callback, this, std::placeholders::_1));

        ready_service_ = this->create_service<Trigger>(
            "/script_runner/ready",
            std::bind(&ScriptRunnerNode::ready_callback, this, std::placeholders::_1, std::placeholders::_2));

        retry_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(retry_period_ms_),
            std::bind(&ScriptRunnerNode::try_run_script, this));
        retry_timer_->cancel();

        RCLCPP_INFO(
            this->get_logger(),
            "script_runner ready. Waiting for /start_repair=true to run Dobot project '%s'.",
            project_name_.c_str());
    }

private:
    void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data) {
            return;
        }

        const auto now = this->now();
        if (have_last_accepted_trigger_ &&
            (now - last_accepted_trigger_time_).nanoseconds() <
            static_cast<int64_t>(trigger_cooldown_ms_) * 1000000) {
            return;
        }

        have_last_accepted_trigger_ = true;
        last_accepted_trigger_time_ = now;

        if (!scanner_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Trigger rejected. /scanner/trigger_scan is not available, so the arm will not move.");
            publish_arm_started(false);
            return;
        }

        if (!client_->wait_for_service(1s)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Trigger rejected. RunScript service '%s' is not available.",
                service_name_.c_str());
            publish_arm_started(false);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Trigger received. Calling RunScript service...");
        call_run_script();
    }

    void call_run_script()
    {
        if (service_call_in_progress_) {
            return;
        }

        retry_timer_->cancel();
        service_call_in_progress_ = true;

        auto request = std::make_shared<RunScript::Request>();
        request->project_name = project_name_;

        client_->async_send_request(
            request,
            [this](rclcpp::Client<RunScript>::SharedFuture future) {
                service_call_in_progress_ = false;
                try {
                    const auto response = future.get();
                    RCLCPP_INFO(
                        this->get_logger(),
                        "RunScript('%s') service returned res=%d.",
                        project_name_.c_str(),
                        response->res);
                    publish_arm_started(true);
                } catch (const std::exception & ex) {
                    RCLCPP_ERROR(this->get_logger(), "RunScript service call failed: %s", ex.what());
                    publish_arm_started(false);
                }
            });
    }

    void try_run_script()
    {
        // Kept for compatibility with older pending requests; normal /start_repair no longer retries.
        call_run_script();
    }

    void ready_callback(
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
    {
        (void)request;
        const bool dobot_ready = client_->service_is_ready();
        const bool scanner_ready = scanner_client_->service_is_ready();

        response->success = dobot_ready && scanner_ready;
        if (response->success) {
            response->message = "script_runner ready: Dobot RunScript and scanner trigger services are available";
        } else {
            response->message = "script_runner not ready: missing Dobot RunScript or scanner trigger service";
        }
    }

    void publish_arm_started(bool started)
    {
        std_msgs::msg::Bool msg;
        msg.data = started;
        arm_started_pub_->publish(msg);
    }

    rclcpp::Client<RunScript>::SharedPtr client_;
    rclcpp::Client<Trigger>::SharedPtr scanner_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_started_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    rclcpp::Service<Trigger>::SharedPtr ready_service_;
    rclcpp::TimerBase::SharedPtr retry_timer_;

    std::string service_name_;
    std::string project_name_;
    int retry_period_ms_;
    int trigger_cooldown_ms_;

    bool service_call_in_progress_{false};
    bool have_last_accepted_trigger_{false};
    rclcpp::Time last_accepted_trigger_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScriptRunnerNode>());
    rclcpp::shutdown();
    return 0;
}
