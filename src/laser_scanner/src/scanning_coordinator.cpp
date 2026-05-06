// scanning_coordinator.cpp
//
// Orchestrates: /start_repair → 3 arm scans → /map_merger/add_block (auto-merge)
// No position limit — trigger as many times as needed.
// Call /map_merger/save when done to write merged_map.pcd.

#include "rclcpp/rclcpp.hpp"
#include "dobot_msgs_v4/srv/run_script.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include <cmath>

using Trigger = std_srvs::srv::Trigger;
using Empty   = std_srvs::srv::Empty;
using RunScript = dobot_msgs_v4::srv::RunScript;
using namespace std::chrono_literals;

class ScanningCoordinatorNode : public rclcpp::Node
{
public:
    ScanningCoordinatorNode()
        : Node("scanning_coordinator"),
          // ======== ADJUST THESE VARIABLES AS NEEDED ========
          num_scans_(3),                    // arm strips per block
          delay_before_first_scan_(2.4),    // seconds before first trigger
          delay_between_scans_(15.0),       // seconds between arm strips
          block_ready_timeout_(20.0),       // seconds to wait for accumulator to finish block publish
          arm_ack_timeout_(5.0),             // seconds to wait for script_runner acknowledgement
          // ===================================================
          current_scan_(0),
          block_count_(0),
          sequence_active_(false),
          waiting_for_arm_ack_(false),
          waiting_for_block_ready_(false),
          block_ready_request_in_flight_(false),
          block_ready_attempts_left_(0)
    {
        scan_client_      = this->create_client<Trigger>("/scanner/trigger_scan");
        add_block_client_ = this->create_client<Trigger>("/map_merger/add_block");
        arm_script_client_ = this->create_client<RunScript>("/dobot_bringup_ros2/srv/RunScript");
        script_runner_ready_client_ = this->create_client<Trigger>("/script_runner/ready");
        block_ready_client_ = this->create_client<Trigger>("/scanner/block_ready");

        sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start_repair", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&ScanningCoordinatorNode::trigger_callback, this, std::placeholders::_1));

        arm_started_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/arm_script_started", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&ScanningCoordinatorNode::arm_started_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "[COORDINATOR] Ready — publish /start_repair to begin a 3-scan block.\n"
            "  Repeat as many times as needed at any position.\n"
            "  Call /map_merger/save when finished to write merged_map.pcd");
    }

private:
    void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data || sequence_active_) return;

        if (!systems_ready()) {
            RCLCPP_ERROR(this->get_logger(),
                "[COORDINATOR] Start rejected. Arm script and scanner must both be ready; nothing was started.");
            return;
        }

        block_count_++;
        RCLCPP_INFO(this->get_logger(),
            "[COORDINATOR] ▶ Block #%d — starting %d-scan sequence", block_count_, num_scans_);

        sequence_active_ = true;
        waiting_for_arm_ack_ = true;
        current_scan_ = 1;

        RCLCPP_INFO(this->get_logger(),
            "[COORDINATOR] Waiting for script_runner arm-start acknowledgement before triggering scanner...");

        arm_ack_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(arm_ack_timeout_ * 1000)),
            [this]() { arm_ack_timer_->cancel(); arm_ack_timeout(); });
    }

    bool systems_ready()
    {
        if (!scan_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "[COORDINATOR] /scanner/trigger_scan unavailable.");
            return false;
        }

        if (!arm_script_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "[COORDINATOR] Dobot RunScript service unavailable.");
            return false;
        }

        if (!script_runner_ready_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "[COORDINATOR] /script_runner/ready unavailable.");
            return false;
        }

        if (!block_ready_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "[COORDINATOR] /scanner/block_ready unavailable.");
            return false;
        }

        return true;
    }

    void arm_started_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!sequence_active_ || !waiting_for_arm_ack_) {
            return;
        }

        if (!msg->data) {
            RCLCPP_ERROR(this->get_logger(),
                "[COORDINATOR] script_runner rejected arm start. Scanner sequence will not start.");
            waiting_for_arm_ack_ = false;
            sequence_active_ = false;
            if (arm_ack_timer_) arm_ack_timer_->cancel();
            return;
        }

        waiting_for_arm_ack_ = false;
        if (arm_ack_timer_) arm_ack_timer_->cancel();

        RCLCPP_INFO(this->get_logger(),
            "[COORDINATOR] Arm script acknowledged. Scanner sequence armed.");

        delay_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(delay_before_first_scan_ * 1000)),
            [this]() { delay_timer_->cancel(); trigger_next_scan(); });
    }

    void arm_ack_timeout()
    {
        if (!sequence_active_ || !waiting_for_arm_ack_) {
            return;
        }

        RCLCPP_ERROR(this->get_logger(),
            "[COORDINATOR] No /arm_script_started acknowledgement received. Scanner sequence will not start.");
        waiting_for_arm_ack_ = false;
        sequence_active_ = false;
    }

    void trigger_next_scan()
    {
        if (current_scan_ > num_scans_)
        {
            begin_wait_for_block_ready();
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "[COORDINATOR] Triggering strip %d/%d...", current_scan_, num_scans_);

        if (current_scan_ > 1)
        {
            delay_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(delay_between_scans_ * 1000)),
                [this]() { delay_timer_->cancel(); trigger_scan(); });
        }
        else
        {
            trigger_scan();
        }
    }

    void trigger_scan()
    {
        if (!scan_client_->wait_for_service(2s))
        {
            RCLCPP_ERROR(this->get_logger(), "[COORDINATOR] Scanner service unavailable!");
            sequence_active_ = false;
            return;
        }
        scan_client_->async_send_request(
            std::make_shared<Trigger::Request>(),
            [this](rclcpp::Client<Trigger>::SharedFuture future) {
                auto resp = future.get();
                if (resp->success) {
                    RCLCPP_INFO(this->get_logger(),
                        "[COORDINATOR] ✓ Strip %d/%d done", current_scan_, num_scans_);
                    current_scan_++;
                    trigger_next_scan();
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                        "[COORDINATOR] ✗ Strip %d failed: %s", current_scan_, resp->message.c_str());
                    sequence_active_ = false;
                }
            });
    }

    void add_block()
    {
        waiting_for_block_ready_ = false;
        block_ready_request_in_flight_ = false;
        if (block_ready_timer_) {
            block_ready_timer_->cancel();
        }

        RCLCPP_INFO(this->get_logger(),
            "[COORDINATOR] Block #%d complete — merging into world map...", block_count_);

        if (!add_block_client_->wait_for_service(2s))
        {
            RCLCPP_ERROR(this->get_logger(), "[COORDINATOR] /map_merger/add_block unavailable!");
            sequence_active_ = false;
            return;
        }
        add_block_client_->async_send_request(
            std::make_shared<Trigger::Request>(),
            [this](rclcpp::Client<Trigger>::SharedFuture future) {
                auto resp = future.get();
                if (resp->success) {
                    RCLCPP_INFO(this->get_logger(),
                        "[COORDINATOR] ✓ %s\n"
                        "  → Publish /start_repair again or call /map_merger/save",
                        resp->message.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                        "[COORDINATOR] add_block failed: %s", resp->message.c_str());
                }
                sequence_active_ = false;
            });
    }

    void begin_wait_for_block_ready()
    {
        waiting_for_block_ready_ = true;
        block_ready_request_in_flight_ = false;
        block_ready_attempts_left_ = std::max(1, static_cast<int>(std::ceil(block_ready_timeout_ / 0.5)));

        RCLCPP_INFO(this->get_logger(),
            "[COORDINATOR] Waiting for accumulator to publish completed block before calling /map_merger/add_block...");

        block_ready_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&ScanningCoordinatorNode::poll_block_ready, this));
    }

    void poll_block_ready()
    {
        if (!sequence_active_ || !waiting_for_block_ready_) {
            if (block_ready_timer_) block_ready_timer_->cancel();
            return;
        }

        if (block_ready_request_in_flight_) {
            return;
        }

        if (block_ready_attempts_left_ <= 0) {
            waiting_for_block_ready_ = false;
            sequence_active_ = false;
            if (block_ready_timer_) block_ready_timer_->cancel();
            RCLCPP_ERROR(this->get_logger(),
                "[COORDINATOR] Timed out waiting for /scanner/merged_cloud readiness.");
            return;
        }
        block_ready_attempts_left_--;

        if (!block_ready_client_->wait_for_service(200ms)) {
            return;
        }

        block_ready_request_in_flight_ = true;
        block_ready_client_->async_send_request(
            std::make_shared<Trigger::Request>(),
            [this](rclcpp::Client<Trigger>::SharedFuture future) {
                block_ready_request_in_flight_ = false;

                if (!sequence_active_ || !waiting_for_block_ready_) {
                    return;
                }

                auto resp = future.get();
                if (resp->success) {
                    RCLCPP_INFO(this->get_logger(),
                        "[COORDINATOR] %s", resp->message.c_str());
                    add_block();
                }
            });
    }

    // ============ CONFIG ============
    int    num_scans_;
    double delay_before_first_scan_;
    double delay_between_scans_;
    double block_ready_timeout_;
    double arm_ack_timeout_;
    // ================================

    rclcpp::Client<Trigger>::SharedPtr scan_client_;
    rclcpp::Client<Trigger>::SharedPtr add_block_client_;
    rclcpp::Client<Trigger>::SharedPtr block_ready_client_;
    rclcpp::Client<Trigger>::SharedPtr script_runner_ready_client_;
    rclcpp::Client<RunScript>::SharedPtr arm_script_client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_started_sub_;
    rclcpp::TimerBase::SharedPtr delay_timer_;
    rclcpp::TimerBase::SharedPtr arm_ack_timer_;
    rclcpp::TimerBase::SharedPtr block_ready_timer_;

    int  current_scan_;
    int  block_count_;
    bool sequence_active_;
    bool waiting_for_arm_ack_;
    bool waiting_for_block_ready_;
    bool block_ready_request_in_flight_;
    int  block_ready_attempts_left_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanningCoordinatorNode>());
    rclcpp::shutdown();
    return 0;
}
