// scanning_coordinator.cpp
//
// Orchestrates: /start_repair → 3 arm scans → /map_merger/add_block (auto-merge)
// No position limit — trigger as many times as needed.
// Call /map_merger/save when done to write merged_map.pcd.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"

using Trigger = std_srvs::srv::Trigger;
using Empty   = std_srvs::srv::Empty;
using namespace std::chrono_literals;

class ScanningCoordinatorNode : public rclcpp::Node
{
public:
    ScanningCoordinatorNode()
        : Node("scanning_coordinator"),
          current_scan_(0),
          block_count_(0),
          sequence_active_(false),
          // ======== ADJUST THESE VARIABLES AS NEEDED ========
          num_scans_(3),                    // arm strips per block
          delay_before_first_scan_(2.4),    // seconds before first trigger
          delay_between_scans_(13.1),       // seconds between arm strips
          delay_before_add_block_(2.0)      // seconds after last scan before adding to world map
          // ===================================================
    {
        scan_client_      = this->create_client<Trigger>("/scanner/trigger_scan");
        add_block_client_ = this->create_client<Trigger>("/map_merger/add_block");

        sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start_repair", 10,
            std::bind(&ScanningCoordinatorNode::trigger_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "[COORDINATOR] Ready — publish /start_repair to begin a 3-scan block.\n"
            "  Repeat as many times as needed at any position.\n"
            "  Call /map_merger/save when finished to write merged_map.pcd");
    }

private:
    void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data || sequence_active_) return;

        block_count_++;
        RCLCPP_INFO(this->get_logger(),
            "[COORDINATOR] ▶ Block #%d — starting %d-scan sequence", block_count_, num_scans_);

        sequence_active_ = true;
        current_scan_ = 1;

        delay_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(delay_before_first_scan_ * 1000)),
            [this]() { delay_timer_->cancel(); trigger_next_scan(); });
    }

    void trigger_next_scan()
    {
        if (current_scan_ > num_scans_)
        {
            delay_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(delay_before_add_block_ * 1000)),
                [this]() { delay_timer_->cancel(); add_block(); });
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

    // ============ CONFIG ============
    int    num_scans_;
    double delay_before_first_scan_;
    double delay_between_scans_;
    double delay_before_add_block_;
    // ================================

    rclcpp::Client<Trigger>::SharedPtr scan_client_;
    rclcpp::Client<Trigger>::SharedPtr add_block_client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr delay_timer_;

    int  current_scan_;
    int  block_count_;
    bool sequence_active_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanningCoordinatorNode>());
    rclcpp::shutdown();
    return 0;
}
