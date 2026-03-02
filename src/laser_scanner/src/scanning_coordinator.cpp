
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"

using Trigger = std_srvs::srv::Trigger;
using Empty = std_srvs::srv::Empty;
using namespace std::chrono_literals;

class ScanningCoordinatorNode : public rclcpp::Node
{
public:
    ScanningCoordinatorNode()
        : Node("scanning_coordinator"), 
          current_scan_(0),
          state_(State::IDLE),
          sequence_active_(false),
          // ======== ADJUST THESE VARIABLES AS NEEDED ========
          num_scans_(3),                        // Number of scans to perform
          delay_before_first_scan_(2.4),        // Delay before first scan (seconds)
          delay_between_scans_(13.1),           // Delay between scans (seconds)
          delay_before_save_(2.0),              // Delay before saving (seconds)
          delay_before_clear_(2.0)              // Delay before clearing (seconds)
          // ===================================================
    {
        
        // Create service clients
        scan_client_ = this->create_client<Trigger>("/scanner/trigger_scan");
        save_client_ = this->create_client<Trigger>("/scanner/save_merged_cloud");
        clear_client_ = this->create_client<Empty>("/scanner/clear_scans");
        
        // Subscribe to trigger topic
        sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start_repair",
            10,
            std::bind(&ScanningCoordinatorNode::trigger_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "[COORDINATOR] Ready - Waiting for /start_repair trigger (3 scans)");
    }

private:
    enum class State {
        IDLE,
        SCANNING,
        SAVING,
        CLEARING,
        DONE
    };

    void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // Only trigger on TRUE message and when not already active
        if (msg->data && !sequence_active_)
        {
            RCLCPP_INFO(this->get_logger(), "[COORDINATOR] ▶ Starting %d-scan sequence", num_scans_);
            
            sequence_active_ = true;
            current_scan_ = 0;
            start_scan_sequence();
        }
    }

    void start_scan_sequence()
    {
        current_scan_ = 1;
        
        // Wait before first scan
        delay_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(delay_before_first_scan_ * 1000)),
            [this]() {
                delay_timer_->cancel();
                trigger_next_scan();
            });
    }

    void trigger_next_scan()
    {
        if (current_scan_ > num_scans_)
        {
            // All scans complete, wait then save
            delay_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(delay_before_save_ * 1000)),
                [this]() {
                    delay_timer_->cancel();
                    finish_scanning_and_save();
                });
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[COORDINATOR] Triggering scan %d/%d...", current_scan_, num_scans_);
        
        // Wait before next scan (robot movement handled externally)
        if (current_scan_ > 1)
        {
            delay_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(delay_between_scans_ * 1000)),
                [this]() {
                    delay_timer_->cancel();
                    trigger_scan();
                });
        }
        else
        {
            trigger_scan();
        }
    }

    void trigger_scan()
    {
        state_ = State::SCANNING;
        
        if (!scan_client_->wait_for_service(2s))
        {
            RCLCPP_ERROR(this->get_logger(), "Scanner service unavailable!");
            state_ = State::IDLE;
            sequence_active_ = false;
            return;
        }

        auto scan_request = std::make_shared<Trigger::Request>();
        scan_client_->async_send_request(scan_request,
            std::bind(&ScanningCoordinatorNode::scan_response_callback, this, std::placeholders::_1));
    }

    void scan_response_callback(rclcpp::Client<Trigger>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "[COORDINATOR] ✓ Scan %d/%d complete", current_scan_, num_scans_);
            current_scan_++;
            trigger_next_scan();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[COORDINATOR] ✗ Scan %d FAILED: %s", current_scan_, response->message.c_str());
            state_ = State::IDLE;
            sequence_active_ = false;
        }
    }

    void finish_scanning_and_save()
    {
        RCLCPP_INFO(this->get_logger(), "[COORDINATOR] Saving merged cloud...");
        
        state_ = State::SAVING;
        
        if (!save_client_->wait_for_service(2s))
        {
            RCLCPP_ERROR(this->get_logger(), "Save service unavailable!");
            state_ = State::IDLE;
            sequence_active_ = false;
            return;
        }

        auto save_request = std::make_shared<Trigger::Request>();
        save_client_->async_send_request(save_request,
            std::bind(&ScanningCoordinatorNode::save_response_callback, this, std::placeholders::_1));
    }

    void save_response_callback(rclcpp::Client<Trigger>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "[COORDINATOR] ✓ %s", response->message.c_str());
            
            // Wait before clearing
            delay_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(delay_before_clear_ * 1000)),
                [this]() {
                    delay_timer_->cancel();
                    clear_scans();
                });
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Save failed: %s", response->message.c_str());
            state_ = State::IDLE;
            sequence_active_ = false;
        }
    }

    void clear_scans()
    {
        state_ = State::CLEARING;
        
        if (!clear_client_->wait_for_service(2s))
        {
            RCLCPP_ERROR(this->get_logger(), "Clear service unavailable!");
            state_ = State::IDLE;
            sequence_active_ = false;
            return;
        }

        auto clear_request = std::make_shared<Empty::Request>();
        clear_client_->async_send_request(clear_request,
            std::bind(&ScanningCoordinatorNode::clear_response_callback, this, std::placeholders::_1));
    }

    void clear_response_callback(rclcpp::Client<Empty>::SharedFuture future)
    {
        state_ = State::DONE;
        RCLCPP_INFO(this->get_logger(), "[COORDINATOR] ■ Sequence complete - Ready for next trigger");
        
        state_ = State::IDLE;
        sequence_active_ = false;
    }

    // ============ CONFIGURATION VARIABLES (EDIT THESE) ============
    int num_scans_;
    double delay_before_first_scan_;
    double delay_between_scans_;
    double delay_before_save_;
    double delay_before_clear_;
    // ===============================================================
    
    // Service clients
    rclcpp::Client<Trigger>::SharedPtr scan_client_;
    rclcpp::Client<Trigger>::SharedPtr save_client_;
    rclcpp::Client<Empty>::SharedPtr clear_client_;
    
    // Topic subscription
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    
    // Delay timer
    rclcpp::TimerBase::SharedPtr delay_timer_;
    
    // State tracking
    int current_scan_;
    State state_;
    bool sequence_active_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanningCoordinatorNode>());
    rclcpp::shutdown();
    return 0;
}
