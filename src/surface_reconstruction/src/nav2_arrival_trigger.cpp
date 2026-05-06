#include <memory>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_msgs/msg/goal_status_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

class Nav2ArrivalTrigger : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalStatus = action_msgs::msg::GoalStatus;
  using GoalStatusArray = action_msgs::msg::GoalStatusArray;

  explicit Nav2ArrivalTrigger(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("nav2_arrival_trigger", options)
  {
    trigger_publish_count_ = this->declare_parameter<int>("trigger_publish_count", 5);
    trigger_publish_period_ms_ = this->declare_parameter<int>("trigger_publish_period_ms", 250);
    min_start_repair_subscribers_ = this->declare_parameter<int>("min_start_repair_subscribers", 2);
    subscriber_wait_timeout_ms_ = this->declare_parameter<int>("subscriber_wait_timeout_ms", 5000);
    start_repair_cooldown_ms_ = this->declare_parameter<int>("start_repair_cooldown_ms", 3000);

    start_repair_pub_ = this->create_publisher<std_msgs::msg::Bool>("/start_repair", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    node_start_time_ = this->now();

    // RViz Nav2 Goal Tool publishes geometry_msgs/PoseStamped on /goal_pose when using Nav2.
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(10),
      std::bind(&Nav2ArrivalTrigger::on_goal_pose, this, std::placeholders::_1));

    nav2_status_sub_ = this->create_subscription<GoalStatusArray>(
      "/navigate_to_pose/_action/status", rclcpp::QoS(10),
      std::bind(&Nav2ArrivalTrigger::on_nav2_status, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    RCLCPP_INFO(get_logger(), "nav2_arrival_trigger started. Waiting for /goal_pose and Nav2 action server /navigate_to_pose...");
  }

private:
  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Avoid sending repeated goals if RViz republishes the same goal.
    const auto key = goal_key(*msg);
    if (last_goal_key_ == key) {
      return;
    }
    last_goal_key_ = key;

    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "Nav2 action server /navigate_to_pose not available yet. Skipping this goal_pose.");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose = *msg;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&Nav2ArrivalTrigger::on_goal_response, this, std::placeholders::_1);
    options.result_callback = std::bind(&Nav2ArrivalTrigger::on_result, this, std::placeholders::_1);

    RCLCPP_INFO(get_logger(), "Forwarding RViz /goal_pose to /navigate_to_pose action...");
    (void)action_client_->async_send_goal(goal, options);
  }

  void on_goal_response(const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_WARN(get_logger(), "Nav2 rejected the forwarded /goal_pose.");
    }
  }

  void on_result(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      schedule_start_repair_burst("Nav2 goal result SUCCEEDED");
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_WARN(get_logger(), "Nav2 goal ABORTED. Not publishing /start_repair.");
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_WARN(get_logger(), "Nav2 goal CANCELED. Not publishing /start_repair.");
    } else {
      RCLCPP_WARN(get_logger(), "Nav2 goal finished with unknown result code. Not publishing /start_repair.");
    }
  }

  void on_nav2_status(const GoalStatusArray::SharedPtr msg)
  {
    for (const auto & status : msg->status_list) {
      if (status.status != GoalStatus::STATUS_SUCCEEDED) {
        continue;
      }

      const auto goal_stamp = rclcpp::Time(status.goal_info.stamp);
      if (goal_stamp.nanoseconds() != 0 && goal_stamp < node_start_time_) {
        continue;
      }

      const auto goal_id = uuid_to_string(status.goal_info.goal_id);
      if (seen_succeeded_goal_ids_.insert(goal_id).second) {
        schedule_start_repair_burst("Nav2 action status SUCCEEDED");
      }
    }
  }

  std::string goal_key(const geometry_msgs::msg::PoseStamped & msg) const
  {
    std::ostringstream key;
    key << msg.header.stamp.sec << ":" << msg.header.stamp.nanosec << ":"
        << msg.header.frame_id << ":"
        << msg.pose.position.x << ":" << msg.pose.position.y << ":" << msg.pose.position.z << ":"
        << msg.pose.orientation.x << ":" << msg.pose.orientation.y << ":"
        << msg.pose.orientation.z << ":" << msg.pose.orientation.w;
    return key.str();
  }

  std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & uuid) const
  {
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (const auto byte : uuid.uuid) {
      out << std::setw(2) << static_cast<int>(byte);
    }
    return out.str();
  }

  void schedule_start_repair_burst(const std::string & reason)
  {
    if (start_repair_burst_active_) {
      return;
    }

    const auto now = this->now();
    if (have_last_start_repair_time_ &&
      (now - last_start_repair_time_).nanoseconds() <
      static_cast<int64_t>(start_repair_cooldown_ms_) * 1000000)
    {
      return;
    }

    have_last_start_repair_time_ = true;
    last_start_repair_time_ = now;
    start_repair_burst_active_ = true;
    start_repair_publish_attempts_ = 0;
    start_repair_wait_start_ = now;

    RCLCPP_INFO(get_logger(), "%s. Publishing /start_repair=true burst.", reason.c_str());

    start_repair_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(trigger_publish_period_ms_),
      std::bind(&Nav2ArrivalTrigger::publish_start_repair_tick, this));
    publish_start_repair_tick();
  }

  void publish_start_repair_tick()
  {
    const size_t subscriber_count = start_repair_pub_->get_subscription_count();
    const bool has_required_subscribers =
      subscriber_count >= static_cast<size_t>(std::max(0, min_start_repair_subscribers_));
    const bool wait_timed_out =
      (this->now() - start_repair_wait_start_).nanoseconds() >=
      static_cast<int64_t>(subscriber_wait_timeout_ms_) * 1000000;

    if (!has_required_subscribers && !wait_timed_out) {
      return;
    }

    if (!has_required_subscribers && start_repair_publish_attempts_ == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Publishing /start_repair with %zu subscriber(s), expected at least %d.",
        subscriber_count,
        min_start_repair_subscribers_);
    }

    std_msgs::msg::Bool msg;
    msg.data = true;
    start_repair_pub_->publish(msg);
    start_repair_publish_attempts_++;

    if (start_repair_publish_attempts_ >= std::max(1, trigger_publish_count_)) {
      if (start_repair_timer_) {
        start_repair_timer_->cancel();
      }
      start_repair_burst_active_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Published /start_repair=true %d time(s).",
        start_repair_publish_attempts_);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_repair_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr nav2_status_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr start_repair_timer_;

  std::string last_goal_key_;
  std::unordered_set<std::string> seen_succeeded_goal_ids_;
  rclcpp::Time node_start_time_;
  rclcpp::Time start_repair_wait_start_;
  rclcpp::Time last_start_repair_time_;
  int trigger_publish_count_;
  int trigger_publish_period_ms_;
  int min_start_repair_subscribers_;
  int subscriber_wait_timeout_ms_;
  int start_repair_cooldown_ms_;
  int start_repair_publish_attempts_{0};
  bool start_repair_burst_active_{false};
  bool have_last_start_repair_time_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2ArrivalTrigger>());
  rclcpp::shutdown();
  return 0;
}
