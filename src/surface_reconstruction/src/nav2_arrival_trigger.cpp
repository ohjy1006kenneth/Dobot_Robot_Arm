#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/empty.hpp"

class Nav2ArrivalTrigger : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalStatus = action_msgs::msg::GoalStatus;

  explicit Nav2ArrivalTrigger(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("nav2_arrival_trigger", options)
  {
    start_repair_pub_ = this->create_publisher<std_msgs::msg::Empty>("/start_repair", 10);

    // RViz Nav2 Goal Tool publishes geometry_msgs/PoseStamped on /goal_pose when using Nav2.
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(10),
      std::bind(&Nav2ArrivalTrigger::on_goal_pose, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    RCLCPP_INFO(get_logger(), "nav2_arrival_trigger started. Waiting for /goal_pose and Nav2 action server /navigate_to_pose...");
  }

private:
  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Avoid sending repeated goals if RViz republishes the same goal.
    // Use header stamp + pose as a simple key.
    const auto key = std::to_string(msg->header.stamp.sec) + ":" + std::to_string(msg->header.stamp.nanosec);
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
    options.result_callback = std::bind(&Nav2ArrivalTrigger::on_result, this, std::placeholders::_1);

    RCLCPP_INFO(get_logger(), "Forwarding RViz /goal_pose to /navigate_to_pose action...");
    (void)action_client_->async_send_goal(goal, options);
  }

  void on_result(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Nav2 goal SUCCEEDED. Publishing /start_repair.");
      std_msgs::msg::Empty msg;
      start_repair_pub_->publish(msg);
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_WARN(get_logger(), "Nav2 goal ABORTED. Not publishing /start_repair.");
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_WARN(get_logger(), "Nav2 goal CANCELED. Not publishing /start_repair.");
    } else {
      RCLCPP_WARN(get_logger(), "Nav2 goal finished with unknown result code. Not publishing /start_repair.");
    }
  }

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_repair_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  std::string last_goal_key_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2ArrivalTrigger>());
  rclcpp::shutdown();
  return 0;
}
