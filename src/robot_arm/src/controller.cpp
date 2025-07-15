#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_controller");

  moveit::planning_interface::MoveGroupInterface move_group(node, "cr20_group");

  // Optional: Set reference frame
  move_group.setPoseReferenceFrame("base_link");

  // Define the target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.24353089928627014;
  target_pose.position.y = -0.16773217916488647;
  target_pose.position.z = 1.061023235321045;

  // Set orientation: facing forward (identity quaternion = no rotation)
  target_pose.orientation.x = 0.6992626190185547;
  target_pose.orientation.y = -0.6917200684547424;
  target_pose.orientation.z = 0.10503347963094711;
  target_pose.orientation.w = -0.1467074751853943;

  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Planning successful.");
    // move_group.execute(plan);
    
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "❌ Planning failed.");
  }

  rclcpp::shutdown();
  return 0;
}
