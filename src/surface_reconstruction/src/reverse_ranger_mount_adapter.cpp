#include <memory>
#include <cmath>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ReverseRangerMountAdapter : public rclcpp::Node
{
public:
    ReverseRangerMountAdapter()
        : Node("reverse_ranger_mount_adapter")
    {
        input_cmd_topic_ = this->declare_parameter<std::string>("input_cmd_topic", "/cmd_vel");
        output_cmd_topic_ = this->declare_parameter<std::string>("output_cmd_topic", "/ranger/cmd_vel");
        input_odom_topic_ = this->declare_parameter<std::string>("input_odom_topic", "/ranger/odom");
        output_odom_topic_ = this->declare_parameter<std::string>("output_odom_topic", "/odom");
        odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        ranger_base_frame_ = this->declare_parameter<std::string>("ranger_base_frame", "ranger_base_link");
        nav_base_frame_ = this->declare_parameter<std::string>("nav_base_frame", "base_link");
        republish_odom_ = this->declare_parameter<bool>("republish_odom", false);
        flip_odom_ = this->declare_parameter<bool>("flip_odom", false);
        publish_tf_ = this->declare_parameter<bool>("publish_tf", false);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic_, 10);
        if (republish_odom_) {
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
            if (publish_tf_) {
                tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            }
        }

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            input_cmd_topic_, 10,
            std::bind(&ReverseRangerMountAdapter::cmdCallback, this, std::placeholders::_1));

        if (republish_odom_) {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                input_odom_topic_, 20,
                std::bind(&ReverseRangerMountAdapter::odomCallback, this, std::placeholders::_1));
        }

        RCLCPP_INFO(this->get_logger(),
            "[REVERSE_MOUNT] Command adapter active: %s -> %s. "
            "republish_odom=%s flip_odom=%s publish_tf=%s (%s -> %s)",
            input_cmd_topic_.c_str(), output_cmd_topic_.c_str(),
            republish_odom_ ? "true" : "false",
            flip_odom_ ? "true" : "false",
            publish_tf_ ? "true" : "false",
            input_odom_topic_.c_str(), output_odom_topic_.c_str());
    }

private:
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::Twist out = *msg;

        // A 180-degree yaw between Nav2 base_link and Ranger hardware frame
        // flips translational X/Y. Yaw rate is unchanged.
        out.linear.x = -msg->linear.x;
        out.linear.y = -msg->linear.y;
        out.angular.z = msg->angular.z;

        cmd_pub_->publish(out);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        nav_msgs::msg::Odometry out = *msg;
        out.header.frame_id = odom_frame_;
        out.child_frame_id = flip_odom_ ? nav_base_frame_ : msg->child_frame_id;

        if (flip_odom_) {
            tf2::Quaternion q_odom_ranger;
            tf2::fromMsg(msg->pose.pose.orientation, q_odom_ranger);

            tf2::Quaternion q_ranger_nav;
            q_ranger_nav.setRPY(0.0, 0.0, M_PI);

            tf2::Quaternion q_odom_nav = q_odom_ranger * q_ranger_nav;
            q_odom_nav.normalize();
            out.pose.pose.orientation = tf2::toMsg(q_odom_nav);

            out.twist.twist.linear.x = -msg->twist.twist.linear.x;
            out.twist.twist.linear.y = -msg->twist.twist.linear.y;
            out.twist.twist.angular.z = msg->twist.twist.angular.z;
        }

        odom_pub_->publish(out);

        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = out.header.stamp;
            tf_msg.header.frame_id = out.header.frame_id;
            tf_msg.child_frame_id = out.child_frame_id;
            tf_msg.transform.translation.x = out.pose.pose.position.x;
            tf_msg.transform.translation.y = out.pose.pose.position.y;
            tf_msg.transform.translation.z = out.pose.pose.position.z;
            tf_msg.transform.rotation = out.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    std::string input_cmd_topic_;
    std::string output_cmd_topic_;
    std::string input_odom_topic_;
    std::string output_odom_topic_;
    std::string odom_frame_;
    std::string ranger_base_frame_;
    std::string nav_base_frame_;
    bool republish_odom_;
    bool flip_odom_;
    bool publish_tf_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReverseRangerMountAdapter>());
    rclcpp::shutdown();
    return 0;
}
