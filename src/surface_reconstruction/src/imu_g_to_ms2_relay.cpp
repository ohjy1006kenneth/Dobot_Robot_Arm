/**
 * imu_g_to_ms2_relay.cpp
 *
 * The RoboSense RSAIRY SDK publishes:
 *   - linear_acceleration in g-force  (acclFsr/32768, no 9.81 applied)
 *   - angular_velocity    in rad/s    (gyroFsr/32768 * pi/180, already converted)
 *
 * FAST_LIO expects linear_acceleration in m/s².
 * This node multiplies linear_acceleration by 9.81 only.
 * angular_velocity is passed through unchanged.
 *
 * Subscribes:  /rslidar_imu_data       (sensor_msgs/Imu, accel: g,    gyro: rad/s)
 * Publishes:   /rslidar_imu_data_m2    (sensor_msgs/Imu, accel: m/s², gyro: rad/s)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

static constexpr double G = 9.81;

class ImuRelayNode : public rclcpp::Node
{
public:
  ImuRelayNode() : Node("imu_g_to_ms2_relay")
  {
    this->declare_parameter<std::string>("input_topic",  "/rslidar_imu_data");
    this->declare_parameter<std::string>("output_topic", "/rslidar_imu_data_m2");

    std::string in  = this->get_parameter("input_topic").as_string();
    std::string out = this->get_parameter("output_topic").as_string();

    // Use reliable QoS so FAST_LIO (reliable subscriber) can receive messages
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>(out, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      in, rclcpp::SensorDataQoS(),  // SDK publishes best-effort, relay accepts it
      [this](sensor_msgs::msg::Imu::UniquePtr msg) {
        // Acceleration: g-force -> m/s²
        msg->linear_acceleration.x *= G;
        msg->linear_acceleration.y *= G;
        msg->linear_acceleration.z *= G;

        // Angular velocity: SDK already outputs rad/s — do NOT convert again

        pub_->publish(std::move(msg));
      });

    RCLCPP_INFO(this->get_logger(),
      "IMU relay: '%s' (accel: g, gyro: rad/s) -> '%s' (accel: m/s², gyro: rad/s)",
      in.c_str(), out.c_str());
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  double last_log_time_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuRelayNode>());
  rclcpp::shutdown();
  return 0;
}
