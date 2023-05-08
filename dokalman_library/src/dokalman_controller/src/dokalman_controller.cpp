#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "dokalman_controller/msg/wheel_velocities.hpp"

class DokalmanController : public rclcpp::Node
{
public:
  DokalmanController()
  : Node("dokalman_controller")
  {
    this->declare_parameter("base_width", 0.31);
    this->declare_parameter("wheel_radius", 0.155);

    this->get_parameter("base_width", base_width_);
    this->get_parameter("wheel_radius", wheel_radius_);
    wheel_vel_pub_ = this->create_publisher<dokalman_controller::msg::WheelVelocities>("wheel_velocities", 10);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&DokalmanController::cmd_vel_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    odom_trans_.header.frame_id = "odom";
    odom_trans_.child_frame_id = "base_footprint";

    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_footprint";
    
    // Initialize the TransformBroadcaster with the node pointer
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double linear_vel_x = msg->linear.x;
    double angular_vel_z = msg->angular.z;

    double wheel_left_vel = (2 * linear_vel_x - angular_vel_z * base_width_) / (2 * wheel_radius_);
    double wheel_right_vel = (2 * linear_vel_x + angular_vel_z * base_width_) / (2 * wheel_radius_);

    // Create a WheelVelocities message and set the left and right velocities
    auto wheel_vel_msg = dokalman_controller::msg::WheelVelocities();
    wheel_vel_msg.left = wheel_left_vel;
    wheel_vel_msg.right = wheel_right_vel;

    // Publish the wheel velocities
    wheel_vel_pub_->publish(wheel_vel_msg);

    // Update the odometry and publish the transform and odometry message
    update_odometry(linear_vel_x, angular_vel_z);
  }

  void update_odometry(double linear_vel_x, double angular_vel_z)
  {
    double dt = 1.0 / 30.0; // Assuming 30 Hz update rate
    double delta_x = linear_vel_x * cos(th_) * dt;
    double delta_y = linear_vel_x * sin(th_) * dt;
    double delta_th = angular_vel_z * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    // Since all odometry is 6DOF we'll need a quaternion created from yaw
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, th_);

    // First, we'll publish the transform over tf
    odom_trans_.header.stamp = this->now();
    odom_trans_.transform.translation.x= x_;
    odom_trans_.transform.translation.y = y_;
    odom_trans_.transform.translation.z = 0.0;
    odom_trans_.transform.rotation.x = odom_quat.x();
    odom_trans_.transform.rotation.y = odom_quat.y();
    odom_trans_.transform.rotation.z = odom_quat.z();
    odom_trans_.transform.rotation.w = odom_quat.w();

    // Send the transform
    tf_broadcaster_->sendTransform(odom_trans_);

    // Next, we'll publish the odometry message over ROS
    odom_.header.stamp = this->now();
    odom_.pose.pose.position.x = x_;
    odom_.pose.pose.position.y = y_;
    odom_.pose.pose.position.z = 0.0;
    odom_.pose.pose.orientation.x = odom_quat.x();
    odom_.pose.pose.orientation.y = odom_quat.y();
    odom_.pose.pose.orientation.z = odom_quat.z();
    odom_.pose.pose.orientation.w = odom_quat.w();

    // Set the velocity
    odom_.twist.twist.linear.x = linear_vel_x;
    odom_.twist.twist.linear.y = 0;
    odom_.twist.twist.angular.z = angular_vel_z;

    // Publish the message
    odom_pub_->publish(odom_);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<dokalman_controller::msg::WheelVelocities>::SharedPtr wheel_vel_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped odom_trans_;
  nav_msgs::msg::Odometry odom_;

  double x_ = 0.0, y_ = 0.0, th_ = 0.0;
  double base_width_;
  double wheel_radius_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DokalmanController>());
  rclcpp::shutdown();
  return 0;
}
