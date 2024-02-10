#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// The OdomHeadingNode class is a subclass of the rclcpp::Node class. 
// It has a constructor that initializes the node and sets up the subscriber and publisher.
// The odom_callback method is called whenever a new odometry message is received.
// The method extracts the quaternion from the odometry message, converts the quaternion to RPY (roll, pitch, yaw),
// and then converts the yaw to degrees. The heading is then published to the /heading topic.
// The main function initializes the node and spins it.
 
class OdomHeadingNode : public rclcpp::Node
{
public:
    OdomHeadingNode() : Node("odom_heading_node")
    {
        // Subscriber for the odometry
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&OdomHeadingNode::odom_callback, this, std::placeholders::_1));

        // Publisher for the heading
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/heading", 10);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract the quaternion from the odometry message
        auto orientation_q = msg->pose.pose.orientation;

        // Convert the quaternion to RPY (roll, pitch, yaw)
        double roll, pitch, yaw;
        tf2::Quaternion q(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        // Convert yaw to degrees
        double yaw_degrees = yaw * (180.0 / M_PI);
        double heading = std::fmod((yaw_degrees + 360.0), 360.0); // Convert yaw to 0 - 360 degrees

        // Create a Float64 message and publish the heading
        auto heading_msg = std_msgs::msg::Float64();
        heading_msg.data = heading;
        publisher_->publish(heading_msg);

        // Uncomment to log the heading
        // RCLCPP_INFO(this->get_logger(), "Published Heading: %.2f degrees", heading);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomHeadingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
