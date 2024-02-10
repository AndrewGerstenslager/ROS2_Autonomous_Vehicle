#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// This program publishes the angle of the caster joint and the front right and left wheel joints
// This is important because the caster joint and the front right and left wheel joints are not actuated
// by something like the differential drive plugin in Gazebo. They are not actuated by the robot's controller
// and therefore, their angles need to be published to the robot's controller for RViz to not
// display the robot in a weird way. If not run, these joints are not in the correct position and rather
// at the center of the base_link frame as well as displaying errors in RViz.

// The joints will be frozen in Rviz even if these joints are moving in Gazebo or in the real robot.
// Since we cant measure the angle of the caster joint and the front right and left wheel joints in real life,
// we will publish a constant angle for these joints for both real and simulated scenarios.

int main(int argc, char** argv) {
    // Initialize the node
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("caster_joint_publisher");

    // Create a publisher
    auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Create a JointState message
    sensor_msgs::msg::JointState joint_state;
    joint_state.name.push_back("CasterJoint");
    joint_state.position.push_back(0.0);  // replace 0.0 with the desired angle

    // Add the front right and left wheel joints
    joint_state.name.push_back("FrontRightWheelJoint");
    joint_state.position.push_back(0.0);  // replace 0.0 with the desired angle for the front right wheel joint

    joint_state.name.push_back("FrontLeftWheelJoint");
    joint_state.position.push_back(0.0);  // replace 0.0 with the desired angle for the front left wheel joint

    // Set the rate at which to publish the angle
    rclcpp::Rate rate(10);  // 10 Hz

    while (rclcpp::ok()) {
        // Update the timestamp
        joint_state.header.stamp = node->now();

        // Publish the JointState message
        pub->publish(joint_state);

        // Sleep for a while before publishing again
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}