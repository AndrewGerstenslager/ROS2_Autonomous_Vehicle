from launch import LaunchDescription
from launch_ros.actions import Node

reg_linear = 0.5
reg_angular = 1.0
turbo_multiplier = 2.0


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[
                    {"dev": "/dev/input/js0", "deadzone": 0.1, "autorepeat_rate": 20.0}
                ],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                parameters=[{
                            "enable_button": -1,
                            "require_enable_button": False,
                            "axis_linear.x": 4,
                            "axis_angular.yaw": 3,
                            "scale_linear.x": reg_linear,
                            "scale_linear.y": reg_linear,
                            "scale_linear.z": reg_linear,
                            "scale_angular.pitch": reg_angular,
                            "scale_angular.roll": reg_angular,
                            "scale_angular.yaw": reg_angular,
                            "enable_turbo_button": 5,
                            "scale_linear_turbo.x": reg_linear * turbo_multiplier,
                            "scale_linear_turbo.y": reg_linear * turbo_multiplier,
                            "scale_linear_turbo.z": reg_linear * turbo_multiplier,
                            "scale_angular_turbo.pitch": reg_angular * turbo_multiplier,
                            "scale_angular_turbo.roll": reg_angular * turbo_multiplier,
                            "scale_angular_turbo.yaw": reg_angular * turbo_multiplier,
                            }],
            ),
        ]
    )