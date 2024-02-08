# Dokalman Library

The Dokalman library is the main package for the Dokalman robot. It contains the necessary nodes, launch files, and scripts for running and controlling the robot.

## Dependencies

- ROS2 (Robot Operating System 2)
- Novatel GPS library
- Arduino library
- OpenCV
- tf2_ros
- geometry_msgs

## How it Works

The Dokalman library is designed to control the Dokalman robot. It communicates with the robot's hardware, processes sensor data, and controls the robot's movements.

The library includes several nodes for different tasks:

- `state_publisher`: Publishes the state of the robot.
- `rviz2`: Provides a 3D visualization of the robot and its environment.
- `gazebo_ros`: Simulates the robot and its environment.

The library also includes launch files for starting these nodes and setting up the robot's environment.

## Nodes

- `state_publisher`: Publishes the state of the robot.
- `rviz2`: Provides a 3D visualization of the robot and its environment.
- `gazebo_ros`: Simulates the robot and its environment.

## Launch Files

- `start.launch.py`: Starts the robot and all necessary nodes.
- `rviz2.launch.py`: Starts RViz for 3D visualization.
- `gazebo_ros.launch.py`: Starts Gazebo for simulation.

## Running the Code

To run the code in this package, follow these steps:

1. Build the package using `colcon build`.
2. Source the setup file using `source install/setup.bash`.
3. Run the desired launch file using `ros2 launch dokalman <launch_file>.launch.py`.

For example, to start the robot and all necessary nodes, use:

```bash
ros2 launch dokalman_library gazebo.launch.py
ros2 launch dokalman_library rviz.launch.py