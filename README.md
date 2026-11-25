# ROSbot Autonomous Navigation – ROS 2

This project implements an autonomous navigation system for a ROSbot using ROS 2 Jazzy. The robot detects ArUco markers through its camera and autonomously approaches them by combining visual information, odometry and a Finite State Machine (FSM).

The system can be tested both in a Gazebo simulation and on the real robot.

## Videos

### Simulation
https://github.com/user-attachments/assets/908c32d3-581b-4d59-819a-f13303b7dfec

### Real Robot

https://github.com/user-attachments/assets/1bec609b-c7fa-463e-bacb-87fa25771580




## Repository Structure

The repository is organized as follows:

```text
ros_ws/
├── marker_images/              # Images of the detected markers
└── src/
    ├── assignment1_erl/        # Main ROS 2 package
    │   ├── launch/
    │   └── src/
    │       ├── image_detection.cpp
    │       ├── fsm.cpp
    │       ├── navigation.cpp
    │       └── image_annotator.cpp
    │
    └── assignment_msg/         # Custom ROS 2 messages
```

### Main Nodes

The `assignment1_erl` package contains four main nodes:

- **`image_detection.cpp`**: Processes ArUco marker detections and publishes marker information in image coordinates through `/aruco_info`.
- **`fsm.cpp`**: Implements the Finite State Machine, managing the robot's states and the sequence of target markers.
- **`navigation.cpp`**: Computes and publishes velocity commands through `/cmd_vel`, using visual feedback and odometry.
- **`image_annotator.cpp`**: Annotates camera images when a marker is reached, saves the resulting images and publishes them through `/marker_image`.

The `assignment_msg` package contains the custom message definitions used for communication between nodes.

## Dependencies

The project requires:

- ROS 2 Jazzy
- Gazebo
- RViz
- `ros_aruco_opencv`
- OpenCV
- `cv_bridge`

Additional ROS 2 dependencies are declared in the package files.

## Build Instructions

Clone the repository inside the `src` folder of your ROS 2 workspace:

```bash
cd ~/your_ros2_ws/src
git clone <repository-url>
```

Install the dependencies and build the workspace:

```bash
cd ~/your_ros2_ws

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

## Launch Instructions

### Gazebo Simulation

To launch the Gazebo simulation:

```bash
ros2 launch assignment1_erl spawn_assignment.launch.py
```

### Real Robot

To launch the system on the real robot:

```bash
ros2 launch assignment1_erl spawn_assignment_robot.launch.py
```

**Note:** When using the real robot, check the launch file parameters and configure them according to the robot setup before running the system.

