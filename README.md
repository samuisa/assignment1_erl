# ArUco Treasure Hunt

A ROS 2 project simulating a robot tasked with finding and identifying ArUco markers within a Gazebo environment.

---

## Project Structure

```bash
.
├── config/        # Configuration files
├── launch/        # ROS 2 launch files
├── models/        # 3D models for Gazeb
├── msg/           # Custom ROS 2 message definitions
├── rviz/          # RViz visualization layouts
├── src/           # C++ / Python source code for nodes
├── urdf/          # Robot URDF/xacro descriptions
└── worlds/        # Gazebo world files
```

## Prerequisites & Dependencies

Ensure you have the following installed on your system:

    - ROS 2 (Humble / Iron / Jazzy)
    - Gazebo Simulator (ros_gz / Gazebo Classic depending on your setup)
    - OpenCV & cv_bridge (for ArUco marker detection)

## Launch the Simulation

To launch the full Gazebo world along with the robot, run:

```bash
ros2 launch gazebo_ath spawn_assignment.launch.py
```