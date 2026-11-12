# ArUco Treasure Hunt

A ROS 2 project simulating a robot tasked with finding and identifying ArUco markers within a Gazebo environment.

---

## Project Structure

```bash
src/assignment1_erl/
├── config
├── launch
├── log
├── models
├── rviz
├── src
├── urdf
└── worlds

src/assignment_msg/
└── msg
```

## Prerequisites & Dependencies

Ensure you have the following installed on your system:

    - ROS 2 
    - Gazebo Simulator (ros_gz / Gazebo Classic depending on your setup)
    - OpenCV & cv_bridge (for ArUco marker detection)

## Launch the Simulation

To launch the full Gazebo world along with the robot, run:

    ```bash
    ./script/menu.sh
    ```

