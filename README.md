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

## Launch on a Real Robot

The launch file starts the assignment nodes without Gazebo. Choose a robot
number that matches the namespace configured for the robot's drivers. For
example, to use robot 2, run:

```bash
ros2 launch assignment1_erl spawn_assignment_robot.launch.py robot_number:=2 rviz:=false
```

The number only sets the ROS namespace; it does not select or configure the
physical robot. With `robot_number:=2`, this project uses topics under
`/rosbot_2/`. Before launching it:

1. Start the robot's base and camera drivers. This launch file does not start
   the drivers or Gazebo.
2. Configure the drivers to use the same namespace. The robot should publish
   `/rosbot_2/odom`, `/rosbot_2/imu`, `/rosbot_2/camera/image`,
   `/rosbot_2/camera/camera_info`, and `/rosbot_2/joint_states`. The base
   driver should receive velocity commands on `/rosbot_2/cmd_vel`. If the
   drivers use different topic names, configure their namespace or remappings
   to match.
3. Ensure the computer running this launch and the robot can communicate over
   ROS 2/DDS, then run the launch command above.
4. Confirm that the sensor topics are publishing with:

   ```bash
   ros2 topic list | grep rosbot_2
   ```

Test in a clear area with the robot's emergency stop accessible; the
controller may send movement commands while running.

