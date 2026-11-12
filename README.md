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
example, the steps below use robot 2.

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
3. Connect the computer running this launch to the same Wi-Fi network as the
   robot.
4. Set `ROS_DOMAIN_ID` to the domain ID configured for that robot, replacing
   `<robot_id>` with its numeric ID:

   ```bash
   export ROS_DOMAIN_ID=<robot_id>
   ```

5. Launch the nodes:

   ```bash
   ros2 launch assignment1_erl spawn_assignment_robot.launch.py robot_number:=2 rviz:=false
   ```

6. Confirm that the sensor topics are publishing with:

   ```bash
   ros2 topic list | grep rosbot_2
   ```

Test in a clear area with the robot's emergency stop accessible; the
controller may send movement commands while running.



