#!/usr/bin/env bash

echo ""
echo "PROJECT MENU:"
echo "--------------------------------"
echo "1. Compile and launch simulation"
echo "2. Compile and launch rosbot"
echo "3. Check the active topics list"
echo "4. Display an active topic"
echo "--------------------------------"

read -p "Enter your choice (1-4): " choice

case $choice in

    1)
        echo "Building workspace and launching Gazebo..."

        # rm -rf build/ log/ install/

        rm src/assignment1_erl/marker_images_robot/*

        source /opt/ros/lyrical/setup.bash
        colcon build
        source install/local_setup.bash

        cd src/ 
        ros2 launch assignment1_erl spawn_assignment_robot.launch.py sim:=true
        ;;

    2)

        cd ~

        

        echo "Building workspace and launching Gazebo..."

        # rm -rf build/ log/ install/

        rm src/assignment1_erl/marker_images_robot/*

        source /opt/ros/lyrical/setup.bash
        colcon build
        source install/local_setup.bash

        cd src/ 
        ros2 launch assignment1_erl spawn_assignment_robot.launch.py sim:=false robot_number:=3
        ;;

    3)

        source /opt/ros/lyrical/setup.bash
        source install/local_setup.bash

        echo "--------------------------------"
        echo "Looking for the active topics..."
        echo "--------------------------------"
        echo ""

        ros2 topic list
        ;;
        
    4)
        
        source /opt/ros/lyrical/setup.bash
        source install/local_setup.bash

        echo "--------------------------------"
        echo "Enter the name of the topic you want to display from the list:"
        echo ""
        ros2 topic list
        echo "--------------------------------"

        read -p "Enter your choice: " topic_name
        
        echo ""
        echo "Starting to listen to the topic: $topic_name"
        echo "Press Ctrl+C to stop viewing and return to the menu (if handled)."
        echo "--------------------------------"
        
        # Executes the ros2 command to print the topic output
        ros2 topic echo "$topic_name"
        ;;
        
    *)
        echo "Invalid option."
        read -p "Press Enter to exit..."
        exit 1
        ;;

esac