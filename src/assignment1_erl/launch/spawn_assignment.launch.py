
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_assignment1_erl = get_package_share_directory('assignment1_erl')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_assignment1_erl)

    # Gazebo resource path
    gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] = gz_path + os.pathsep + gazebo_models_path

    # Launch arguments
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz.'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='assignment.sdf',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='my_robot.urdf',
        description='Name of the URDF description to load'
    )

    # Robot description
    urdf_file_path = PathJoinSubstitution(
        [
            pkg_assignment1_erl,
            "urdf",
            LaunchConfiguration('model'),
        ]
    )

    # Gazebo world
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_assignment1_erl,
                'launch',
                'world.launch.py'
            ),
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg_assignment1_erl, 'rviz', 'urdf.rviz')
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ],
    )

    # Spawn robot in Gazebo
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5",
            "-Y", "0.0",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ],
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': Command(['xacro', ' ', urdf_file_path]),
                'use_sim_time': True
            },
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
    )

    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[
            {'use_sim_time': True},
        ],
    )

    # Gazebo image bridge
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {
                'use_sim_time': True,
                'camera.image.compressed.jpeg_quality': 75
            },
        ],
    )

    # EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_assignment1_erl, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ],
    )

    # Gazebo ROS bridge
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ],
    )

    # ArUco tracker
    aruco_node = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        output='screen',
        parameters=[
            {
                'cam_base_topic': 'camera/image',
                'marker_dict': 'ARUCO_ORIGINAL',
                'marker_size': 0.2,
                'image_is_rectified': False,
                'image_sub_compressed': False,
                'image_sub_qos.reliability': 2,
                'image_sub_qos.durability': 2,
                'image_sub_qos.depth': 1,
                'publish_tf': True,
                'output_frame': '',
                'use_sim_time': True,
            }
        ],
    )

    # =====================================================
    # STANDALONE NODES
    # =====================================================

    # FSM node
    fsm_node = Node(
        package='assignment1_erl',
        executable='fsm_node',
        name='fsm_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ],
    )

    # Image detection node
    image_detection_node = Node(
        package='assignment1_erl',
        executable='image_detection_node',
        name='detection_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'marker_size_m': 0.2,
                'desired_size_sim_px': 166.0,
                'simulation_focal_length_px': 381.4,
            },
        ],
    )

    # Navigation node
    navigation_node = Node(
        package='assignment1_erl',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ],
    )

    # =====================================================
    # LAUNCH DESCRIPTION
    # =====================================================

    launch_description = LaunchDescription()

    launch_description.add_action(rviz_launch_arg)
    launch_description.add_action(world_arg)
    launch_description.add_action(model_arg)

    launch_description.add_action(world_launch)

    launch_description.add_action(rviz_node)
    launch_description.add_action(spawn_urdf_node)
    launch_description.add_action(robot_state_publisher_node)

    # launch_description.add_action(joint_state_publisher_gui_node)

    launch_description.add_action(gz_bridge_node)
    launch_description.add_action(gz_image_bridge_node)

    # launch_description.add_action(ekf_node)
    launch_description.add_action(aruco_node)

    # Standalone nodes
    launch_description.add_action(fsm_node)
    launch_description.add_action(image_detection_node)
    launch_description.add_action(navigation_node)

    return launch_description
