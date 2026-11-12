import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_assignment1_erl = get_package_share_directory('assignment1_erl')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_assignment1_erl)
    # lets Gazebo find package://assignment1_erl            /... (e.g. the meshes)
    gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] = gz_path + os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.')

    world_arg = DeclareLaunchArgument(
        'world', default_value='assignment.sdf', description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='my_robot.urdf', description='Name of the URDF description to load'
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution(
        [
            pkg_assignment1_erl,  # Replace with your package name
            "urdf",
            LaunchConfiguration('model'),  # Replace with your URDF or Xacro file
        ]
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_assignment1_erl, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_assignment1_erl, 'rviz', 'urdf.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ],
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "my_robot",
            "-topic",
            "/rosbot_1/robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.5",
            "-Y",
            "0.0",  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]), 'use_sim_time': True},
        ],
        remappings=[('/tf', '/rosbot_1/tf'), ('/tf_static', '/rosbot_1/tf_static')],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': True}],
    )

    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True, 'camera.image.compressed.jpeg_quality': 75},
        ],
    )

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

    aruco_node = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        output='screen',
        parameters=[
            {
                'cam_base_topic': '/rosbot_1/camera/rgb/image_raw',
                'marker_dict': 'ARUCO_ORIGINAL',
                'marker_size': 0.2,  # lato reale del marker in metri
                'image_is_rectified': False,
                'image_sub_compressed': False,
                'image_sub_qos.reliability': 2,  # best effort
                'image_sub_qos.durability': 2,  # volatile
                'image_sub_qos.depth': 1,
                'publish_tf': True,
                'output_frame': '',  # pose nel frame della camera
                'use_sim_time': True,
            }
        ],
    )

    # Launch navigation container
    navigation_container_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_assignment1_erl, 'launch', 'navigation_container.launch.py'),
        )
    )

    launchDescriptionObject = LaunchDescription()

    # Apply global remappings (escludendo /aruco_info e /fsm_status per mantenerli globali)
    remap_actions = [
        SetRemap(src='/clock', dst='/rosbot_1/clock'),
        SetRemap(src='/cmd_vel', dst='/rosbot_1/cmd_vel'),
        SetRemap(src='/odom', dst='/rosbot_1/odom'),
        SetRemap(src='/joint_states', dst='/rosbot_1/joint_states'),
        SetRemap(src='/tf', dst='/rosbot_1/tf'),
        SetRemap(src='/tf_static', dst='/rosbot_1/tf_static'),
        SetRemap(src='/camera/image', dst='/rosbot_1/camera/rgb/image_raw'),
        SetRemap(src='/camera/camera_info', dst='/rosbot_1/camera/rgb/camera_info'),
        SetRemap(src='/camera_joint/cmd_pos', dst='/rosbot_1/camera_joint/cmd_pos'),
        SetRemap(src='/imu', dst='/rosbot_1/imu'),
        SetRemap(src='/scan', dst='/rosbot_1/scan'),
        SetRemap(src='/scan/points', dst='/rosbot_1/scan/points'),
        SetRemap(src='/camera/depth_image', dst='/rosbot_1/camera/rgb/depth_image'),
        SetRemap(src='/camera/points', dst='/rosbot_1/camera/rgb/points'),
        SetRemap(src='/aruco_detections', dst='/rosbot_1/aruco_detections'),
        SetRemap(src='/maneuver_complete', dst='/rosbot_1/maneuver_complete'),
        SetRemap(src='/odometry/filtered', dst='/rosbot_1/odometry/filtered'),
        SetRemap(src='/robot_description', dst='/rosbot_1/robot_description'),
        SetRemap(src='robot_description', dst='/rosbot_1/robot_description'),
        # Aruco Tracker specific topics forniti
        SetRemap(src='/aruco_tracker/debug', dst='/rosbot_1/aruco_tracker/debug'),
        SetRemap(
            src='/aruco_tracker/debug/compressed', dst='/rosbot_1/aruco_tracker/debug/compressed'
        ),
        SetRemap(
            src='/aruco_tracker/debug/compressedDepth',
            dst='/rosbot_1/aruco_tracker/debug/compressedDepth',
        ),
        SetRemap(src='/aruco_tracker/debug/theora', dst='/rosbot_1/aruco_tracker/debug/theora'),
        SetRemap(src='/aruco_tracker/debug/zstd', dst='/rosbot_1/aruco_tracker/debug/zstd'),
        SetRemap(
            src='/aruco_tracker/transition_event', dst='/rosbot_1/aruco_tracker/transition_event'
        ),
    ]

    for remap in remap_actions:
        launchDescriptionObject.add_action(remap)

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    # launchDescriptionObject.add_action(joint_state_publisher_gui_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(gz_image_bridge_node)
    launchDescriptionObject.add_action(ekf_node)
    launchDescriptionObject.add_action(aruco_node)
    launchDescriptionObject.add_action(navigation_container_launch)

    return launchDescriptionObject
