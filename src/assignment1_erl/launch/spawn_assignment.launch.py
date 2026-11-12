import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
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
            "robot_description",
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
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
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
            # "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            # "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
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
                'cam_base_topic': 'camera/image',
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

    assignment_nodes_container = ComposableNodeContainer(
        name='assignment_nodes_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='assignment1_erl',
                plugin='assignment1_erl::FsmManager',
                name='fsm_node',
            ),
            ComposableNode(
                package='assignment1_erl',
                plugin='assignment1_erl::NavigationManager',
                name='navigation_node',
            ),
            ComposableNode(
                package='assignment1_erl',
                plugin='assignment1_erl::DetectionManager',
                name='image_detection_node',
            ),
        ],
        output='screen',
    )


    launchDescriptionObject = LaunchDescription()

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
    launchDescriptionObject.add_action(assignment_nodes_container)

    return launchDescriptionObject
