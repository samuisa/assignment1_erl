import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_assignment1_erl = get_package_share_directory('assignment1_erl')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Open RViz.'
    )
    robot_number_arg = DeclareLaunchArgument(
        'robot_number', default_value='1', description='Robot number used in the ROS namespace.'
    )
    model_arg = DeclareLaunchArgument(
        'model', default_value='my_robot.urdf', description='Name of the URDF or Xacro file to load.'
    )

    robot_namespace = PythonExpression(
        ["'rosbot_' + '", LaunchConfiguration('robot_number'), "'"]
    )
    urdf_file_path = PathJoinSubstitution(
        [pkg_assignment1_erl, 'urdf', LaunchConfiguration('model')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_assignment1_erl, 'rviz', 'urdf.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': False}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]), 'use_sim_time': False},
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
                'marker_size': 0.2,
                'image_is_rectified': False,
                'image_sub_compressed': False,
                'image_sub_qos.reliability': 2,
                'image_sub_qos.durability': 2,
                'image_sub_qos.depth': 1,
                'publish_tf': True,
                'output_frame': '',
                'use_sim_time': False,
            }
        ],
    )

    topic_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', 'cmd_vel'),
        ('/maneuver_complete', 'maneuver_complete'),
        ('/odom', 'odom'),
        ('/aruco_info', 'aruco_info'),
        ('/fsm_status', 'fsm_status'),
        ('/camera/camera_info', 'camera/camera_info'),
        ('/aruco_detections', 'aruco_detections'),
    ]

    assignment_nodes_container = ComposableNodeContainer(
        name='assignment_nodes_container',
        namespace=robot_namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='assignment1_erl',
                plugin='assignment1_erl::FsmManager',
                name='fsm_node',
                namespace=robot_namespace,
                remappings=topic_remappings,
            ),
            ComposableNode(
                package='assignment1_erl',
                plugin='assignment1_erl::NavigationManager',
                name='navigation_node',
                namespace=robot_namespace,
                remappings=topic_remappings,
            ),
            ComposableNode(
                package='assignment1_erl',
                plugin='assignment1_erl::DetectionManager',
                name='image_detection_node',
                namespace=robot_namespace,
                remappings=topic_remappings,
            ),
        ],
        output='screen',
    )

    robot_nodes = GroupAction(
        actions=[
            PushRosNamespace(robot_namespace),
            *[
                SetRemap(src=source, dst=target)
                for source, target in topic_remappings
            ],
            rviz_node,
            robot_state_publisher_node,
            aruco_node,
        ]
    )

    return LaunchDescription(
        [
            rviz_launch_arg,
            robot_number_arg,
            model_arg,
            robot_nodes,
            assignment_nodes_container,
        ]
    )
    