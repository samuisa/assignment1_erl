
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)

from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():

    pkg_assignment1_erl = get_package_share_directory('assignment1_erl')

    # =====================================================
    # LAUNCH ARGUMENTS
    # =====================================================

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz.'
    )

    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='1',
        description='Robot number used in the ROS namespace.'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='my_robot.urdf',
        description='Name of the URDF or Xacro file.'
    )

    camera_image_arg = DeclareLaunchArgument(
        'camera_image_topic',
        default_value='camera/image',
        description='Camera image topic.'
    )

    robot_namespace = PythonExpression(
        ["'rosbot_' + '", LaunchConfiguration('robot_number'), "'"]
    )

    urdf_file_path = PathJoinSubstitution(
        [
            pkg_assignment1_erl,
            'urdf',
            LaunchConfiguration('model')
        ]
    )

    # =====================================================
    # RVIZ
    # =====================================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg_assignment1_erl, 'rviz', 'urdf.rviz')
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ],
    )

    # =====================================================
    # ROBOT STATE PUBLISHER
    # =====================================================

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': Command(
                    ['xacro', ' ', urdf_file_path]
                ),
                'use_sim_time': False
            }
        ],
    )

    # =====================================================
    # ARUCO TRACKER
    # =====================================================

    aruco_node = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_tracker',
        output='screen',
        parameters=[
            {
                'cam_base_topic': LaunchConfiguration('camera_image_topic'),
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

    # =====================================================
    # IMAGE DETECTION NODE
    # =====================================================

    image_detection_node = Node(
        package='assignment1_erl',
        executable='image_detection_node',
        name='detection_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'marker_size_m': 0.2,
                'desired_size_sim_px': 166.0,
                'simulation_focal_length_px': 381.4,
            }
        ],
    )

    # =====================================================
    # FSM NODE
    # =====================================================

    fsm_node = Node(
        package='assignment1_erl',
        executable='fsm_node',
        name='fsm_node',
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ],
    )

    # =====================================================
    # NAVIGATION NODE
    # =====================================================

    navigation_node = Node(
        package='assignment1_erl',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ],
    )

    # =====================================================
    # ROBOT NAMESPACE AND TOPIC REMAPPING
    # =====================================================

    robot_nodes = GroupAction(
        actions=[

            PushRosNamespace(robot_namespace),

            # TF
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),

            # Robot navigation
            SetRemap(src='/cmd_vel', dst='cmd_vel'),
            SetRemap(src='/odom', dst='odom'),

            # FSM communication
            SetRemap(src='/maneuver_complete', dst='maneuver_complete'),
            SetRemap(src='/fsm_status', dst='fsm_status'),

            # ArUco communication
            SetRemap(src='/aruco_info', dst='aruco_info'),
            SetRemap(src='/aruco_detections', dst='aruco_detections'),

            # Camera calibration
            SetRemap(
                src='/camera/camera_info',
                dst='camera/camera_info'
            ),

            # Nodes
            rviz_node,
            robot_state_publisher_node,
            aruco_node,
            image_detection_node,
            fsm_node,
            navigation_node,
        ]
    )

    return LaunchDescription([
        rviz_launch_arg,
        robot_number_arg,
        model_arg,
        camera_image_arg,
        robot_nodes
    ])
