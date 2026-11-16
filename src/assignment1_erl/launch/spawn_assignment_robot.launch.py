"""
Launch file per il ROSbot REALE .

Tutto quello che puo' cambiare quando ti connetti al robot e' un argomento:

  ros2 launch assignment1_erl real_robot.launch.py robot_number:=2
  ros2 launch assignment1_erl real_robot.launch.py image_compressed:=true
  ros2 launch assignment1_erl real_robot.launch.py rviz:=false domain_id:=11

Non parte Gazebo, non parte robot_state_publisher (lo pubblica gia' il robot).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg = get_package_share_directory('assignment1_erl')

    # =====================================================
    # ARGOMENTI: rete e robot
    # =====================================================

    domain_id_arg = DeclareLaunchArgument(
        'domain_id', default_value='11',
        description='ROS_DOMAIN_ID (identificatore del dominio ROS) del robot.')

    robot_number_arg = DeclareLaunchArgument(
        'robot_number', default_value='1',
        description='Numero del robot: il namespace diventa /rosbot_<n>.')

    # =====================================================
    # ARGOMENTI: topic del robot (relativi al namespace /rosbot_<n>)
    # =====================================================

    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='camera/rgb/image_raw',
        description='Topic immagine (SENZA /rosbot_n davanti, lo aggiunge il namespace). '
                    'camera_info deve stare nella stessa cartella.')

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='camera/rgb/camera_info',
        description='Topic camera_info, letto da detection_node.')

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='odometry/filtered',
        description='Topic odometria, letto da navigation_node.')

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='cmd_vel',
        description='Topic dei comandi di velocita'' (da verificare con ros2 topic list).')

    # =====================================================
    # ARGOMENTI: ArUco tracker
    # =====================================================

    image_compressed_arg = DeclareLaunchArgument(
        'image_compressed', default_value='false',
        description='true = il tracker si iscrive a <image_topic>/compressed.')

    marker_dict_arg = DeclareLaunchArgument(
        'marker_dict', default_value='ARUCO_ORIGINAL',
        description='Dizionario ArUco dei marker reali.')

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.2',
        description='Lato REALE del marker in metri (misuralo con un righello).')

    image_rectified_arg = DeclareLaunchArgument(
        'image_is_rectified', default_value='false',
        description='true se l''immagine e'' gia'' priva di distorsione.')

    qos_reliability_arg = DeclareLaunchArgument(
        'qos_reliability', default_value='2',
        description='QoS (Quality of Service) affidabilita'': 1 = reliable, 2 = best effort.')

    qos_durability_arg = DeclareLaunchArgument(
        'qos_durability', default_value='2',
        description='QoS durabilita'': 1 = transient local, 2 = volatile.')

    # =====================================================
    # ARGOMENTI: image_detection_node (parametri camera)
    # =====================================================

    desired_size_arg = DeclareLaunchArgument(
        'desired_size_sim_px', default_value='166.0',
        description='Dimensione in pixel del marker alla distanza voluta, '
                    'misurata con la camera di riferimento.')

    reference_focal_arg = DeclareLaunchArgument(
        'simulation_focal_length_px', default_value='381.4',
        description='Focale (pixel) della camera di riferimento per desired_size_sim_px. '
                    'Il nodo la scala con la focale reale letta da camera_info.')

    # =====================================================
    # ARGOMENTI: RViz
    # =====================================================

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Apri RViz (ROS Visualization).')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=os.path.join(pkg, 'rviz', 'urdf.rviz'),
        description='Percorso del file di configurazione di RViz.')

    # =====================================================
    # NODI
    # =====================================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    aruco_node = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_tracker',
        output='screen',
        parameters=[{
            'cam_base_topic': LaunchConfiguration('image_topic'),
            'marker_dict': LaunchConfiguration('marker_dict'),
            'marker_size': ParameterValue(
                LaunchConfiguration('marker_size'), value_type=float),
            'image_is_rectified': ParameterValue(
                LaunchConfiguration('image_is_rectified'), value_type=bool),
            'image_sub_compressed': ParameterValue(
                LaunchConfiguration('image_compressed'), value_type=bool),
            'image_sub_qos.reliability': ParameterValue(
                LaunchConfiguration('qos_reliability'), value_type=int),
            'image_sub_qos.durability': ParameterValue(
                LaunchConfiguration('qos_durability'), value_type=int),
            'image_sub_qos.depth': 1,
            'publish_tf': True,
            'output_frame': '',
            'use_sim_time': False,
        }],
    )

    image_detection_node = Node(
        package='assignment1_erl',
        executable='image_detection_node',
        name='detection_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'marker_size_m': ParameterValue(
                LaunchConfiguration('marker_size'), value_type=float),
            'desired_size_sim_px': ParameterValue(
                LaunchConfiguration('desired_size_sim_px'), value_type=float),
            'simulation_focal_length_px': ParameterValue(
                LaunchConfiguration('simulation_focal_length_px'), value_type=float),
        }],
    )

    fsm_node = Node(
        package='assignment1_erl',
        executable='fsm_node',
        name='fsm_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    navigation_node = Node(
        package='assignment1_erl',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # =====================================================
    # NAMESPACE + REMAP
    # I nodi usano nomi assoluti (/cmd_vel, /odom, ...): ogni remap li
    # trasforma in un nome relativo, che il namespace porta a /rosbot_<n>/...
    # =====================================================

    robot_group = GroupAction(actions=[

        PushRosNamespace(['rosbot_', LaunchConfiguration('robot_number')]),

        # TF (Transform): il robot le pubblica dentro il suo namespace
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),

        # Topic del robot (nomi configurabili dagli argomenti)
        SetRemap(src='/cmd_vel', dst=LaunchConfiguration('cmd_vel_topic')),
        SetRemap(src='/odom', dst=LaunchConfiguration('odom_topic')),
        SetRemap(src='/camera/camera_info',
                 dst=LaunchConfiguration('camera_info_topic')),

        # Topic interni al tuo sistema (restano privati del robot)
        SetRemap(src='/aruco_detections', dst='aruco_detections'),
        SetRemap(src='/aruco_info', dst='aruco_info'),
        SetRemap(src='/fsm_status', dst='fsm_status'),
        SetRemap(src='/maneuver_complete', dst='maneuver_complete'),

        rviz_node,
        aruco_node,
        image_detection_node,
        fsm_node,
        navigation_node,
    ])

    return LaunchDescription([
        # Va impostato prima dei nodi: lo ereditano tutti i processi lanciati qui
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),

        domain_id_arg,
        robot_number_arg,
        image_topic_arg,
        camera_info_topic_arg,
        odom_topic_arg,
        cmd_vel_topic_arg,
        image_compressed_arg,
        marker_dict_arg,
        marker_size_arg,
        image_rectified_arg,
        qos_reliability_arg,
        qos_durability_arg,
        desired_size_arg,
        reference_focal_arg,
        rviz_arg,
        rviz_config_arg,

        robot_group,
    ])