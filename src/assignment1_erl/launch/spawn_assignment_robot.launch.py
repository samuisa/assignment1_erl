"""
Launch file unificato per ROSbot (Simulazione Gazebo e Robot Reale).

Esempi d'uso:
  # Simulazione (default: sim:=true)
  ros2 launch assignment1_erl spawn_assignment_rosbot.launch.py

  # Robot reale (disabilita Gazebo/EKF/RSP, imposta use_sim_time:=false e ROS_DOMAIN_ID)
  ros2 launch assignment1_erl spawn_assignment_rosbot.launch.py sim:=false robot_number:=2
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_assignment1_erl = get_package_share_directory('assignment1_erl')

    # Permette a Gazebo di trovare package://assignment1_erl/... (es. mesh)
    gazebo_models_path, _ = os.path.split(pkg_assignment1_erl)
    gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_path + os.pathsep + gazebo_models_path

    # =====================================================
    # ARGOMENTI: modalità, rete e robot
    # =====================================================

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description="true = avviato in simulazione Gazebo (use_sim_time=true); "
        "false = robot reale (use_sim_time=false).",
    )

    domain_id_arg = DeclareLaunchArgument(
        'domain_id',
        default_value='13',
        description="ROS_DOMAIN_ID del robot reale (applicato quando sim:=false).",
    )

    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='3',
        description="Numero del robot: il namespace diventa /rosbot_<n>.",
    )

    # =====================================================
    # ARGOMENTI: simulazione Gazebo / URDF
    # =====================================================

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='assignment.sdf',
        description="Nome del file world di Gazebo da caricare.",
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='my_robot.urdf',
        description="Nome del file URDF/Xacro del robot.",
    )

    # =====================================================
    # ARGOMENTI: topic del robot (relativi al namespace /rosbot_<n>)
    # =====================================================

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='camera/rgb/image_raw',
        description="Topic immagine (senza /rosbot_<n>, aggiunto dal namespace).",
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='camera/rgb/camera_info',
        description="Topic camera_info, letto da detection_node.",
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='odometry/filtered',
        description="Topic odometria filtrata, letto da navigation_node.",
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description="Topic dei comandi di velocita'.",
    )

    # =====================================================
    # ARGOMENTI: ArUco tracker
    # =====================================================

    image_compressed_arg = DeclareLaunchArgument(
        'image_compressed',
        default_value='true',
        description="true = il tracker si iscrive a <image_topic>/compressed.",
    )

    marker_dict_arg = DeclareLaunchArgument(
        'marker_dict',
        default_value='ARUCO_ORIGINAL',
        description="Dizionario ArUco dei marker.",
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.2',
        description="Lato reale del marker in metri.",
    )

    image_rectified_arg = DeclareLaunchArgument(
        'image_is_rectified',
        default_value='false',
        description="true se l'immagine e' gia' priva di distorsione.",
    )

    qos_reliability_arg = DeclareLaunchArgument(
        'qos_reliability',
        default_value='2',
        description="QoS affidabilita': 1 = reliable, 2 = best effort.",
    )

    qos_durability_arg = DeclareLaunchArgument(
        'qos_durability',
        default_value='2',
        description="QoS durabilita': 1 = transient local, 2 = volatile.",
    )

    # =====================================================
    # ARGOMENTI: image_detection_node (parametri camera)
    # =====================================================

    desired_size_arg = DeclareLaunchArgument(
        'desired_size_sim_px',
        default_value='140.0',
        description="Dimensione in pixel del marker alla distanza voluta.",
    )

    reference_focal_arg = DeclareLaunchArgument(
        'simulation_focal_length_px',
        default_value='381.4',
        description="Focale (pixel) della camera di riferimento per desired_size_sim_px.",
    )

    # =====================================================
    # ARGOMENTI: RViz e image_annotator
    # =====================================================

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description="Apri RViz.",
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_assignment1_erl, 'rviz', 'urdf.rviz'),
        description="Percorso del file di configurazione di RViz.",
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.join(os.path.expanduser('~'), 'marker_images_robot'),
        description="Cartella dove salvare le immagini annotate.",
    )

    # =====================================================
    # CONFIGURAZIONI COMUNI
    # =====================================================

    use_sim_time = ParameterValue(LaunchConfiguration('sim'), value_type=bool)
    sim_condition = IfCondition(LaunchConfiguration('sim'))

    urdf_file_path = PathJoinSubstitution(
        [
            pkg_assignment1_erl,
            'urdf',
            LaunchConfiguration('model'),
        ]
    )

    # =====================================================
    # NODI DI SIMULAZIONE (attivi solo se sim:=true)
    # =====================================================

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_assignment1_erl, 'launch', 'world.launch.py'),
        ),
        condition=sim_condition,
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    spawn_urdf_node = Node(
        package='ros_gz_sim',
        executable='create',
        condition=sim_condition,
        arguments=[
            '-name',
            'my_robot',
            '-topic',
            ['/rosbot_', LaunchConfiguration('robot_number'), '/robot_description'],
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.5',
            '-Y',
            '0.0',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        condition=sim_condition,
        output='screen',
        parameters=[
            {
                'robot_description': Command(['xacro', ' ', urdf_file_path]),
                'use_sim_time': use_sim_time,
            }
        ],
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        condition=sim_condition,
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        # Evita collisione con odometry/filtered: odom grezzo va all'EKF
        remappings=[('/odom', 'odom')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    gz_image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        condition=sim_condition,
        arguments=['/camera/image'],
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'camera.image.compressed.jpeg_quality': 75,
            }
        ],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        condition=sim_condition,
        output='screen',
        parameters=[
            os.path.join(pkg_assignment1_erl, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # =====================================================
    # NODI COMUNI (Simulazione e Robot Reale)
    # =====================================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    aruco_node = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_tracker',
        output='screen',
        parameters=[
            {
                'cam_base_topic': LaunchConfiguration('image_topic'),
                'marker_dict': LaunchConfiguration('marker_dict'),
                'marker_size': ParameterValue(LaunchConfiguration('marker_size'), value_type=float),
                'image_is_rectified': ParameterValue(
                    LaunchConfiguration('image_is_rectified'), value_type=bool
                ),
                'image_sub_compressed': ParameterValue(
                    LaunchConfiguration('image_compressed'), value_type=bool
                ),
                'image_sub_qos.reliability': ParameterValue(
                    LaunchConfiguration('qos_reliability'), value_type=int
                ),
                'image_sub_qos.durability': ParameterValue(
                    LaunchConfiguration('qos_durability'), value_type=int
                ),
                'image_sub_qos.depth': 1,
                'publish_tf': True,
                'output_frame': '',
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # Launch navigation container
    navigation_container_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_assignment1_erl, 'launch', 'navigation_container.launch.py'),
        )
    )

    # =====================================================
    # NAMESPACE + REMAP UNIFICATI
    # =====================================================

    robot_group = GroupAction(
        actions=[
            PushRosNamespace(['rosbot_', LaunchConfiguration('robot_number')]),
            # Clock e TF nel namespace del robot
            SetRemap(src='/clock', dst='clock'),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            SetRemap(src='/robot_description', dst='robot_description'),
            SetRemap(src='robot_description', dst='robot_description'),
            # Topic hardware / bridge Gazebo
            SetRemap(src='/cmd_vel', dst=LaunchConfiguration('cmd_vel_topic')),
            SetRemap(src='/odom', dst=LaunchConfiguration('odom_topic')),
            SetRemap(src='/odometry/filtered', dst=LaunchConfiguration('odom_topic')),
            SetRemap(src='/joint_states', dst='joint_states'),
            SetRemap(src='/imu', dst='imu'),
            SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/scan/points', dst='scan/points'),
            SetRemap(src='/camera_joint/cmd_pos', dst='camera_joint/cmd_pos'),
            # Topic camera (raw + compressed)
            SetRemap(src='/camera/image', dst=LaunchConfiguration('image_topic')),
            SetRemap(
                src='/camera/image/compressed',
                dst=[LaunchConfiguration('image_topic'), '/compressed'],
            ),
            SetRemap(src='/camera/camera_info', dst=LaunchConfiguration('camera_info_topic')),
            SetRemap(src='/camera/depth_image', dst='camera/rgb/depth_image'),
            SetRemap(src='/camera/points', dst='camera/rgb/points'),
            # Topic interni al sistema
            SetRemap(src='/aruco_detections', dst='aruco_detections'),
            SetRemap(src='/aruco_info', dst='aruco_info'),
            SetRemap(src='/fsm_status', dst='fsm_status'),
            SetRemap(src='/maneuver_complete', dst='maneuver_complete'),
            SetRemap(src='/marker_reached', dst='marker_reached'),
            SetRemap(src='/marker_image', dst='marker_image'),
            # Nodi di simulazione (condizionati a sim:=true)
            robot_state_publisher_node,
            gz_bridge_node,
            gz_image_bridge_node,
            ekf_node,
            # Nodi comuni
            rviz_node,
            aruco_node,
        ]
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(navigation_container_launch)

    return LaunchDescription(
        [
            # 1) Dichiarazione argomenti
            sim_arg,
            domain_id_arg,
            robot_number_arg,
            world_arg,
            model_arg,
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
            output_dir_arg,
            # 2) Variabile d'ambiente (applicata solo su robot reale)
            SetEnvironmentVariable(
                'ROS_DOMAIN_ID',
                LaunchConfiguration('domain_id'),
                condition=UnlessCondition(LaunchConfiguration('sim')),
            ),
            # 3) Avvio Gazebo e spawn (solo se sim:=true)
            world_launch,
            spawn_urdf_node,
            # 4) Gruppo nodi nel namespace /rosbot_<n>
            robot_group,
        ]
    )
