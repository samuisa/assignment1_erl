from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Questo container raggruppa la navigazione e il visual servoing
    navigation_container = ComposableNodeContainer(
        name='navigation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        prefix='xterm -T "Navigation & Control Menu" -e',
        composable_node_descriptions=[
            ComposableNode(
                package='gazebo_ath',
                plugin='gazebo_ath::NavigationManager',
                name='navigation_node',
            ),
            ComposableNode(
                package='gazebo_ath',
                plugin='gazebo_ath::FsmManager',
                name='fsm_node',
            ),
            ComposableNode(
                package='gazebo_ath',
                plugin='gazebo_ath::DetectionManager',
                name='image_detection_node',
            ),
        ],
        output='screen',
    )

    # Le variabili tornate ora corrispondono a quelle dichiarate sopra
    return LaunchDescription([navigation_container])
