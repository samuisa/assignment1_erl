from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    workspace_root = Path(get_package_prefix('assignment1_erl')).resolve().parents[1]
    log_dir = workspace_root / 'assignment' / 'assignment1_erl' / 'log'
    log_dir.mkdir(parents=True, exist_ok=True)

    nodes = [
        ('fsm_node', 'fsm.log'),
        ('navigation_node', 'navigation.log'),
        ('image_detection_node', 'image_detection.log'),
    ]

    return LaunchDescription([
        Node(
            package='assignment1_erl',
            executable=executable,
            name=executable,
            prefix=(
                "bash -c 'exec \"$@\" > "
                f"\"{log_dir / log_file}\" 2>&1' --"
            ),
            output='screen',
        )
        for executable, log_file in nodes
    ])
