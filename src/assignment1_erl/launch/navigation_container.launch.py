from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    workspace_root = Path(get_package_prefix('assignment1_erl')).resolve().parents[1]
    log_dir = workspace_root / 'src' / 'assignment1_erl' / 'log'
    log_dir.mkdir(parents=True, exist_ok=True)

    nodes = [
        ('fsm_node', 'fsm.log'),
        ('image_detection_node', 'image_detection.log'),
        ('navigation_node', 'navigation.log'),
        ('image_annotator_node', 'image_annotator.log'),
    ]

    for _, log_file in nodes:
        (log_dir / log_file).write_text('')

    return LaunchDescription(
        [
            Node(
                package='assignment1_erl',
                executable=executable,
                name=executable,
                prefix=f'bash -c \'exec "$@" > "{log_dir / log_file}" 2>&1\' --',
                output='screen',
            )
            for executable, log_file in nodes
        ]
    )