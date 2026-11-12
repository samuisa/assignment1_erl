import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world', default_value='my_world.sdf',
        description='Name of the Gazebo world file to load'
    )

    pkg_assignment1_erl = get_package_share_directory('assignment1_erl')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Add your own gazebo library path here
    gazebo_models_path = os.path.join(pkg_assignment1_erl, "models")

    gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")

    if gazebo_models_path not in gz_path.split(os.pathsep):
        os.environ["GZ_SIM_RESOURCE_PATH"] = (
            gz_path + os.pathsep + gazebo_models_path
            if gz_path else gazebo_models_path
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': [
                PathJoinSubstitution([pkg_assignment1_erl, 'worlds', LaunchConfiguration('world')]),
                # TextSubstitution(text=' -r -v -v1 --render-engine ogre')],
                # TextSubstitution(text=' -r -v -v1')],
                TextSubstitution(text=' -r -v1'),
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(gazebo_launch)

    return launchDescriptionObject
