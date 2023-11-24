from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the package share directory
    tsk_pkg_share = FindPackageShare(package='tsk').find('tsk')

    # Run tsk transport in a new terminal
    transport_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'tsk', 'transport'],
        output='screen'
    )

    return LaunchDescription([
        transport_node,
    ])
