from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the package share directory
    tsk_pkg_share = FindPackageShare(package='tsk').find('tsk')
    turtlebot3_gazebo_share = FindPackageShare(package='turtlebot3_gazebo').find('turtlebot3_gazebo')
    turtlebot3_navigation2_share = FindPackageShare(package='turtlebot3_navigation2').find('turtlebot3_navigation2')

    # # Turtlebot3 Gazebo Simulation
    # turtlebot3_world_launch = ExecuteProcess(
    #     cmd=['ros2', 'launch', turtlebot3_gazebo_share, 'turtlebot3_world.launch.py'],
    #     output='screen'
    # )

    # Run tsk machine_command in a new terminal
    machine_command_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'tsk', 'machine_command'],
        output='screen'
    )

    # Run tsk transport in a new terminal
    transport_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'tsk', 'transport'],
        output='screen'
    )

    return LaunchDescription([
        #turtlebot3_world_launch,
        # navigation2_launch,
        machine_command_node,
        transport_node
    ])
