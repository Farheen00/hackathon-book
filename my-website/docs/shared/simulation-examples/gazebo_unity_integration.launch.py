# ROS 2 launch file for Gazebo-Unity integration
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch Gazebo with a world file
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'simple_physics.sdf'],
        output='screen'
    )

    # Launch ROS-TCP-Endpoint to bridge ROS 2 and Unity
    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        parameters=[
            {'tcp_ip': '127.0.0.1'},
            {'tcp_port': 10000},
            {'unity_ip': '127.0.0.1'},
            {'unity_port': 5005}
        ],
        output='screen'
    )

    # Launch a simple controller node
    controller_node = Node(
        package='demo_nodes_py',
        executable='talker',
        name='controller_node',
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add actions
    ld.add_action(gz_sim)
    ld.add_action(ros_tcp_endpoint)
    ld.add_action(controller_node)

    return ld