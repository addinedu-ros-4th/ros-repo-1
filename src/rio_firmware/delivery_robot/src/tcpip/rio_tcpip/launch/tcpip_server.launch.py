from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rio_tcpip',
            executable='ip_publisher_node.py',
            name='ip_publisher_node',
            output='screen',
            prefix='python3'
        ),
        Node(
            package='rio_tcpip',
            executable='tcpip_server_node.py',
            name='tcpip_server_node',
            output='screen',
            prefix='python3'
        )
    ])
