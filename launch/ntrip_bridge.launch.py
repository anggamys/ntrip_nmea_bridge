from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    tcp_host = LaunchConfiguration('tcp_host')
    tcp_port = LaunchConfiguration('tcp_port')
    frame_id = LaunchConfiguration('frame_id')
    publish_fix = LaunchConfiguration('publish_fix')
    publish_twist = LaunchConfiguration('publish_twist')

    return LaunchDescription([
        DeclareLaunchArgument('tcp_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('tcp_port', default_value='52001'),
        DeclareLaunchArgument('frame_id', default_value='gps'),
        DeclareLaunchArgument('publish_fix', default_value='true'),
        DeclareLaunchArgument('publish_twist', default_value='true'),

        Node(
            package='ntrip_nmea_bridge',
            executable='ntrip_nmea_bridge_node',
            name='ntrip_nmea_bridge',
            output='screen',
            parameters=[{
                'tcp_host': tcp_host,
                'tcp_port': tcp_port,
                'frame_id': frame_id,
                'publish_fix': publish_fix,
                'publish_twist': publish_twist
            }]
        )
    ])
