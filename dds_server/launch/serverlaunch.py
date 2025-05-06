from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    server_id = '69'
    server_ip = '10.0.0.1'
    server_port = '11811'

    return LaunchDescription([
        ExecuteProcess(
        cmd=[
            'fastdds', 'discovery', '-i', server_id,
            '-l', server_ip, '-p', server_port
        ],
        name='discovery_server',
        output='screen'
        )
    ])
