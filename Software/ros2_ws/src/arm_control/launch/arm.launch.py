import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Pfad zur YAML-Datei automatisch auflösen
    param_config = os.path.join(
        get_package_share_directory('arm_control'),
        'config',
        'arm_parameter.yaml'
    )
    pkg_share = get_package_share_directory('arm_control')
    rqt_perspective_path = os.path.join(pkg_share, 'config', 'arm_dashboard.perspective')

    return LaunchDescription([
        # Node 1: Input
        Node(
            package='arm_control',
            executable='step_publisher',
            name='step_publisher',
            output='screen',
            emulate_tty=True  # Wichtig, damit print() und input() im Terminal funktionieren
        ),
        # Node 2: Umrechnung + Parameter-Datei
        Node(
            package='arm_control',
            executable='step_converter',
            name='step_to_angle_converter',
            parameters=[param_config],
            output='screen'
        ),
        # In arm_system.launch.py hinzufügen:
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
            arguments=['--perspective-file', rqt_perspective_path],
            output='screen')
                # Node 3: UART-Bridge
    ])