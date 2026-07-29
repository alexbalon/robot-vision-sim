"""
robot.launch.py
Orquestación reproducible de la prueba experimental:
  1. Levanta Gazebo Sim con line_world.world
  2. Publica el URDF vía robot_state_publisher (TF)
  3. Genera (spawn) el robot en el mundo
  4. Activa el puente ros_gz_bridge (Gazebo <-> ROS2) para cámara y cmd_vel
  5. Lanza el nodo de visión y control con los parámetros de config/params.yaml

Uso:
  $ ros2 launch robot_vision_gazebo robot.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_vision_gazebo')

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'line_world.world')
    params_path = os.path.join(pkg_share, 'config', 'params.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # 1. Gazebo Sim con el mundo de la pista
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 2. robot_state_publisher (transformaciones TF del gemelo digital)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                      'use_sim_time': True}],
    )

    # 3. Spawn del robot dentro del mundo ya cargado
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot_vision',
                   '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0.06'],
        output='screen',
    )

    # 4. Puente Gazebo <-> ROS2 (imagen de cámara y cmd_vel)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen',
    )

    # 5. Nodo de visión y control (PID + FSM de evasión)
    vision_control_node = Node(
        package='robot_vision_gazebo',
        executable='vision_node',
        name='vision_control_node',
        output='screen',
        parameters=[params_path, {'use_sim_time': True}],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        vision_control_node,
    ])
