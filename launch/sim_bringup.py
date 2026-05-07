import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            [os.path.join(get_package_share_directory('nav'), 'config', 'sim.rviz')]
        ],
        parameters=[{
            'use_sim_time': True
        }]
    )

    pure_pursuit_tracker = Node(
        package="nav",
        executable="pure_pursuit",
        name="pure_pursuit",
        parameters=[{
            'use_sim_time': True,
            'debug': True,
            'control_hz': 30.0,
            'lookahead_distance': 0.5,
            'lookahead_time': 1.5,
            'min_lookahead_distance': 0.3,
            'max_lookahead_distance': 0.9,
            'use_velocity_scaled_lookahead': True,
            'desired_linear_vel': 0.3,
            'desired_angular_vel': 0.35,
            'use_regulated_linear_velocity_scaling': True,
            'regulated_linear_scaling_min_radius': 0.90,
            'velocity_scaling_distance': 0.5,
            'v_max': 0.5,
            'dv_max': 0.1,
            'w_max': 0.5,
            'dw_max': 0.25,
            'k_rot': 1.0,
            'goal_position_tolerance': 0.2,
            'use_rotate_to_heading': True,
            'rotate_to_heading_min_angle': 0.785,
            'point_threshold': 15,
            'obstacle_avoidance_distance': 1.0,
        }]
    )

    mcp_node = Node(
        package="nav",
        executable="mpc.py",
        name="mcp",
        parameters=[{
            'use_sim_time': True
        }],
        output="screen"
    )


    nodes = []
    nodes.append(turtlebot3_gazebo)
    # nodes.append(pure_pursuit_tracker)
    nodes.append(rviz)
    nodes.append(mcp_node)

    return LaunchDescription(nodes)
