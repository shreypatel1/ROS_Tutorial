from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('stinger_bringup'),
        'launch',
        'vehicle_sim.launch.py'
    )

    return LaunchDescription([
        Node(
            package='student_code',
            executable='question_4_3',
            name='question_4_3',
            output='screen',
        ),

        Node(
            package='helpers',
            executable='node_q_4_2',
            name='node_q_4_2',
            output='screen',
        ),

        Node(
            package='autograder',
            executable='test_topic_4_3',
            name='test_topic_4_3',
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={'world': 'empty.world'}.items()
        ),
    ])
