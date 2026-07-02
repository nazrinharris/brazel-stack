from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    hero_spawner_node = Node(
        package="brazel_bringup",
        executable="hero_spawner",
        name="hero_spawner",
        output="screen",
    )

    return LaunchDescription([hero_spawner_node])