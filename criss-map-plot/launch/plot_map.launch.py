from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    plot_map_node = Node(
        package="criss-map-plot",
        executable="plot_map",
        output="screen",
    )

    return LaunchDescription([plot_map_node])
