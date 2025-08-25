from launch import LaunchDescription

def generate_launch_descripion():

    plot_map_node = Node(
        package="criss_plot_map",
        executable="plot_map",
        output="screen",
    )

    return LaunchDescription([plot_map_node])
