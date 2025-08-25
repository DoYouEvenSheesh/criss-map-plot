#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import os


class MapPlotter(Node):
    def __init__(self):
        super().__init__("map_plotter")

        self.save_dir = "saved_maps"
        os.makedirs(self.save_dir, exist_ok=True)
        self.save_counter = 1
        self.save_timer = self.create_timer(20.0, self.periodic_save_callback)

        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.graph_subscription = self.create_subscription(
            MarkerArray, "/slam_toolbox/graph_visualization", self.graph_callback, 10
        )

        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.map_image = None
        self.path_plot = None
        self.start_pose_plot = None

        self.map_data = None
        self.display_map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.path_x = []
        self.path_y = []

        self.get_logger().info("Map Plotter Node has been started.")
        self.get_logger().info(
            f"Plots will be saved every 20 seconds in the '{self.save_dir}' directory."
        )
        self.get_logger().info(
            "Running in headless mode. Press Ctrl+C to save the final plot and exit."
        )

    def periodic_save_callback(self):
        filename = f"{self.save_counter}.jpeg"
        filepath = os.path.join(self.save_dir, filename)
        self.save_plot(filepath)
        self.save_counter += 1

    def map_callback(self, msg):
        self.get_logger().info("Received map data.")
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]

        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

        display_data = self.map_data.copy().astype(np.float32)
        display_data[display_data == -1] = 50.0

        self.display_map_data = display_data
        self.update_plot()

    def graph_callback(self, msg):
        self.get_logger().info(f"Received graph data with {len(msg.markers)} markers.")

        new_path_x = []
        new_path_y = []
        path_found_in_msg = False

        for marker in msg.markers:
            if marker.action != marker.ADD:
                continue

            if marker.type in [marker.LINE_STRIP, marker.LINE_LIST] and marker.points:
                path_found_in_msg = True
                for point in marker.points:
                    new_path_x.append(point.x)
                    new_path_y.append(point.y)

        if path_found_in_msg:
            self.path_x = new_path_x
            self.path_y = new_path_y
            self.get_logger().info(f"Updated path with {len(self.path_x)} points.")
            self.update_plot()
        else:
            self.get_logger().warn("No valid path marker found in message.")

    def update_plot(self):
        if self.display_map_data is not None:
            if self.map_image is None:
                self.map_image = self.ax.imshow(
                    self.display_map_data,
                    cmap="gray_r",
                    origin="lower",
                    extent=[
                        self.map_origin[0],
                        self.map_origin[0]
                        + self.map_data.shape[1] * self.map_resolution,
                        self.map_origin[1],
                        self.map_origin[1]
                        + self.map_data.shape[0] * self.map_resolution,
                    ],
                )
                self.ax.set_xlabel("X (meters)")
                self.ax.set_ylabel("Y (meters)")
                self.ax.set_title("Map and Robot Path")
                self.ax.set_aspect("equal", adjustable="box")
            else:
                self.map_image.set_data(self.display_map_data)
                self.map_image.set_extent(
                    [
                        self.map_origin[0],
                        self.map_origin[0]
                        + self.map_data.shape[1] * self.map_resolution,
                        self.map_origin[1],
                        self.map_origin[1]
                        + self.map_data.shape[0] * self.map_resolution,
                    ]
                )

        if self.path_x and self.path_y:
            if self.path_plot is None:
                (self.path_plot,) = self.ax.plot(
                    self.path_x, self.path_y, "r-", linewidth=1, label="Robot Path"
                )
                (self.start_pose_plot,) = self.ax.plot(
                    self.path_x[0],
                    self.path_y[0],
                    "o",
                    color="g",
                    markersize=8,
                    label="Start Pose",
                )
                self.ax.legend()
            else:
                self.path_plot.set_data(self.path_x, self.path_y)

        self.ax.relim()
        self.ax.autoscale_view()

    def save_plot(self, filename):
        if self.map_data is None:
            self.get_logger().warn("No map data received. Cannot save plot.")
            return

        self.get_logger().info(f"Saving plot to {filename}...")
        try:
            self.fig.savefig(filename, dpi=300, bbox_inches="tight")
            self.get_logger().info("Plot saved successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to save plot: {e}")


def main(args=None):
    rclpy.init(args=args)
    map_plotter = MapPlotter()

    try:
        rclpy.spin(map_plotter)
    except KeyboardInterrupt:
        map_plotter.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        final_filename = os.path.join(map_plotter.save_dir, "final_map.jpeg")
        map_plotter.get_logger().info("Saving final map...")
        map_plotter.save_plot(final_filename)

        map_plotter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
