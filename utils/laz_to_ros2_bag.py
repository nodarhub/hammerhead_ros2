import os
import sys
import shutil
from pathlib import Path
from tqdm import tqdm

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py

import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg

import laspy

class LasToRos2Node(Node):
    def __init__(self, file_path, bag_path, topic_name):
        print("Path for las/laz files: ", file_path)
        print("Publishing to topic: ", topic_name)
        super().__init__('las_to_ros2_node')
        self.publisher_ = self.create_publisher(PointCloud2, topic_name, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.file_path = file_path

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=16, datatype=PointField.UINT32, count=1),  # Represent color as UINT32
        ]

        self.header = std_msgs.msg.Header()
        self.header.frame_id = "map"
        self.fields = fields
        self.topic_name = topic_name

        # Get the current time
        self.current_time = self.get_clock().now().nanoseconds

        # Create bag to record
        self.writer = rosbag2_py.SequentialWriter()

        # Overwrite bag directory if it exists
        if os.path.exists(bag_path):
            shutil.rmtree(bag_path)

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=str(bag_path),
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name=self.topic_name,
            type='sensor_msgs/PointCloud2',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.callback_run_once = False
    
    def read_las_file(self, file_path):
    
        fh = laspy.open(str(file_path))
        #print('Number of points from file header:', fh.header.point_count)
        las = fh.read()

        # Extract x, y, z, r, g, b as NumPy arrays
        x = las.x
        y = las.y
        z = las.z
        r = las.red
        g = las.green
        b = las.blue

        # Combine the arrays into a single NumPy array
        points = np.column_stack((x, y, z, r, g, b))

        return points
    
    def convert_rgb_to_uint32(self, r, g, b):
        # Combine separate RGB values into a single UINT32 value
        rgb_uint32 = (int(r) << 16) | (int(g) << 8) | int(b)
        return rgb_uint32

    def timer_callback(self):
        if not self.callback_run_once:
            # Iterate through LAS/LAZ files
            for i,file in tqdm(enumerate(list(sorted(self.file_path.glob("*.las"))) or list(sorted(self.file_path.glob("*.laz"))))):
                # Read LAS/LAZ file and extract points
                points = self.read_las_file(file)

                # Convert RGB values to UINT32
                points_with_rgb_uint32 = []
                for x, y, z, r, g, b in points:
                    rgb_uint32 = self.convert_rgb_to_uint32(r, g, b)
                    points_with_rgb_uint32.append([x, y, z, rgb_uint32])

                 # Create the PointCloud2 message
                pc2_msg = pc2.create_cloud(self.header, self.fields, points_with_rgb_uint32)
                
                # Publish the PointCloud2 message
                self.publisher_.publish(pc2_msg)

                # Time (increases the next frame time by 250ms)
                self.current_time += 250000000

                # Write to bag
                self.writer.write(
                self.topic_name,
                serialize_message(pc2_msg),
                self.current_time)

            self.callback_run_once = True
        
        else:
            self.get_logger().info("Callback executed once. Shutting down the node.")
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    if args is None:
        args = sys.argv

    if len(args) < 3:
        print("Usage: python laz_to_ros2_bag.py <source_directory_path> <output_bag_file> <pointcloud_topic> ")
        sys.exit(1)

    rclpy.init(args=args)

    source_directory_path = args[1]
    output_bag_file = args[2]
    topic_name = args[3] if len(args) >= 4 else "point_cloud_topic"

    # Check if the provided directory exists
    if not os.path.isdir(source_directory_path):
        print("Invalid directory path.")
        sys.exit(1)

    las_to_ros2_node = LasToRos2Node(Path(source_directory_path), Path(output_bag_file), topic_name)

    rclpy.spin_once(las_to_ros2_node)

    las_to_ros2_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
