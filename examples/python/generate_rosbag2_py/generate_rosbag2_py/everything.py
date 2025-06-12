import os
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from generate_rosbag2_py.bag_writer import BagWriter
from generate_rosbag2_py.details_parameters import DetailsParameters
from generate_rosbag2_py.get_files import get_files
from generate_rosbag2_py.safe_load import safe_load
from generate_rosbag2_py.to_image_msg import to_image_msg
from generate_rosbag2_py.to_point_cloud_msg import to_point_cloud_msg
from tqdm import tqdm


def main():
    if len(sys.argv) < 2:
        print(f"You passed the args: {sys.argv}")
        print("Expecting at least one argument (the path to the recorded data). "
              "Usage:\n\n\tros2 run generate_rosbag2_py everything path_to_data [path_to_output_bag]")

        return

    input_dir = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(input_dir, "bag")

    disparity_dir = os.path.join(input_dir, "disparity")
    depth_dir = os.path.join(input_dir, "depth")
    depth_colormap_dir = os.path.join(input_dir, "depth-colormap")  # Optional
    details_dir = os.path.join(input_dir, "details")
    left_rect_dir = os.path.join(input_dir, "left-rect")
    topbot_dir = os.path.join(input_dir, "topbot")

    if not os.path.exists(disparity_dir):
        if os.path.exists(depth_dir):
            print("\n[WARNING] No 'disparity' folder found, but 'depth' exists.")
            return
        else:
            print(f"[ERROR] Required 'disparity' folder is missing and no 'depth' folder was found.\n"
                  f"Please verify the input path: {input_dir}")
            return

    topics = [
        {"name": "nodar/point_cloud", "type": "sensor_msgs/msg/PointCloud2"},
        {"name": "nodar/left/image_raw", "type": "sensor_msgs/msg/Image"},
        {"name": "nodar/right/image_raw", "type": "sensor_msgs/msg/Image"},
        {"name": "nodar/left/image_rect", "type": "sensor_msgs/msg/Image"},
        {"name": "nodar/disparity/image_raw", "type": "sensor_msgs/msg/Image"},
        {"name": "nodar/color_blended_depth/image_raw", "type": "sensor_msgs/msg/Image"},
    ]

    if os.path.exists(output_dir):
        print(f"Something already exists in the directory\n\t{output_dir}\nDid you already generate this bag?")
        return
    os.makedirs(Path(output_dir).parent, exist_ok=True)

    rclpy.init()
    bag_writer = BagWriter(output_dir, topics)

    disparities = get_files(disparity_dir, ".tiff")
    print(f"Found {len(disparities)} disparity maps to convert to point clouds")

    bridge = CvBridge()
    for disparity in tqdm(disparities):
        disparity_image = safe_load(disparity, cv2.IMREAD_UNCHANGED, [np.uint16, ], 1)

        left_rect_tiff = os.path.join(left_rect_dir, os.path.splitext(os.path.basename(disparity))[0] + ".tiff")
        left_rect_png = os.path.join(left_rect_dir, os.path.splitext(os.path.basename(disparity))[0] + ".png")
        left_rect_filename = left_rect_tiff if os.path.exists(left_rect_tiff) else left_rect_png
        left_rect = safe_load(left_rect_filename, cv2.IMREAD_COLOR, [np.uint8], 3)

        topbot_tiff = os.path.join(topbot_dir, os.path.splitext(os.path.basename(disparity))[0] + ".tiff")
        topbot_png = os.path.join(topbot_dir, os.path.splitext(os.path.basename(disparity))[0] + ".png")
        topbot_filename = topbot_tiff if os.path.exists(topbot_tiff) else topbot_png
        topbot = safe_load(topbot_filename, cv2.IMREAD_COLOR, [np.uint8], 3)

        if disparity_image is None or left_rect is None or topbot is None:
            continue

        left_raw = topbot[:topbot.shape[0] // 2]
        right_raw = topbot[topbot.shape[0] // 2:]

        details_filename = os.path.join(details_dir, os.path.splitext(os.path.basename(disparity))[0] + ".yaml")
        if not os.path.exists(details_filename):
            print(f"Could not find the corresponding details for\n{disparity}. "
                  f"This path does not exist:\n{details_filename}")
            continue
        details = DetailsParameters(details_filename)
        bag_writer.write("nodar/point_cloud", to_point_cloud_msg(details, disparity_image, left_rect))
        bag_writer.write("nodar/left/image_raw", to_image_msg(bridge, left_raw, details.left_time))
        bag_writer.write("nodar/right/image_raw", to_image_msg(bridge, right_raw, details.right_time))
        bag_writer.write("nodar/left/image_rect", to_image_msg(bridge, left_rect, details.left_time))
        bag_writer.write("nodar/disparity/image_raw", to_image_msg(bridge, disparity_image, details.left_time))
        # Optional depth colormap
        colormap_file = os.path.join(depth_colormap_dir, os.path.splitext(os.path.basename(disparity))[0] + ".tiff")

        if os.path.exists(colormap_file):
            depth_colormap = cv2.imread(colormap_file, cv2.IMREAD_COLOR)
            if depth_colormap is not None:
                bag_writer.write("nodar/color_blended_depth/image_raw",
                                 to_image_msg(bridge, depth_colormap, details.left_time))

    rclpy.shutdown()


if __name__ == '__main__':
    main()
