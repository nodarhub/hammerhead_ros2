import os
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from generate_rosbag2_py.bag_writer import BagWriter
from generate_rosbag2_py.details import Details
from generate_rosbag2_py.get_files import get_files
from generate_rosbag2_py.safe_load import safe_load
from generate_rosbag2_py.to_image_msg import to_image_msg
from generate_rosbag2_py.to_point_cloud_msg import to_point_cloud_msg
from tqdm import tqdm


def main():
    if len(sys.argv) < 2:
        print(f"You passed the args: {sys.argv}")
        print("Expecting at least one argument (the path to the recorded data). "
              "Usage:\n\n\tros2 run generate_rosbag2_py xyz path_to_data [path_to_output_bag]")

        return

    input_dir = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(input_dir, "bag")

    disparity_dir = os.path.join(input_dir, "disparity")
    depth_dir = os.path.join(input_dir, "depth")
    details_dir = os.path.join(input_dir, "details")
    left_rect_dir = os.path.join(input_dir, "left-rect")

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
        if disparity_image is not None:
            disparity_image = cv2.convertScaleAbs(disparity_image, alpha=1.0 / 16.0)

        left_rect_tiff = os.path.join(left_rect_dir, os.path.splitext(os.path.basename(disparity))[0] + ".tiff")
        left_rect_png = os.path.join(left_rect_dir, os.path.splitext(os.path.basename(disparity))[0] + ".png")
        left_rect_filename = left_rect_tiff if os.path.exists(left_rect_tiff) else left_rect_png
        left_rect = safe_load(left_rect_filename, cv2.IMREAD_UNCHANGED, [np.uint8, np.uint16], 3)

        if disparity_image is None or left_rect is None:
            continue

        details_filename = os.path.join(details_dir, os.path.splitext(os.path.basename(disparity))[0] + ".csv")
        if not os.path.exists(details_filename):
            print(f"Could not find the corresponding details for\n{disparity}. "
                  f"This path does not exist:\n{details_filename}")
            continue
        details = Details(details_filename)
        bag_writer.write("nodar/point_cloud", to_point_cloud_msg(details, disparity_image, left_rect))

    rclpy.shutdown()


if __name__ == '__main__':
    main()
