#!/usr/bin/env python3
"""
Publish a folder of images on a ROS2 topic at a specified FPS.

Usage:
    ros2 run topbot_publisher_py topbot_publisher_py /path/to/images [--topic /nodar/topbots] [--fps 10] [--encoding auto]
"""

import argparse
import glob
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

DEFAULT_IMAGE_DIR = "."
DEFAULT_TOPIC = "/nodar/topbots"
DEFAULT_FPS = 10.0


def detect_encoding(img):
    """Detect the ROS2 image encoding from a cv2 image."""
    ndim = img.ndim
    depth = img.dtype

    if ndim == 2:
        # Single channel
        if depth == "uint8":
            return "mono8"
        elif depth == "uint16":
            return "mono16"
    elif ndim == 3:
        channels = img.shape[2]
        if channels == 3:
            if depth == "uint8":
                return "bgr8"
            elif depth == "uint16":
                return "bgr16"
        elif channels == 4:
            if depth == "uint8":
                return "bgra8"
            elif depth == "uint16":
                return "bgra16"

    raise ValueError(f"Unsupported image format: ndim={ndim}, dtype={depth}, shape={img.shape}")


def main():
    parser = argparse.ArgumentParser(description="Publish images on a ROS2 topic")
    parser.add_argument("image_dir", nargs="?", default=DEFAULT_IMAGE_DIR, help="Path to image folder")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="ROS2 topic name")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS, help="Publish rate in frames per second")
    parser.add_argument(
        "--encoding",
        default="auto",
        help="Image encoding (auto, mono8, mono16, bgr8, bgr16, bayer_bggr8, bayer_rggb8, etc.)",
    )
    parser.add_argument("--loop", action="store_true", help="Loop over the images continuously")
    args = parser.parse_args()

    image_dir = os.path.abspath(args.image_dir)
    files = sorted(glob.glob(os.path.join(image_dir, "*.tiff")))
    if not files:
        files = sorted(glob.glob(os.path.join(image_dir, "*.tif")))
    if not files:
        files = sorted(glob.glob(os.path.join(image_dir, "*.png")))
    if not files:
        print(f"No images found in {image_dir}")
        return

    print(f"Found {len(files)} images in {image_dir}")
    print(f"Publishing on topic: {args.topic} at {args.fps} fps")

    rclpy.init()
    node = Node("image_publisher")

    # Use sensor data QoS profile to match typical subscribers
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    pub = node.create_publisher(Image, args.topic, qos)
    bridge = CvBridge()

    period = 1.0 / args.fps

    # Detect encoding from first image if auto
    encoding = args.encoding
    if encoding == "auto":
        first_img = cv2.imread(files[0], cv2.IMREAD_UNCHANGED)
        encoding = detect_encoding(first_img)
        print(f"Auto-detected encoding: {encoding}")

    # For bayer encodings, load as grayscale
    load_unchanged = encoding.startswith("bayer_") or encoding in ("mono8", "mono16")

    try:
        run = True
        next_publish_time = time.monotonic()
        frame_count = 0
        while run:
            run = args.loop
            for f in files:
                t0 = time.monotonic()
                img = cv2.imread(f, cv2.IMREAD_UNCHANGED)
                t1 = time.monotonic()

                if img is None:
                    print(f"Failed to read {f}, skipping")
                    continue

                t2 = time.monotonic()
                msg = bridge.cv2_to_imgmsg(img, encoding=encoding)
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = "camera"
                t3 = time.monotonic()

                remaining = next_publish_time - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)

                t4 = time.monotonic()
                pub.publish(msg)
                t5 = time.monotonic()
                next_publish_time += period
                frame_count += 1

                print(f"[{frame_count}] imread={t1-t0:.3f}s  cv2msg={t3-t2:.3f}s  publish={t5-t4:.3f}s  total={t5-t0:.3f}s")

    except KeyboardInterrupt:
        pass

    print("Done")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
