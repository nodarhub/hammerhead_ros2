import sys
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class Ros2ImageViewer(Node):
    def __init__(self, topic):
        super().__init__('ros2_image_viewer')
        self.topic = topic
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.on_new_image,
            self.qos_profile
        )
        cv2.namedWindow(topic, cv2.WINDOW_NORMAL)
        self.original_window_flag = cv2.getWindowProperty(topic, cv2.WND_PROP_VISIBLE)
        self.stop_command = False
        self.frame_count = 0
        self.timestamps = deque(maxlen=20)

    def from_message(self, msg):
        # Get the type
        if msg.encoding in ('bayer_bggr8', 'bayer_rggb8', 'bayer_gbrg8', 'bayer_grbg8', 'mono8'):
            channels, dtype = 1, np.uint8
        elif msg.encoding in ('bgr8', 'rgb8'):
            channels, dtype = 3, np.uint8
        elif msg.encoding in ('bgra8', 'rgba8'):
            channels, dtype = 4, np.uint8
        elif msg.encoding in ('bayer_bggr16', 'bayer_rggb16', 'bayer_gbrg16', 'bayer_grbg16', 'mono16'):
            channels, dtype = 1, np.uint16
        elif msg.encoding in ('bgr16', 'rgb16'):
            channels, dtype = 3, np.uint16
        elif msg.encoding in ('bgra16', 'rgba16'):
            channels, dtype = 4, np.uint16
        else:
            print(f"Unknown image encoding `{msg.encoding}`")
            return None

        if channels != 1:
            img = np.array(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
        else:
            img = np.array(msg.data, dtype=dtype).reshape(msg.height, msg.width)

        # Convert Bayer and RGB encodings to BGR
        to_bgr_codes = {
            "bayer_bggr": cv2.COLOR_BayerBGGR2BGR,
            "bayer_rggb": cv2.COLOR_BayerRGGB2BGR,
            "bayer_gbrg": cv2.COLOR_BayerGBRG2BGR,
            "bayer_grbg": cv2.COLOR_BayerGRBG2BGR,
            "rgb": cv2.COLOR_RGB2BGR,
            "rgba": cv2.COLOR_RGBA2BGR,
        }
        pattern = msg.encoding.rstrip("0123456789")
        if pattern in to_bgr_codes:
            img = cv2.cvtColor(img, to_bgr_codes[pattern])
        return img

    def on_new_image(self, msg):
        if self.original_window_flag == 1 and cv2.getWindowProperty(self.topic, cv2.WND_PROP_VISIBLE) < 1:
            print("Stopping...")
            self.stop_command = True
            return

        img = self.from_message(msg)
        if img is None:
            self.stop_command = True
            return

        self.frame_count += 1
        self.timestamps.append(time.monotonic())
        if len(self.timestamps) >= 2:
            elapsed = self.timestamps[-1] - self.timestamps[0]
            fps = (len(self.timestamps) - 1) / elapsed
            print(f"\rFrame # {self.frame_count}  {fps:.1f} fps", end="", flush=True)
        else:
            print(f"\rFrame # {self.frame_count}", end="", flush=True)

        # Downsize the image before viewing
        cv2.resizeWindow(self.topic, 640, 480)
        cv2.imshow(self.topic, img)
        cv2.waitKey(1)


def main(args=None):
    if len(sys.argv) == 1:
        print(
            "Usage:\n\n    ros2 run image_viewer image_viewer topic\n\nYou can run `ros2 topic list` to see a list of active topics.")
        return

    topic = sys.argv[1]
    rclpy.init(args=args)
    subscriber = Ros2ImageViewer(topic)

    while rclpy.ok() and not subscriber.stop_command:
        rclpy.spin_once(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
