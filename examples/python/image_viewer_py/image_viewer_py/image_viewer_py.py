import sys

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

    def from_message(self, msg):
        # Get the type
        if msg.encoding == 'bayer_bggr8' or msg.encoding == 'bayer_rggb8' or msg.encoding == 'mono8':
            channels, dtype = 1, np.uint8
        elif msg.encoding == 'bgr8':
            channels, dtype = 3, np.uint8
        elif msg.encoding == 'bgra8':
            channels, dtype = 4, np.uint8
        elif msg.encoding == 'bayer_bggr16' or msg.encoding == 'bayer_rggb16' or msg.encoding == 'mono16':
            channels, dtype = 1, np.uint16
        elif msg.encoding == 'bgr16':
            channels, dtype = 3, np.uint16
        elif msg.encoding == 'bgra16':
            channels, dtype = 4, np.uint16
        else:
            print(f"Unknown image encoding `{msg.encoding}`")
            return None

        if channels != 1:
            img = np.array(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
        else:
            img = np.array(msg.data, dtype=dtype).reshape(msg.height, msg.width)

        # If the encoding is a Bayer pattern, convert to BGR
        if msg.encoding == "bayer_bggr8" or msg.encoding == "bayer_bggr16":
            print("Converting Bayer to BGR")
            img = cv2.cvtColor(img, cv2.COLOR_BayerRG2BGR)
        elif msg.encoding == "bayer_rggb8" or msg.encoding == "bayer_rggb16":
            print("Converting Bayer to BGR")
            img = cv2.cvtColor(img, cv2.COLOR_BayerBG2BGR)
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
