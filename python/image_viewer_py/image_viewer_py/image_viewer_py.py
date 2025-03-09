import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class Ros2ImageViewer(Node):
    def __init__(self, topic):
        super().__init__('ros2_image_viewer')
        self.topic = topic
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
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
        self.image = None
        self.stop_command = False

    def on_new_image(self, msg):
        if self.original_window_flag == 1 and cv2.getWindowProperty(self.topic, cv2.WND_PROP_VISIBLE) < 1:
            print("Stopping...")
            self.stop_command = True
            return

        if self.image is None or self.image.shape[0] != msg.height or self.image.shape[1] != msg.width:
            print(f"Cached image is the wrong size. Resizing to {msg.width}x{msg.height} with the type {msg.encoding}")
            if msg.encoding == 'bgr8':
                self.image = np.zeros((msg.height, msg.width, 3), dtype=np.uint8)
            elif msg.encoding == 'mono16':
                self.image = np.zeros((msg.height, msg.width), dtype=np.uint16)
            else:
                print(f"Unknown image encoding `{msg.encoding}`")
                self.stop_command = True
                return

        self.image.data = msg.data
        cv2.resizeWindow(self.topic, 640, 480)
        cv2.imshow(self.topic, self.image)
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
