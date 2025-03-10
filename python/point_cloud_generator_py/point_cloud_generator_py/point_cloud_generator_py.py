import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField

from hammerhead_msgs.msg import PointCloudSoup


class PointCloudGeneratorNode(Node):
    def __init__(self):
        super().__init__('point_cloud_generator_node')
        self.logger = self.get_logger()
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.subscription = self.create_subscription(
            PointCloudSoup,
            'nodar/point_cloud_soup',
            self.on_new_message,
            qos_profile
        )
        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            'nodar/point_cloud',
            qos_profile
        )
        self.disparity_to_depth4x4 = np.eye(4, dtype=np.float32)
        self.border = 8
        self.z_min = 8.0
        self.z_max = 500.0
        self.y_min = -50.0
        self.y_max = 50.0

    def from_message(self, msg, img):
        if img.shape[0] != msg.height or img.shape[1] != msg.width:
            self.logger.info(
                f"Cached image is the wrong size. Resizing to {msg.width}x{msg.height} with the type {msg.encoding}")
            if msg.encoding == 'bgr8':
                img = np.zeros((msg.height, msg.width, 3), dtype=np.uint8)
            elif msg.encoding == 'mono16':
                img = np.zeros((msg.height, msg.width), dtype=np.int16)
            else:
                self.logger.error(f"Unknown image encoding `{msg.encoding}`")
                return
        img = np.frombuffer(msg.data, dtype=np.uint8 if msg.encoding == 'bgr8' else np.int16).reshape(img.shape)

    def is_valid(self, xyz):
        return not np.isinf(xyz).any()

    def in_range(self, xyz):
        x, y, z = -xyz[0], -xyz[1], -xyz[2]
        return not (np.isinf(x) or np.isinf(y) or np.isinf(
            z) or y < self.y_min or y > self.y_max or z < self.z_min or z > self.z_max)

    def on_new_message(self, msg):
        self.logger.info("onNewMessage")

        num_point_cloud_subs = self.count_subscribers(self.point_cloud_publisher.topic_name)

        if num_point_cloud_subs > 0:
            disparity = np.zeros((msg.disparity.height, msg.disparity.width), dtype=np.uint16)
            rectified = np.zeros((msg.rectified.height, msg.rectified.width, 3), dtype=np.uint8)

            self.from_message(msg.disparity, disparity)
            self.from_message(msg.rectified, rectified)

            self.disparity_to_depth4x4 = np.array(msg.disparity_to_depth4x4.data).reshape(4, 4)

            point_cloud = PointCloud2()
            point_cloud.header.frame_id = "map"
            point_cloud.height = msg.rectified.height
            point_cloud.width = msg.rectified.width
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
            ]
            point_cloud.fields = fields
            point_cloud.is_bigendian = False
            point_cloud.point_step = 16
            point_cloud.row_step = point_cloud.point_step * point_cloud.width
            point_cloud.is_dense = False

            disparity_scaled = disparity.astype(np.float32) / 16.0
            depth3d = cv2.reprojectImageTo3D(disparity_scaled, self.disparity_to_depth4x4)

            xyz = depth3d.reshape(-1, 3)
            bgr = rectified.reshape(-1, 3)
            rows, cols = disparity.shape
            min_row, max_row = self.border, rows - 1 - self.border
            min_col, max_col = self.border, cols - 1 - self.border
            total = rows * cols
            in_range = 0
            valid = 0
            downsample = 1
            num_points = 0
            points = []

            for row in range(rows):
                for col in range(cols):
                    idx = row * cols + col
                    if min_row < row < max_row and min_col < col < max_col and self.is_valid(xyz[idx]):
                        valid += 1
                        if self.in_range(xyz[idx]):
                            in_range += 1
                            if (in_range % downsample) == 0:
                                num_points += 1
                                pt = [
                                    -xyz[idx][0], xyz[idx][1], xyz[idx][2],
                                    int(bgr[idx][0]), int(bgr[idx][1]), int(bgr[idx][2]), 0
                                ]
                                points.extend(pt)

            point_cloud.data = np.array(points, dtype=np.float32).tobytes()
            self.logger.info(f"{num_points} / {total} number of points used")
            self.logger.info(f"{in_range} / {total} in_range points")
            self.logger.info(f"{valid} / {total} valid points")
            self.point_cloud_publisher.publish(point_cloud)
        else:
            self.logger.info("Point Cloud not subscribed")


def main(args=None):
    rclpy.init(args=args)

    exec = rclpy.executors.MultiThreadedExecutor()
    point_cloud_generator_node = PointCloudGeneratorNode()
    exec.add_node(point_cloud_generator_node)

    exec.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
