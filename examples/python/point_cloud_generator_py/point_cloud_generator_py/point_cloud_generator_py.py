import time

import cv2
import numpy as np
import rclpy
from hammerhead_msgs.msg import PointCloudSoup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField


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

        self.point_cloud = PointCloud2()
        self.point_cloud.header.frame_id = "map"
        self.point_cloud.height = 1
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]
        self.point_cloud.fields = fields
        self.point_cloud.is_bigendian = False
        self.point_cloud.point_step = 16
        self.point_cloud.is_dense = True
        self.img = np.zeros((1, 1), dtype=np.uint8)
        self.disparity = np.zeros((1, 1), dtype=np.uint8)
        self.rectified = np.zeros((1, 1), dtype=np.uint8)
        self.depth3d = None

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
            self.logger.error(f"Unknown image encoding `{msg.encoding}`")
            return None

        img = np.frombuffer(msg.data, dtype=dtype)
        if channels != 1:
            img = img.reshape(msg.height, msg.width, channels)
        else:
            img = img.reshape(msg.height, msg.width)

        # If the encoding is a Bayer pattern, convert to BGR
        if msg.encoding == "bayer_bggr8" or msg.encoding == "bayer_bggr16":
            self.logger.info("Converting Bayer to BGR")
            img = cv2.cvtColor(img, cv2.COLOR_BayerRG2BGR)
        elif msg.encoding == "bayer_rggb8" or msg.encoding == "bayer_rggb16":
            self.logger.info("Converting Bayer to BGR")
            img = cv2.cvtColor(img, cv2.COLOR_BayerBG2BGR)
        return img

    def on_new_message(self, msg):
        self.logger.info("onNewMessage")
        if self.count_subscribers(self.point_cloud_publisher.topic_name) == 0:
            self.logger.info("Nobody is subscribed nodar/point_cloud")
            return

        self.logger.info("msg.disparity.encoding {msg.disparity.encoding}")
        self.logger.info("msg.rectified.encoding {msg.rectified.encoding}")
        self.disparity = self.from_message(msg.disparity)
        if self.disparity is None:
            return
        self.rectified = self.from_message(msg.rectified)
        if self.rectified is None:
            return

        self.disparity_to_depth4x4 = np.array(msg.disparity_to_depth4x4.data).reshape(4, 4)
        self.logger.info("Details:\n" +
                         f"\tfocal_length : {msg.focal_length}\n" +
                         f"\tbaseline     : {msg.baseline}\n"
                         )

        disparity_scaled = self.disparity / np.float32(16)
        if self.depth3d is None:
            self.depth3d = cv2.reprojectImageTo3D(disparity_scaled, self.disparity_to_depth4x4)
        else:
            cv2.reprojectImageTo3D(disparity_scaled, self.disparity_to_depth4x4, self.depth3d)

        xyz = self.depth3d[self.border:-self.border, self.border:-self.border, :]
        bgr = self.rectified[self.border:-self.border, self.border:-self.border, :]

        x = -xyz[:, :, 0]
        y = -xyz[:, :, 1]
        z = -xyz[:, :, 2]
        valid = ~(np.isinf(x) | np.isinf(y) | np.isinf(z))

        in_range = valid & (y >= self.y_min) & (y <= self.y_max) & (z >= self.z_min) & (z <= self.z_max)
        xyz = xyz[in_range]
        bgr = bgr[in_range]

        downsample = 10
        xyz = xyz[::downsample, :]
        bgr = bgr[::downsample, :]

        self.point_cloud.width = xyz.shape[0]
        self.point_cloud.row_step = self.point_cloud.point_step * self.point_cloud.width

        # TODO: This line is the bottleneck for large matrices.
        # I tried to optimistically oversize this in the constructor,
        # (in which case the frombuffer call below needs the arg: count=point_cloud.width).
        # That is very fast, but then the receiver complains about the size of the data
        # not matching what is expected.
        # I also tried something like creating a one time allocation self.data_buffer
        # and then here doing point_cloud.data = memoryview(self.data_buffer)[:self.point_cloud.row_step]
        # but that is just as slow.
        # If we remedy this, then the downsample isn't as critical.
        self.point_cloud.data = bytearray(self.point_cloud.row_step)

        # Combine xyz and rgb into a single array
        points = np.frombuffer(self.point_cloud.data, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
        ])
        points['x'] = xyz[:, 0]
        points['y'] = xyz[:, 1]
        points['z'] = xyz[:, 2]
        points['rgb'] = ((bgr[:, 2].astype(np.uint32) << 16) |
                         (bgr[:, 1].astype(np.uint32) << 8) |
                         (bgr[:, 0].astype(np.uint32)))
        total = self.disparity.size
        self.logger.info(f"{self.point_cloud.width} / {total} number of points used")
        self.logger.info(f"{np.sum(in_range)} / {total} in_range points")
        self.logger.info(f"{np.sum(valid)} / {total} valid points")
        self.point_cloud_publisher.publish(self.point_cloud)


def main(args=None):
    rclpy.init(args=args)

    exec = rclpy.executors.MultiThreadedExecutor()
    point_cloud_generator_node = PointCloudGeneratorNode()
    exec.add_node(point_cloud_generator_node)

    while point_cloud_generator_node.index < 5:
        exec.spin_once()
        time.sleep(.1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
