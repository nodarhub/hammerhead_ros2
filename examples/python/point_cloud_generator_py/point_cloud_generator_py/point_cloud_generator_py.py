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
            return False

        # Allocate space for the output
        chnls = self.img.shape[2] if self.img is not None and self.img.ndim == 3 else 1
        if (self.img is None
                or self.img.shape[0] != msg.height or self.img.shape[1] != msg.width
                or chnls != channels or self.img.dtype != dtype):
            print(
                f"Cached image is the wrong size. Changing to {msg.width}x{msg.height} with the type {msg.encoding}, that is, channels = {chnls}, dtype = {dtype}")
            if channels != 1:
                self.img = np.zeros((msg.height, msg.width, channels), dtype=dtype)
            else:
                self.img = np.zeros((msg.height, msg.width), dtype=dtype)

        # Copy in the message data
        self.img.data = msg.data

        # If the encoding is a Bayer pattern, convert to BGR
        if msg.encoding == "bayer_bggr8" or msg.encoding == "bayer_bggr16":
            print("Converting Bayer to BGR")
            self.img = cv2.cvtColor(self.img, cv2.COLOR_BayerRG2BGR)
        elif msg.encoding == "bayer_rggb8" or msg.encoding == "bayer_rggb16":
            print("Converting Bayer to BGR")
            self.img = cv2.cvtColor(self.img, cv2.COLOR_BayerBG2BGR)
        return True

    def on_new_message(self, msg):
        self.logger.info("onNewMessage")

        num_point_cloud_subs = self.count_subscribers(self.point_cloud_publisher.topic_name)

        if num_point_cloud_subs >= 0:
            rectified = np.zeros((msg.rectified.height, msg.rectified.width, 3), dtype=np.uint8)

            self.img = self.disparity
            if not self.from_message(msg.disparity):
                self.stop_command = True
                return
            self.disparity = self.img
            self.img = self.rectified
            if not self.from_message(msg.rectified):
                self.stop_command = True
                return
            self.rectified = self.img

            self.disparity_to_depth4x4 = np.array(msg.disparity_to_depth4x4.data).reshape(4, 4)
            print("Details:\n" +
                  f"\tfocal_length : {msg.focal_length}\n" +
                  f"\tbaseline     : {msg.baseline}\n"
                  )

            disparity_scaled = self.disparity.astype(np.float32) / 16.0
            depth3d = cv2.reprojectImageTo3D(disparity_scaled, self.disparity_to_depth4x4)

            xyz = depth3d[self.border:-self.border, self.border:-self.border, :]
            bgr = self.rectified[self.border:-self.border, self.border:-self.border, :]
            valid = ~np.isinf(xyz).all(axis=2)

            x = -xyz[:, :, 0]
            y = -xyz[:, :, 1]
            z = -xyz[:, :, 2]

            # Filter out inf values
            x_filtered = x[~np.isinf(x)]
            y_filtered = y[~np.isinf(y)]
            z_filtered = z[~np.isinf(z)]

            # Calculate min and max values for x, y, and z
            min_x, max_x = np.min(x_filtered), np.max(x_filtered)
            min_y, max_y = np.min(y_filtered), np.max(y_filtered)
            min_z, max_z = np.min(z_filtered), np.max(z_filtered)

            # Print the results
            print(f"Min x: {min_x}, Max x: {max_x}")
            print(f"Min y: {min_y}, Max y: {max_y}")
            print(f"Min z: {min_z}, Max z: {max_z}")

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
