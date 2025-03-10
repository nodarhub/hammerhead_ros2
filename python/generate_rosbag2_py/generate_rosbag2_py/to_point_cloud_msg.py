import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def to_point_cloud_msg(details,
                       disparity,
                       rectified,
                       border=8,
                       y_min=-50.0,
                       y_max=50.0,
                       z_min=8.0,
                       z_max=500.0):
    disparity_scaled = disparity.astype(np.float32) / 16.0
    depth3d = cv2.reprojectImageTo3D(disparity_scaled, details.disparity_to_depth4x4)

    xyz = depth3d[border:-border, border:-border, :]
    bgr = rectified[border:-border, border:-border, :]
    valid = ~np.isinf(xyz).all(axis=2)

    y = -xyz[:, :, 1]
    z = -xyz[:, :, 2]
    in_range = valid & (y >= y_min) & (y <= y_max) & (z >= z_min) & (z <= z_max)
    xyz = xyz[in_range]
    bgr = bgr[in_range]

    downsample = 2
    xyz = xyz[::downsample, :]
    bgr = bgr[::downsample, :]

    point_cloud = PointCloud2()
    point_cloud.header.frame_id = "map"
    point_cloud.height = 1
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
    ]
    point_cloud.fields = fields
    point_cloud.is_bigendian = False
    point_cloud.point_step = 16
    point_cloud.is_dense = True

    point_cloud.width = xyz.shape[0]
    point_cloud.row_step = point_cloud.point_step * point_cloud.width

    # TODO: This line is the bottleneck for large matrices.
    # I tried to optimistically oversize this in the constructor,
    # (in which case the frombuffer call below needs the arg: count=point_cloud.width).
    # That is very fast, but then the receiver complains about the size of the data
    # not matching what is expected.
    # I also tried something like creating a one time allocation data_buffer
    # and then here doing point_cloud.data = memoryview(data_buffer)[:point_cloud.row_step]
    # but that is just as slow.
    # If we remedy this, then the downsample isn't as critical.
    point_cloud.data = bytearray(point_cloud.row_step)

    # Combine xyz and rgb into a single array
    points = np.frombuffer(point_cloud.data, dtype=[
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

    total = disparity.size
    print(f"{point_cloud.width} / {total} number of points used")
    print(f"{np.sum(in_range)} / {total} in_range points")
    print(f"{np.sum(valid)} / {total} valid points")
    return point_cloud
