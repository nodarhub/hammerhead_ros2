import numpy as np
from builtin_interfaces.msg import Time


def to_image_msg(bridge, image, timestamp):
    if image.ndim == 2:  # single channel image
        if image.dtype == np.uint8:
            encoding = "mono8"
        elif image.dtype == np.uint16:
            encoding = "mono16"
        elif image.dtype == np.float32:
            encoding = "32FC1"
        else:
            raise ValueError("Unsupported single channel image type: {}".format(image.dtype))
    elif image.ndim == 3:
        channels = image.shape[2]
        if channels == 3:
            if image.dtype == np.uint8:
                encoding = "bgr8"
            elif image.dtype == np.uint16:
                encoding = "bgr16"
            else:
                raise ValueError("Unsupported 3-channel image type: {}".format(image.dtype))
        else:
            raise ValueError("Unsupported channel count: {}".format(channels))
    else:
        raise ValueError("Unsupported image shape: {}".format(image.shape))
    img_msg = bridge.cv2_to_imgmsg(image, encoding=encoding)
    ros_time = Time()
    ros_time.sec = timestamp // 1_000_000_000
    ros_time.nanosec = timestamp % 1_000_000_000
    img_msg.header.stamp = ros_time
    return img_msg
