import yaml
import numpy as np


class DetailsParameters:
    def __init__(self, path_to_yaml: str):
        with open(path_to_yaml, "r") as f:
            data = yaml.safe_load(f)

        self.left_time = int(data["left_time"])
        self.right_time = int(data["right_time"])
        self.exposure = float(data.get("exposure", 0.0))
        self.gain = float(data.get("gain", 0.0))
        self.focal_length = float(data["focal_length"])
        self.baseline = float(data["baseline"])
        self.meters_above_ground = float(data["meters_above_ground"])
        self.projection_type = int(data.get("projection_type", 0))  # 0 = pinhole, 1 = cylindrical

        self.disparity_to_depth4x4 = np.array(data["projection"], dtype=np.float32).reshape((4, 4))
        self.rotation_disparity_to_raw_cam = np.array(
            data["rotation_disparity_to_raw_cam"], dtype=np.float32
        ).reshape((3, 3))
        self.rotation_world_to_raw_cam = np.array(
            data["rotation_world_to_raw_cam"], dtype=np.float32
        ).reshape((3, 3))
        self.rotation_right_rect_to_raw_cam = np.array(
            data.get("rotation_right_rect_to_raw_cam", [1, 0, 0, 0, 1, 0, 0, 0, 1]), dtype=np.float32
        ).reshape((3, 3))

    def __str__(self):
        return (
            "Details:\n"
            f"\tleft_time               : {self.left_time}\n"
            f"\tright_time              : {self.right_time}\n"
            f"\texposure                : {self.exposure:.6f}\n"
            f"\tgain                    : {self.gain:.6f}\n"
            f"\tfocal_length            : {self.focal_length:.6f}\n"
            f"\tbaseline                : {self.baseline:.6f}\n"
            f"\tmeters_above_ground     : {self.meters_above_ground:.6f}\n"
            f"\tprojection_type         : {self.projection_type}\n"
            f"\tdisparity_to_depth4x4   :\n{self.disparity_to_depth4x4}\n"
            f"\trotation_disp_to_raw_cam:\n{self.rotation_disparity_to_raw_cam}\n"
            f"\trotation_world_to_raw_cam:\n{self.rotation_world_to_raw_cam}\n"
            f"\trotation_right_rect_to_raw_cam:\n{self.rotation_right_rect_to_raw_cam}\n"
        )
