import os

import cv2


def safe_load(filename, read_mode, dtype, reference_exr, image_type):
    if not os.path.exists(filename):
        print(
            f"Could not find the corresponding {image_type} for\n{reference_exr}. This path does not exist:\n{filename}")
        return None
    try:
        img = cv2.imread(filename, read_mode)
        if img is None or img.dtype != dtype:
            print(f"Error loading {filename}. The {image_type} pixels are not the expected type. Skipping.")
            return None
        return img
    except Exception as e:
        print(f"Error loading {filename}: {e}")
        return None
