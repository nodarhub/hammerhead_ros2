#pragma once

#include <filesystem>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

inline auto safeLoad(const std::filesystem::path &filename, int read_mode, int pixel_type,
                     const std::filesystem::path &reference_exr, const char *image_type) {
    cv::Mat img;
    if (not std::filesystem::exists(filename)) {
        std::cerr << "Could not find the corresponding " << image_type << " for\n"
                  << reference_exr << ". This path does not exist:\n"
                  << filename << std::endl;
    } else {
        try {
            img = cv::imread(filename, read_mode);
            if (img.type() != pixel_type or img.empty()) {
                std::cerr << "Error loading " << filename << ". "
                          << "The " << image_type << " pixels are not the expected type. Skipping." << std::endl;
                img = cv::Mat();
            }
        } catch (...) {
            std::cerr << "Error loading " << filename << std::endl;
        }
    }
    return img;
}