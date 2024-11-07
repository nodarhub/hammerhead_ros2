#pragma once

#include <iostream>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "generate_rosbag2/details.hpp"
#include "generate_rosbag2/point_filter.hpp"

inline auto toPointCloud2Msg(const Details &details, const cv::Mat &depth_image, const cv::Mat &left_rect) {
    // Convert the depth map to a point cloud
    const auto disparity = details.focal_length * details.baseline / depth_image;
    cv::Mat point_cloud;
    cv::reprojectImageTo3D(disparity, point_cloud, details.projection);

    // Remove some area around the border
    const auto border = 8;
    const auto width = std::max(0, point_cloud.rows - 2 * border);
    const auto height = std::max(0, point_cloud.cols - 2 * border);
    cv::Mat point_cloud_trimmed = point_cloud({border, border, height, width});
    cv::Mat left_rect_trimmed = left_rect({border, border, height, width});

    // Create the point cloud message and a modifier to iterate over it
    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    point_cloud_msg.header.stamp = time == 0 ? rclcpp::Clock{}.now() : rclcpp::Time(details.left_time);
    point_cloud_msg.header.frame_id = "map";
    point_cloud_msg.height = point_cloud_trimmed.rows;
    point_cloud_msg.width = point_cloud_trimmed.cols;
    sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(point_cloud_msg.height * point_cloud_msg.width);
    sensor_msgs::PointCloud2Iterator<float> x(point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> y(point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> z(point_cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> r(point_cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> g(point_cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> b(point_cloud_msg, "b");

    // Apply the point filter, and choose how much to downsample.
    // 1 means "no downsampling"
    // 10 reduces the data by a factor of 10
    // Without downsampling, the point cloud can be extremely large
    const auto downsample = 1;
    size_t in_range = 0;
    size_t num_points = 0;
    PointFilter point_filter;
    auto xyz = reinterpret_cast<float *>(point_cloud_trimmed.data);
    auto bgr = left_rect_trimmed.data;
    try {
        for (size_t i = 0; i < point_cloud_trimmed.total(); ++i, xyz += 3, bgr += 3) {
            if (point_filter.isValid(xyz) and point_filter.inRange(xyz)) {
                ++in_range;
                if ((in_range % downsample) == 0) {
                    ++num_points;
                    *x = -xyz[0], *y = xyz[1], *z = xyz[2];
                    *b = bgr[0], *g = bgr[1], *r = bgr[2];
                    ++x, ++y, ++z, ++r, ++g, ++b;
                }
            }
        }
    } catch (...) {
        std::cerr << "Error generating point cloud" << std::endl;
    }
    modifier.resize(num_points);
    return point_cloud_msg;
}

inline std::shared_ptr<sensor_msgs::msg::Image> toImageMsg(const cv::Mat &img, uint64_t time) {
    auto message = std::make_shared<sensor_msgs::msg::Image>();
    message->header.stamp = time == 0 ? rclcpp::Clock{}.now() : rclcpp::Time(time);
    message->is_bigendian = 0;
    message->height = img.rows;
    message->width = img.cols;
    message->step = img.cols * img.elemSize();
    if (img.type() == CV_8UC3) {
        message->encoding = "bgr8";
    } else if (img.type() == CV_16UC1) {
        message->encoding = "mono16";
    } else if (img.type() == CV_32FC1) {
        message->encoding = "32FC1";
    } else {
        std::cerr << "Trying to convert unknown image type" << std::endl;
        return nullptr;
    }
    const auto size = message->height * message->step;
    message->data.resize(size);
    std::copy(img.data, img.data + size, message->data.data());
    return message;
}