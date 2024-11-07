#include <iostream>
#include <opencv2/calib3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "hammerhead_msgs/msg/point_cloud_soup.hpp"

void signalHandler(int signum) {
    std::cerr << "SIGINT or SIGTERM received." << std::endl;
    std::exit(EXIT_FAILURE);
}

class PointCloudGeneratorNode : public rclcpp::Node {
public:
    using Image = sensor_msgs::msg::Image;
    using Msg = hammerhead_msgs::msg::PointCloudSoup;
    using PointCloud = sensor_msgs::msg::PointCloud2;
    rclcpp::Logger logger;

    PointCloudGeneratorNode() : Node("point_cloud_generator_node"), logger(get_logger()) {
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        // subscription = this->create_subscription<Msg>("nodar/point_cloud_soup", 1,
        //                                               [=](const Msg::SharedPtr msg) { this->onNewMessage(msg); });
        // point_cloud_publisher = this->create_publisher<PointCloud>("nodar/point_cloud", 10);
        subscription = this->create_subscription<Msg>("nodar/point_cloud_soup", qos,
                                                      [=](const Msg::SharedPtr msg) { this->onNewMessage(msg); });
        point_cloud_publisher = this->create_publisher<PointCloud>("nodar/point_cloud", qos);
        disparity_to_depth4x4 = cv::Mat(4, 4, CV_32FC1);
    }

private:
    void fromMessage(const Image& msg, cv::Mat& img) {
        if (img.rows != msg.height or img.cols != msg.width) {
            std::cout << "Cached image is the wrong size. Resizing to " << msg.width << "x" << msg.height
                      << " with the type " << msg.encoding << std::endl;
            if (msg.encoding == "bgr8") {
                img = cv::Mat(msg.height, msg.width, CV_8UC3);
            } else if (msg.encoding == "mono16") {
                img = cv::Mat(msg.height, msg.width, CV_16SC1);
            } else {
                std::cerr << "Unknown image encoding `" << msg.encoding << "`\n";
                return;
            }
        }
        const auto size = msg.height * msg.step;
        std::copy(msg.data.data(), msg.data.data() + size, img.data);
    }

    bool isValid(const float* const xyz) {
        return not std::isinf(xyz[0]) and not std::isinf(xyz[1]) and not std::isinf(xyz[2]);
    }

    bool inRange(const float* const xyz) {
        const auto x = -xyz[0];
        const auto y = -xyz[1];
        const auto z = -xyz[2];
        return not(std::isinf(x)  //
                   or std::isinf(y) or y < y_min or y > y_max  //
                   or std::isinf(z) or z < z_min or z > z_max);
    }

    void onNewMessage(const Msg::SharedPtr msg) {
        RCLCPP_INFO(logger, "onNewMessage");

        size_t num_point_cloud_subs = 0;
        num_point_cloud_subs = count_subscribers(point_cloud_publisher->get_topic_name());

        if (num_point_cloud_subs > 0) {
            // Retrieve OpenCV matrices from the message
            fromMessage(msg->disparity, disparity);
            fromMessage(msg->rectified, rectified);

            const auto& data = msg->disparity_to_depth4x4.data();
            std::copy(data, data + 16, disparity_to_depth4x4.begin<float>());

            // Construct the point cloud
            PointCloud point_cloud;
            point_cloud.header.frame_id = "map";
            point_cloud.height = msg->rectified.height;
            point_cloud.width = msg->rectified.width;
            sensor_msgs::PointCloud2Modifier modifier(point_cloud);
            modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
            modifier.resize(point_cloud.height * point_cloud.width);
            sensor_msgs::PointCloud2Iterator<float> x(point_cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> y(point_cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> z(point_cloud, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> r(point_cloud, "r");
            sensor_msgs::PointCloud2Iterator<uint8_t> g(point_cloud, "g");
            sensor_msgs::PointCloud2Iterator<uint8_t> b(point_cloud, "b");

            // Disparity is in 11.6 format
            disparity.convertTo(disparity_scaled, CV_32F, 1. / 16);
            cv::reprojectImageTo3D(disparity_scaled, depth3d, disparity_to_depth4x4);

            auto xyz = reinterpret_cast<float*>(depth3d.data);
            auto bgr = rectified.data;
            const auto rows = disparity.rows;
            const auto cols = disparity.cols;
            const auto min_row = border;
            const auto max_row = rows - 1 - border;
            const auto min_col = border;
            const auto max_col = cols - 1 - border;
            size_t total = 0;
            size_t in_range = 0;
            size_t valid = 0;
            const auto downsample = 1;
            size_t num_points = 0;
            try {
                for (size_t row = 0; row < rows; ++row) {
                    for (size_t col = 0; col < cols; ++col, xyz += 3, bgr += 3) {
                        if (row > min_row and row < max_row and col > min_col and col < max_col and isValid(xyz)) {
                            ++valid;
                            if (inRange(xyz)) {
                                ++in_range;
                                if ((in_range % downsample) == 0) {
                                    ++num_points;
                                    *x = -xyz[0], *y = xyz[1], *z = xyz[2];
                                    *b = bgr[0], *g = bgr[1], *r = bgr[2];
                                    ++x, ++y, ++z, ++r, ++g, ++b;
                                }
                            }
                        }
                        ++total;
                    }
                }
            } catch (...) {
                RCLCPP_ERROR(logger, "Error generating point cloud");
            }
            modifier.resize(num_points);
            if (true) {
                std::cout << num_points << " / " << total << " number of points used" << std::endl;
                std::cout << in_range << " / " << total << " in_range points" << std::endl;
                std::cout << valid << " / " << total << " valid points" << std::endl;
            }
            point_cloud_publisher->publish(point_cloud);
        } else {
            RCLCPP_INFO(get_logger(), "Point Cloud not subscribed");
        }
    }

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    rclcpp::Subscription<Msg>::SharedPtr subscription;
    rclcpp::Publisher<PointCloud>::SharedPtr point_cloud_publisher;
    cv::Mat disparity, rectified, disparity_scaled, depth3d, disparity_to_depth4x4;
    size_t border = 8;
    float z_min = 8.0;
    float z_max = 500.0;
    float y_min = -50.0;
    float y_max = 50.0;
};

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;

    auto point_cloud_generator_node = std::make_shared<PointCloudGeneratorNode>();
    exec.add_node(point_cloud_generator_node);

    exec.spin();

    rclcpp::shutdown();

    return 0;
}