#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

namespace fs = std::filesystem;

static const std::string DEFAULT_TOPIC = "/nodar/topbots";
static const double DEFAULT_FPS = 10.0;

void signalHandler(int signum) {
    std::cerr << "SIGINT or SIGTERM received." << std::endl;
    std::exit(EXIT_FAILURE);
}

std::string detectEncoding(const cv::Mat& img) {
    int channels = img.channels();
    int depth = img.depth();

    if (channels == 1) {
        if (depth == CV_8U) return "mono8";
        if (depth == CV_16U) return "mono16";
    } else if (channels == 3) {
        if (depth == CV_8U) return "bgr8";
        if (depth == CV_16U) return "bgr16";
    } else if (channels == 4) {
        if (depth == CV_8U) return "bgra8";
        if (depth == CV_16U) return "bgra16";
    }

    throw std::runtime_error("Unsupported image format: channels=" + std::to_string(channels) +
                             " depth=" + std::to_string(depth));
}

std::vector<std::string> collectImages(const std::string& dir) {
    std::vector<std::string> files;

    for (const auto& entry : fs::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;
        auto ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".tiff" || ext == ".tif" || ext == ".png") {
            files.push_back(entry.path().string());
        }
    }

    std::sort(files.begin(), files.end());
    return files;
}

void printUsage(const char* prog) {
    std::cout << "Usage:\n\n"
              << "    ros2 run topbot_publisher topbot_publisher [image_dir] [options]\n\n"
              << "Options:\n"
              << "    --topic <topic>      ROS2 topic name (default: " << DEFAULT_TOPIC << ")\n"
              << "    --fps <fps>          Publish rate in fps (default: " << DEFAULT_FPS << ")\n"
              << "    --encoding <enc>     Image encoding: auto, mono8, mono16, bgr8, etc. (default: auto)\n"
              << "    --loop               Loop over images continuously\n"
              << "    --help               Show this help message\n"
              << std::endl;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Parse arguments
    std::string image_dir = ".";
    std::string topic = DEFAULT_TOPIC;
    double fps = DEFAULT_FPS;
    std::string encoding = "auto";
    bool loop = false;

    // Collect non-ROS args (ROS may inject its own)
    std::vector<std::string> args;
    for (int i = 1; i < argc; i++) {
        args.push_back(argv[i]);
    }

    for (size_t i = 0; i < args.size(); i++) {
        if (args[i] == "--help" || args[i] == "-h") {
            printUsage(argv[0]);
            return EXIT_SUCCESS;
        } else if (args[i] == "--topic" && i + 1 < args.size()) {
            topic = args[++i];
        } else if (args[i] == "--fps" && i + 1 < args.size()) {
            fps = std::stod(args[++i]);
        } else if (args[i] == "--encoding" && i + 1 < args.size()) {
            encoding = args[++i];
        } else if (args[i] == "--loop") {
            loop = true;
        } else if (args[i][0] != '-') {
            image_dir = args[i];
        }
    }

    // Resolve to absolute path
    image_dir = fs::absolute(image_dir).string();

    // Collect image files
    auto files = collectImages(image_dir);
    if (files.empty()) {
        std::cout << "No images found in " << image_dir << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Found " << files.size() << " images in " << image_dir << std::endl;
    std::cout << "Publishing on topic: " << topic << " at " << fps << " fps" << std::endl;

    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("image_publisher");

    // Sensor data QoS: BEST_EFFORT, KEEP_LAST, depth=1
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history,
                                                       rmw_qos_profile_sensor_data.depth),
                            rmw_qos_profile_sensor_data);
    auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic, qos);

    double period_s = 1.0 / fps;
    auto period = std::chrono::duration<double>(period_s);

    // Detect encoding from first image if auto
    if (encoding == "auto") {
        cv::Mat first_img = cv::imread(files[0], cv::IMREAD_UNCHANGED);
        if (first_img.empty()) {
            std::cerr << "Failed to read first image: " << files[0] << std::endl;
            return EXIT_FAILURE;
        }
        encoding = detectEncoding(first_img);
        std::cout << "Auto-detected encoding: " << encoding << std::endl;
    }

    using Clock = std::chrono::steady_clock;
    auto next_publish_time = Clock::now();
    int frame_count = 0;

    bool run = true;
    while (run) {
        run = loop;
        for (const auto& f : files) {
            auto t0 = Clock::now();
            cv::Mat img = cv::imread(f, cv::IMREAD_UNCHANGED);
            auto t1 = Clock::now();

            if (img.empty()) {
                std::cout << "Failed to read " << f << ", skipping" << std::endl;
                continue;
            }

            auto t2 = Clock::now();
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, img).toImageMsg();
            msg->header.stamp = node->get_clock()->now();
            msg->header.frame_id = "camera";
            auto t3 = Clock::now();

            auto remaining = next_publish_time - Clock::now();
            if (remaining.count() > 0) {
                std::this_thread::sleep_for(remaining);
            }

            auto t4 = Clock::now();
            pub->publish(*msg);
            auto t5 = Clock::now();
            next_publish_time += std::chrono::duration_cast<Clock::duration>(period);
            frame_count++;

            auto ms = [](auto d) { return std::chrono::duration<double>(d).count(); };
            std::cout << "[" << frame_count << "]"
                      << " imread=" << std::fixed << std::setprecision(3) << ms(t1 - t0) << "s"
                      << "  cv2msg=" << ms(t3 - t2) << "s"
                      << "  publish=" << ms(t5 - t4) << "s"
                      << "  total=" << ms(t5 - t0) << "s"
                      << std::endl;
        }
    }

    std::cout << "Done" << std::endl;
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
