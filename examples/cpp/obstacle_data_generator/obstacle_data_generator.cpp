#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sstream>

#include "hammerhead_msgs/msg/obstacle_data.hpp"

void signalHandler(int signum) {
    std::cerr << "SIGINT or SIGTERM received." << std::endl;
    std::exit(EXIT_FAILURE);
}

inline void write_data(const std::string& filename, const hammerhead_msgs::msg::ObstacleData::SharedPtr& obstacleData) {
    std::ofstream out{filename};
    out << "x1," << "z1," << "x2," << "z2," << "x3," << "z3," << "x4," << "z4," << "vx," << "vz" << "\n";
    for (const auto& obstacle : obstacleData->obstacles) {
        for (const auto& p : obstacle.bounding_box) {
            out << std::fixed << std::setprecision(6) << p.x << "," << p.z << ",";
        }
        out << obstacle.velocity.x << "," << obstacle.velocity.z << '\n';
    }
    out << std::endl;
}

class ObstacleDataRecorderNode : public rclcpp::Node {
public:
    using ObstacleData = hammerhead_msgs::msg::ObstacleData;
    rclcpp::Logger logger;

    ObstacleDataRecorderNode(std::filesystem::path output_dir)
        : Node("obstacle_data_recorder_node"), logger(get_logger()), output_dir(output_dir) {
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
        subscription = this->create_subscription<ObstacleData>(
            "nodar/obstacle_data", qos, [=](const ObstacleData::SharedPtr msg) { this->onNewMessage(msg); });
    }

private:
    void onNewMessage(const ObstacleData::SharedPtr msg) {
        static size_t frame_index = 0;
        ++frame_index;

        const auto filename{output_dir / (std::to_string(frame_index) + ".txt")};

        std::stringstream ss;
        ss << "onNewMessage: Received " << msg->obstacles.size() << " obstacles at ";
        ss << std::fixed << std::setprecision(9) << ((rclcpp::Time)msg->header.stamp).seconds();
        ss << ". Writing " << filename << "\n";
        RCLCPP_INFO(logger, ss.str().c_str());

        write_data(filename, msg);
    }

    std::filesystem::path output_dir;
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    rclcpp::Subscription<ObstacleData>::SharedPtr subscription;
};

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "Starting obstacle_data_recorder_node" << std::endl;
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;

    const auto HERE{std::filesystem::path(__FILE__).parent_path()};
    const auto output_dir{HERE / "obstacle_data"};
    std::filesystem::create_directories(output_dir);

    auto obstacle_data_recorder_node = std::make_shared<ObstacleDataRecorderNode>(output_dir);
    exec.add_node(obstacle_data_recorder_node);

    exec.spin();

    rclcpp::shutdown();

    return 0;
}