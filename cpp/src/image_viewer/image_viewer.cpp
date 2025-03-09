#include <iostream>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

void signalHandler(int signum) {
    std::cerr << "SIGINT or SIGTERM received." << std::endl;
    std::exit(EXIT_FAILURE);
}

class Ros2ImageViewer : public rclcpp::Node {
public:
    using Msg = sensor_msgs::msg::Image;
    Ros2ImageViewer(const std::string &topic)
        : Node("ros2_image_viewer"), topic(topic), stop_command(stop_promise.get_future()) {
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        std::cout << "Subscribing to " << topic << std::endl;
        subscription =
            this->create_subscription<Msg>(topic, qos, [=](const Msg::SharedPtr msg) { this->onNewImage(msg); });
        cv::namedWindow(topic, cv::WINDOW_NORMAL);
        original_window_flag = cv::getWindowProperty(topic, cv::WND_PROP_VISIBLE);
    }

private:
    void onNewImage(const Msg::SharedPtr msg) {
        if (original_window_flag == 1 and cv::getWindowProperty(topic, cv::WND_PROP_VISIBLE) < 1) {
            std::cout << "Stopping..." << std::endl;
            stop_promise.set_value();
            return;
        }
        if (image.rows != msg->height or image.cols != msg->width) {
            std::cout << "Cached image is the wrong size. Resizing to " << msg->width << "x" << msg->height
                      << " with the type " << msg->encoding << std::endl;
            if (msg->encoding == "bgr8") {
                image = cv::Mat(msg->height, msg->width, CV_8UC3);
            } else if (msg->encoding == "mono16") {
                image = cv::Mat(msg->height, msg->width, CV_16UC1);
            } else {
                std::cerr << "Unknown image encoding `" << msg->encoding << "`\n";
                stop_promise.set_value();
                return;
            }
        }
        const auto size = msg->height * msg->step;
        std::copy(msg->data.data(), msg->data.data() + size, image.data);
        // Downsize the image before viewing
        cv::resizeWindow(topic, {640, 480});
        cv::imshow(topic, image);
        cv::waitKey(1);
    }

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    cv::Mat image;
    std::string topic;
    int original_window_flag;
    std::promise<void> stop_promise;

public:
    std::shared_future<void> stop_command;
};

int main(int argc, char *argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    if (argc == 1) {
        std::cout << "Usage:\n\n"
                  << "    ros2 run image_viewer image_viewer topic\n\n"
                  << "You can run `ros2 topic list` to see a list of active topics." << std::endl;
        return EXIT_SUCCESS;
    }
    const auto topic = argv[1];
    rclcpp::init(argc, argv);
    auto subscriber = std::make_shared<Ros2ImageViewer>(topic);
    rclcpp::spin_until_future_complete(subscriber, subscriber->stop_command);
    rclcpp::shutdown();
}