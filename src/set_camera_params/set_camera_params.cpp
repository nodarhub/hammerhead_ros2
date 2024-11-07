#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "hammerhead_msgs/srv/camera_param.hpp"

#ifdef TOPIC_exposure
static constexpr auto TOPIC = "nodar/set_exposure";
#elif TOPIC_gain
static constexpr auto TOPIC = "nodar/set_gain";
#endif

void signalHandler(int signum) {
    std::cerr << "SIGINT or SIGTERM received." << std::endl;
    std::exit(EXIT_FAILURE);
}

class ClientNode {
public:
    using CameraParam = hammerhead_msgs::srv::CameraParam;
    ClientNode(const std::string &node_name) { node = rclcpp::Node::make_shared(init(node_name)); }
    ~ClientNode() { rclcpp::shutdown(); }

    void sendRequest(const std::string &topic, float val) {
        auto result = clients.find(topic);
        decltype(result->second) client;
        if (result == clients.end()) {
            client = node->create_client<CameraParam>(topic);
            clients.insert({topic, client});
        } else {
            client = result->second;
        }
        if (not client->wait_for_service(std::chrono::seconds(1))) {
            std::cerr << "There does not appear to be a service for the topic: `" << topic << "`\n"
                      << "Please check the spelling, and check that the service is actually up by running\n\n"
                      << "    ros2 service list\n"
                      << std::endl;
            return;
        }
        auto request = std::make_shared<CameraParam::Request>();
        request->val = val;
        auto response = client->async_send_request(request);

        // Wait for the result
        std::cout << "Waiting for a response..." << std::endl;
        if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), response) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            std::cout << "Client" << std::endl;
            std::cout << "    request->val      : " << request->val << std::endl;
            std::cout << "    response->success : " << response.get()->success << std::endl;
        } else {
            std::cerr << "Failed to call service" << std::endl;
        }
    }

private:
    static std::string init(const std::string &node_name) {
        rclcpp::init(0, nullptr);
        return node_name;
    }

    std::shared_ptr<rclcpp::Node> node;
    std::unordered_map<std::string, rclcpp::Client<CameraParam>::SharedPtr> clients;
};

int main(int argc, char **argv) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    std::cout << TOPIC << "\n\n--------------------\n"
              << "To set a parameter, just input the desired value, and press ENTER.\n"
              << "--------------------\n";
    auto client_node = std::make_shared<ClientNode>("client_node");
    while (true) {
        int val;
        if (std::cin >> val) {
            std::cout << "Requesting " << TOPIC << " = " << val << std::endl;
            client_node->sendRequest(TOPIC, val);
        } else {
            std::cerr << "Could not parse your last input. Try entering that again. " << std::endl;
        }
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return 0;
}