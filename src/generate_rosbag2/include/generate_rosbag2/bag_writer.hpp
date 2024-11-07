#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

struct Topic {
    std::string name;
    std::string type;
};

struct BagWriter {
    BagWriter(const std::string &output_dir, const std::vector<Topic> &topics) {
        const rosbag2_cpp::StorageOptions storage_options({output_dir, "sqlite3"});
        const rosbag2_cpp::ConverterOptions converter_options(
            {rmw_get_serialization_format(), rmw_get_serialization_format()});
        writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
        writer_->open(storage_options, converter_options);
        for (const auto &topic : topics) {
            writer_->create_topic({topic.name, topic.type, rmw_get_serialization_format(), ""});
        }
    }

    void write(const std::string &topic, const sensor_msgs::msg::PointCloud2 &point_cloud_msg) const {
        static rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
        rclcpp::SerializedMessage msg;
        serializer.serialize_message(&point_cloud_msg, &msg);
        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->serialized_data =
            std::shared_ptr<rcutils_uint8_array_t>(new rcutils_uint8_array_t, [](rcutils_uint8_array_t *msg) {
                auto fini_return = rcutils_uint8_array_fini(msg);
                delete msg;
                if (fini_return != RCUTILS_RET_OK) {
                    printf("Failed to destroy serialized message %s", rcutils_get_error_string().str);
                }
            });
        *bag_message->serialized_data = msg.release_rcl_serialized_message();
        bag_message->topic_name = topic;
        if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
            printf("Error getting current time: %s", rcutils_get_error_string().str);
        }
        writer_->write(bag_message);
    }

    void write(const std::string &topic, const std::shared_ptr<sensor_msgs::msg::Image> image_msg) const {
        static rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
        rclcpp::SerializedMessage msg;
        serializer.serialize_message(image_msg.get(), &msg);
        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->serialized_data =
            std::shared_ptr<rcutils_uint8_array_t>(new rcutils_uint8_array_t, [](rcutils_uint8_array_t *msg) {
                auto fini_return = rcutils_uint8_array_fini(msg);
                delete msg;
                if (fini_return != RCUTILS_RET_OK) {
                    printf("Failed to destroy serialized message %s", rcutils_get_error_string().str);
                }
            });
        *bag_message->serialized_data = msg.release_rcl_serialized_message();
        bag_message->topic_name = topic;
        if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
            printf("Error getting current time: %s", rcutils_get_error_string().str);
        }
        writer_->write(bag_message);
    }

    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
};