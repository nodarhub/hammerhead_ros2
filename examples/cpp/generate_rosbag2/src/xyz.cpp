#include <filesystem>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <vector>

#include "generate_rosbag2/bag_writer.hpp"
#include "generate_rosbag2/details.hpp"
#include "generate_rosbag2/get_files.hpp"
#include "generate_rosbag2/safe_load.hpp"
#include "generate_rosbag2/to_msg.hpp"
#include "tqdm.hpp"

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Expecting at least one argument "
                  << "(the path to the recorded data). Usage:\n\n"
                  << "\tros2 run generate_rosbag2 xyz path_to_data [path_to_output_bag]" << std::endl;
        return EXIT_FAILURE;
    }
    const std::filesystem::path input_dir(argv[1]);
    const std::filesystem::path output_dir = argc > 2 ? argv[2] : (input_dir / "bag");

    // Directories that we read
    const auto disparity_dir = input_dir / "disparity";
    const auto depth_dir = input_dir / "depth";  // Possible alternative
    const auto details_dir = input_dir / "details";
    const auto left_rect_dir = input_dir / "left-rect";

    // Check if `disparity` folder exists
    if (!std::filesystem::exists(disparity_dir)) {
        if (std::filesystem::exists(depth_dir)) {
            std::cerr << "\n[WARNING] No 'disparity' folder found, but 'depth' exists."
                      << "\nThis may indicate you are using an old Hammerhead version.\n"
                      << "\nPlease refer to the following example to convert depth to disparity:\n"
                      << "https://github.com/nodarhub/hammerhead_zmq/tree/main/examples/depth_to_disparity.\n"
                      << std::endl;
            return EXIT_FAILURE;
        } else {
            std::cerr << "[ERROR] Required 'disparity' folder is missing and no 'depth' folder was found."
                      << "\nPlease verify the input path: " << input_dir << "\n"
                      << std::endl;
            return EXIT_FAILURE;
        }
    }

    // Topic names
    std::vector<Topic> topics{{
        Topic{"nodar/point_cloud", "sensor_msgs/msg/PointCloud2"},
    }};

    // Remove old bag output if it exists
    if (std::filesystem::exists(output_dir)) {
        // If you don't want to delete and overwrite old data, then set this bool to true
        if (false) {
            std::cerr << "Something already exists in the directory\n\t" << output_dir
                      << "\nDid you already generate this bag?\n"
                      << "If you want to rerun this tool on\n\t" << input_dir << "\nthen either delete the folder\n\t"
                      << output_dir << "\nor specify a different path_to_output_bag "
                      << "as the second argument.\nFor example:\n\t"
                      << "ros2 run generate_rosbag2 xyz " << input_dir << " path_to_output_bag" << std::endl;
            return EXIT_FAILURE;
        }
        std::filesystem::remove_all(output_dir);
    }
    std::filesystem::create_directories(output_dir.parent_path());

    // Create the bag writer
    BagWriter bag_writer(output_dir, topics);

    // Load the disparity data
    const auto disparities = getFiles(disparity_dir, ".tiff");
    std::cout << "Found " << disparities.size() << " disparity maps to convert to point clouds" << std::endl;
    for (const auto &disparity : tq::tqdm(disparities)) {
        // Safely load all the images.
        auto disparity_image = safeLoad(disparity, cv::IMREAD_ANYDEPTH, CV_16UC1, disparity, "disparity image");
        disparity_image.convertTo(disparity_image, CV_32FC1, -1.0 / 16.0);
        // New versions of the sdk save images as .tiff instead of .png
        // Look for a tiff, and if it doesn't exist, look for a png.
        const auto left_rect_tiff = left_rect_dir / (disparity.stem().string() + ".tiff");
        const auto left_rect_png = left_rect_dir / (disparity.stem().string() + ".png");
        const auto left_rect_filename = std::filesystem::exists(left_rect_tiff) ? left_rect_tiff : left_rect_png;
        const auto left_rect =
            safeLoad(left_rect_filename, cv::IMREAD_COLOR, CV_8UC3, disparity, "left rectified image");
        if (disparity_image.empty() or left_rect.empty()) {
            continue;
        }

        // Load the details
        const auto details_filename = details_dir / (disparity.stem().string() + ".csv");
        if (not std::filesystem::exists(details_filename)) {
            std::cerr << "Could not find the corresponding details for\n"
                      << disparity << ". This path does not exist:\n"
                      << details_filename << std::endl;
        }
        const Details details(details_filename);

        // Write the messages
        bag_writer.write("nodar/point_cloud", toPointCloud2Msg(details, disparity_image, left_rect));
    }
    std::cout << std::endl;
    return 0;
}