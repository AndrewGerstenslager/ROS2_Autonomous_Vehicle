#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <regex>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

class CameraPublisherNode : public rclcpp::Node {
public:
    CameraPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("image_publisher_node", options) {
        this->declare_parameter<std::string>("device_serial", ""); // Serial number of the device
        this->declare_parameter<std::string>("device_path", ""); // Direct path to the device
        this->declare_parameter<std::string>("published_topic", "image"); // Topic name

        std::string device_serial, device_path, topic;
        this->get_parameter("device_serial", device_serial);
        this->get_parameter("device_path", device_path);
        this->get_parameter("published_topic", topic);

        // If device_serial is provided, find the device by serial number
        if (!device_serial.empty()) {
            device_path = findDeviceBySerial(device_serial);
            if (device_path.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No device found with serial %s", device_serial.c_str());
                rclcpp::shutdown();
                return;
            }
        }

        if (device_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Device path is empty.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Using device path: %s", device_path.c_str());

        cap_ = std::make_unique<cv::VideoCapture>(device_path, cv::CAP_V4L);

        if (!cap_->isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open video device %s", device_path.c_str());
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully opened video device %s", device_path.c_str());
        }

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic, 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CameraPublisherNode::timer_callback, this));
    }

private:
    std::string exec(const char* cmd) {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        return result;
    }

    std::string findDeviceBySerial(const std::string& targetSerial) {
        std::string commandListDevices = "v4l2-ctl --list-devices 2>&1"; // Redirect stderr to stdout to capture any errors
        std::string devicesOutput = exec(commandListDevices.c_str());
        std::istringstream deviceStream(devicesOutput);
        std::string line;
        std::regex deviceRegex("/dev/video\\d+");

        while (std::getline(deviceStream, line)) {
            std::smatch deviceMatch;
            if (std::regex_search(line, deviceMatch, deviceRegex)) {
                std::string device = deviceMatch.str();
                std::string commandGetInfo = "v4l2-ctl --device=" + device + " --all 2>&1"; // Again, redirect stderr to stdout
                std::string deviceInfo = exec(commandGetInfo.c_str());
                std::regex serialRegex("Serial Number\\s*:\\s*(\\S+)");
                std::smatch serialMatch;

                if (std::regex_search(deviceInfo, serialMatch, serialRegex)) {
                    if (serialMatch[1].str() == targetSerial) {
                        return device;
                    }
                }
            }
        }

        return ""; // Return an empty string if no device matches
    }

    void timer_callback() {
        cv::Mat frame;
        *cap_ >> frame;

        if (!frame.empty()) {
            auto msg = cv_bridge::CvImage(std
