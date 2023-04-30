#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <regex>

class CameraPublisherNode : public rclcpp::Node
{
public:
    CameraPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("image_publisher_node", options)
    {
        this->declare_parameter<std::string>("serial", "");  // default to empty string
        this->declare_parameter<std::string>("topic", "image");  // default to "image"

        std::string serial;
        this->get_parameter("serial", serial);
        
        std::string topic;
        this->get_parameter("topic", topic);

        std::string device = findDeviceBySerial(serial);

        cap_ = std::make_unique<cv::VideoCapture>(device, cv::CAP_V4L);

        if (!cap_->isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open video device %s", device.c_str());
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully opened video device %s", device.c_str());
        }


        pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic, 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CameraPublisherNode::timer_callback, this));
    }

private:
    std::string findDeviceBySerial(const std::string& targetSerial)
    {
        std::string device = "";
        std::regex serialRegex("Serial\\s+:\\s+(\\w+)");
        std::smatch serialMatch;

        for (int i = 0; i < 10; ++i) {
            std::string dev = "/dev/video" + std::to_string(i);
            std::string command = "v4l2-ctl --device=" + dev + " --all";
            std::string output = execCommand(command.c_str());

            if (std::regex_search(output, serialMatch, serialRegex) && serialMatch[1] == targetSerial) {
                device = dev;
                break;
            }
        }

        if (device.empty()) {
            throw std::runtime_error("No device found with serial " + targetSerial);
        }

        return device;
    }

    std::string execCommand(const char* cmd) {
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

    void timer_callback()
    {
        cv::Mat frame;
        *cap_ >> frame;

        if (!frame.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            pub_->publish(*msg);
        }
    }

    std::unique_ptr<cv::VideoCapture> cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}