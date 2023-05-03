#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>

class CameraPublisherNode : public rclcpp::Node
{
public:
    CameraPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("image_publisher_node", options)
    {
        this->declare_parameter<std::string>("device_path", "");  // default to empty string
        this->declare_parameter<std::string>("published_topic", "image");  // default to "image"

        std::string device_path;
        this->get_parameter("device_path", device_path);
        
        std::string topic;
        this->get_parameter("published_topic", topic);

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
