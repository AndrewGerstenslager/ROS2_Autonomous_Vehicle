#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class CameraPublisherNode : public rclcpp::Node
{
public:
    CameraPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("camera_publisher_node", options)
    {
        std::string device_path = this->declare_parameter("device_path", "/dev/video0"); // Default path added
        std::string topic = this->declare_parameter("published_topic", "image");

        cap_ = std::make_unique<cv::VideoCapture>(device_path, cv::CAP_V4L);

        // Check if video capture has been opened correctly
        if (!cap_->isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open video device: %s", device_path.c_str());
            return;
        }

        // Set video format to MJPG
        cap_->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        // Set resolution to 1920x1080
        cap_->set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap_->set(cv::CAP_PROP_FRAME_HEIGHT, 720);

        // Verify settings
        if (cap_->get(cv::CAP_PROP_FRAME_WIDTH) != 1920 || cap_->get(cv::CAP_PROP_FRAME_HEIGHT) != 1080) {
            RCLCPP_WARN(this->get_logger(), "Unable to set resolution to 1920x1080 on %s, using default resolution", device_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Resolution set to 1920x1080 on %s", device_path.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Successfully opened video device: %s", device_path.c_str());

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic, 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(250), 
            std::bind(&CameraPublisherNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (cap_->read(frame) && !frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Captured frame dimensions: %dx%d", frame.cols, frame.rows);
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            pub_->publish(*msg.get());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture frame or frame is empty");
        }
    }

    std::unique_ptr<cv::VideoCapture> cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
