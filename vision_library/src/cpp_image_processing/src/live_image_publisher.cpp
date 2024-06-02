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
        std::string device_path = this->declare_parameter("device_path", "/dev/video0");
        std::string topic = this->declare_parameter("published_topic", "image");
        
        cap_ = std::make_unique<cv::VideoCapture>(device_path, cv::CAP_V4L);

        if (!cap_->isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open video device: %s", device_path.c_str());
            return;
        }

        cap_->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap_->set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap_->set(cv::CAP_PROP_FRAME_HEIGHT, 720);

        if (cap_->get(cv::CAP_PROP_FRAME_WIDTH) != 1280 || cap_->get(cv::CAP_PROP_FRAME_HEIGHT) != 720) {
            RCLCPP_WARN(this->get_logger(), "Unable to set resolution to 1280x720 on %s, using default resolution", device_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Resolution set to 1280x720 on %s", device_path.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Successfully opened video device: %s", device_path.c_str());

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic, 5);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&CameraPublisherNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (cap_->read(frame) && !frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Captured frame dimensions: %dx%d", frame.cols, frame.rows);
            
            // Adjust exposure based on the frame brightness
            double average_brightness = cv::mean(frame)[0]; // Compute the mean brightness
            adjust_exposure(average_brightness);
            
            // Apply gamma correction
            apply_gamma_correction(frame, 0.5);

            // Reduce to 640x320
            cv::Mat smaller_frame;
            cv::resize(frame, smaller_frame, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", smaller_frame).toImageMsg();
            pub_->publish(*msg.get());

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture frame or frame is empty");
        }
    }

    void adjust_exposure(double brightness)
    {
        double target_brightness = 128.0; // Target brightness level (mid-range)
        double current_exposure = cap_->get(cv::CAP_PROP_EXPOSURE);
        double exposure_adjustment = 0.1; // Adjust this value based on how responsive you want the adjustment to be

        if (brightness < target_brightness - 10) {
            current_exposure -= exposure_adjustment; // Increase exposure if frame is too dark
        } else if (brightness > target_brightness + 10) {
            current_exposure += exposure_adjustment; // Decrease exposure if frame is too bright
        }

        cap_->set(cv::CAP_PROP_EXPOSURE, current_exposure);
        RCLCPP_INFO(this->get_logger(), "Adjusted exposure to %f based on brightness %f", current_exposure, brightness);
    }

    void apply_gamma_correction(cv::Mat &image, double gamma)
    {
        CV_Assert(gamma >= 0);
        cv::Mat lookUpTable(1, 256, CV_8U);
        uchar* p = lookUpTable.ptr();
        for (int i = 0; i < 256; ++i)
            p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
        cv::LUT(image, lookUpTable, image);
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
