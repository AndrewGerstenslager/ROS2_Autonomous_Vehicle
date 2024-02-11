#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <filesystem>

class CameraImageProcessor : public rclcpp::Node {
public:
    CameraImageProcessor() : Node("camera_image_processor") {
        this->declare_parameter<std::string>("camera_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("save_file_path", "./ipm_matrix.txt");

        this->get_parameter("camera_topic", camera_topic_);
        this->get_parameter("save_file_path", save_file_path_);

        image_transport::ImageTransport it(shared_from_this());
        image_sub_ = it.subscribe(camera_topic_, 1, 
                                  std::bind(&CameraImageProcessor::imageCallback, this, std::placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        try {
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            // Process the image, find chessboard, apply IPM, etc.
            cv::Mat processed_image = processImage(image);
            if (!processed_image.empty()) {
                cv::imshow("Processed Image", processed_image);
                char key = cv::waitKey(0);
                if (key == 'y' || key == 'Y') {
                    saveIPMMatrix(save_file_path_, ipm_matrix_);
                }
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    cv::Mat processImage(cv::Mat& image) {
        // Your image processing logic here
        // For example, finding chessboard corners, applying IPM transformation, etc.
        cv::Mat processed_image;
        // Placeholder for processing logic
        return processed_image;
    }

    void saveIPMMatrix(const std::string& file_path, const cv::Mat& matrix) {
        // Save the IPM matrix to file
    }

    std::string camera_topic_;
    std::string save_file_path_;
    cv::Mat ipm_matrix_;
    image_transport::Subscriber image_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
