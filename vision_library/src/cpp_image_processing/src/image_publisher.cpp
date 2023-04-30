#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <experimental/filesystem>
#include "string"

using namespace std::chrono_literals;

class RawImageNode : public rclcpp::Node {
public:
  RawImageNode()
      : Node("opencv_image_publisher"), count_(0), current_image_(image_paths_.begin()) {
    // Declare and retrieve the parameter
    this->declare_parameter("topic_name_pub", "default_topic_pub");
    topic_name_pub_ = this->get_parameter("topic_name_pub").as_string();

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_pub_, 10);
    timer_ = this->create_wall_timer(5000ms, std::bind(&RawImageNode::timer_callback, this));
    
    // Replace "/path/to/your/image/directory" with the actual directory containing the images
    load_image_paths("/home/ubuntu/repo/dokalman/vision_library/src/dokalman_vision/img_training_data/raw_images");
    if (image_paths_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No images found in the specified directory.");
    } else {
      current_image_ = image_paths_.begin();
    }
  }

private:
  void load_image_paths(const std::string &image_directory) {
    if (!std::experimental::filesystem::exists(image_directory)) {
      RCLCPP_ERROR(this->get_logger(), "Image directory does not exist: %s", image_directory.c_str());
      return;
    }

    namespace fs = std::experimental::filesystem;
    for (const auto &entry : fs::directory_iterator(image_directory)) {
      if (entry.path().has_extension()) {
        image_paths_.push_back(entry.path());
      }
    }
  }

  void timer_callback() {
    if (image_paths_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No images to publish.");
      return;
    }

    cv::Mat my_image = cv::imread(current_image_->string(), cv::IMREAD_COLOR);

    if (my_image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read image: %s", current_image_->string().c_str());
    } else {
      msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image).toImageMsg();
      publisher_->publish(*msg_.get());
      RCLCPP_INFO(this->get_logger(), "Image %ld published: %s", count_, current_image_->string().c_str());
      count_++;
    }

    ++current_image_;
    if (current_image_ == image_paths_.end()) {
      current_image_ = image_paths_.begin();
    }
  }

  std::string topic_name_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
  std::vector<std::experimental::filesystem::path> image_paths_;
  std::vector<std::experimental::filesystem::path>::iterator current_image_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RawImageNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
