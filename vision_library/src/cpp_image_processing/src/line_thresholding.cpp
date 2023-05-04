#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

class ImageProcessor : public rclcpp::Node {
public:
  ImageProcessor() 
    : Node("image_processor") {

    // Retrieve the topic names from the ROS2 parameter server
    this->declare_parameter<std::string>("subscribed_topic", "input_image");
    this->declare_parameter<std::string>("published_topic", "output_image");
    this->declare_parameter<int>("threshold_value", 100);

    std::string subscribed_topic = this->get_parameter("subscribed_topic").as_string();
    std::string published_topic = this->get_parameter("published_topic").as_string();
    thresh_val = this->get_parameter("threshold_value").as_int();
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(published_topic, 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subscribed_topic, 10, bind(&ImageProcessor::image_callback, this, placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert ROS Image message to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    Mat img = cv_ptr->image;

    // Check if the image is loaded successfully
    if (img.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Error loading the image");
      return;
    }

    // Convert the image to grayscale
    Mat img_bw;
    cvtColor(img, img_bw, COLOR_BGR2GRAY);

    // Threshold the image to detect the white line
    Mat img_thr;
    threshold(img_bw, img_thr, thresh_val, 255, THRESH_BINARY);

    // Find contours in the thresholded image
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_thr, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Initialize an empty mask of the same size as the thresholded image
    Mat mask = Mat::zeros(img_thr.size(), img_thr.type());

    // Filter blobs by size and draw them on the mask
    int min_area = 250;
    for (const auto &contour : contours) {
      double area = contourArea(contour);
      if (area >= min_area) {
        drawContours(mask, vector<vector<Point>>(1, contour), -1, Scalar(255), FILLED);
      }
    }

    // Convert the processed OpenCV image to a ROS Image message
    sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(msg->header, "mono8", mask).toImageMsg();

    // Publish the processed image
    publisher_->publish(*processed_msg);
  }
  int thresh_val;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create the ImageProcessor node
  auto node = std::make_shared<ImageProcessor>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

