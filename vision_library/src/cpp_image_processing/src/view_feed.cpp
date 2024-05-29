#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraDisplayNode : public rclcpp::Node
{
public:
  CameraDisplayNode() : Node("camera_display_node")
  {
    this->declare_parameter<std::string>("camera_topic", "/camera/image_raw");
    this->get_parameter("camera_topic", camera_topic_);

    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", camera_topic_.c_str());

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, 10, std::bind(&CameraDisplayNode::image_callback, this, std::placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::imshow("Camera Feed", cv_ptr->image);
    cv::waitKey(10);
  }

  std::string camera_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDisplayNode>());
  rclcpp::shutdown();
  return 0;
}
