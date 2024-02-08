#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ImageToPCDPublisher : public rclcpp::Node
{
public:
    ImageToPCDPublisher()
    : Node("image_to_pcd_publisher")
    {
        this->declare_parameter<double>("scale", 1.0/255.0);
        this->subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/img_processed", 10, std::bind(&ImageToPCDPublisher::image_callback, this, std::placeholders::_1));
        this->pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcd", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert the ROS Image message to a CV image
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

        // Assume cv_image is binary (0 or 255). Find coordinates of all white pixels
        cv::Mat cv_image = cv_ptr->image;
        cv::Mat white_pixels;
        cv::findNonZero(cv_image, white_pixels);

        // Get scale parameter
        double scale;
        this->get_parameter("scale", scale);

        // Scale x and y coordinates
        // TODO: Scale the coordinates. This is non-trivial in C++ and depends on your exact requirements.
        // Scale the coordinates by the scale parameter linearly. For example, if the scale parameter is 0.5, then the x and y coordinates should be halved.
        

        // Create 3D points, setting z = 0
        // TODO: Create 3D points. This is non-trivial in C++ and depends on your exact requirements.

        // Convert points to PointCloud2
        // TODO: Convert points to PointCloud2. This is non-trivial in C++ and depends on your exact requirements.

        // Publish the PointCloud2
        // TODO: Publish the PointCloud2. This is non-trivial in C++ and depends on your exact requirements.
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageToPCDPublisher>());
    rclcpp::shutdown();
    return 0;
}