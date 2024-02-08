#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class ImageToPCDPublisher : public rclcpp::Node
{
public:
    ImageToPCDPublisher()
    : Node("image_to_pcd_publisher")
    {
        this->declare_parameter<float>("scale", 1.0f / 255.0f);
        this->declare_parameter<std::string>("input_topic", "/img_processed");
        this->declare_parameter<std::string>("output_topic", "pcd");
        
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic,
            10,
            std::bind(&ImageToPCDPublisher::image_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat &cv_image = cv_ptr->image;

        std::vector<cv::Point2f> points;
        for (int y = 0; y < cv_image.rows; y++)
        {
            for (int x = 0; x < cv_image.cols; x++)
            {
                if (cv_image.at<uchar>(y, x) == 255)
                {
                    float scale = static_cast<float>(this->get_parameter("scale").as_double());

                    points.push_back(cv::Point2f(x * scale, y * scale));
                }
            }
        }

        auto cloud_msg = create_point_cloud(points, "base_link");
        publisher_->publish(*cloud_msg);
    }

    std::shared_ptr<sensor_msgs::msg::PointCloud2> create_point_cloud(const std::vector<cv::Point2f>& points, const std::string& frame_id)
    {
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header.frame_id = frame_id;
        cloud_msg->height = 1;
        cloud_msg->width = points.size();
        cloud_msg->is_dense = false;
        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

        for (const auto& point : points)
        {
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = 0.0f; // Assuming Z = 0
            ++iter_x; ++iter_y; ++iter_z;
        }

        return cloud_msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageToPCDPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
