#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


using namespace std;
using namespace cv;

class ImageToPointCloudNode : public rclcpp::Node {
public:
    ImageToPointCloudNode()
: Node("image_to_pointcloud_node")/*, static_broadcaster_(this->shared_from_this()) */{
    // Retrieve the topic names from the ROS2 parameter server
    this->declare_parameter<std::string>("subscribed_topic", "input_image");
    this->declare_parameter<std::string>("published_topic", "output_pointcloud");

    std::string subscribed_topic = this->get_parameter("subscribed_topic").as_string();
    std::string published_topic = this->get_parameter("published_topic").as_string();

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(published_topic, 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    subscribed_topic, 10, std::bind(&ImageToPointCloudNode::image_callback, this, std::placeholders::_1));
    /*
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "dokalman";
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;

    static_broadcaster_.sendTransform(transform_stamped);*/
}


private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS Image message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        Mat input_image = cv_ptr->image;
        // Check if the image is loaded successfully
        if (input_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Error loading the image");
            return;
        }

        // Create PointCloud2 message
        sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pointcloud_msg->header = msg->header;
        pointcloud_msg->height = 1;
        pointcloud_msg->width = static_cast<uint32_t>(countNonZero(input_image));
        pointcloud_msg->is_bigendian = false;
        pointcloud_msg->is_dense = true;

        pointcloud_msg->point_step = 12;
        pointcloud_msg->row_step = pointcloud_msg->point_step * pointcloud_msg->width;
        pointcloud_msg->data.resize(pointcloud_msg->row_step);

        // Add fields for x, y, z
        sensor_msgs::msg::PointCloud2::_fields_type::value_type x_field, y_field, z_field;
        x_field.name = "x";
        x_field.offset = 0;
        x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        x_field.count = 1;

        y_field.name = "y";
        y_field.offset = 4;
        y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        y_field.count = 1;

        z_field.name = "z";
        z_field.offset = 8;
        z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        z_field.count = 1;

        pointcloud_msg->fields.push_back(x_field);
        pointcloud_msg->fields.push_back(y_field);
        pointcloud_msg->fields.push_back(z_field);
        // Fill in the point cloud data
        uint32_t idx = 0;
        for (int y = 0; y < input_image.rows; y++) {
            for (int x = 0; x < input_image.cols; x++) {
                // If the pixel is white, add it to the point cloud
                if (input_image.at<uint8_t>(y, x) > 0) {
                    float *x_ptr = reinterpret_cast<float*>(&pointcloud_msg->data[idx * pointcloud_msg->point_step]);
                    float *y_ptr = reinterpret_cast<float*>(&pointcloud_msg->data[idx * pointcloud_msg->point_step + y_field.offset]);
                    float *z_ptr = reinterpret_cast<float*>(&pointcloud_msg->data[idx * pointcloud_msg->point_step + z_field.offset]);

                    *x_ptr = static_cast<float>(x);
                    *y_ptr = static_cast<float>(y);
                    *z_ptr = 0.0;

                    idx++;
                }
            }
        }
        pointcloud_msg->header.frame_id = "map";

        // Publish the point cloud
        publisher_->publish(*pointcloud_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    //tf2_ros::StaticTransformBroadcaster static_broadcaster_; // Add this line
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create the ImageToPointCloudNode
    auto node = std::make_shared<ImageToPointCloudNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

