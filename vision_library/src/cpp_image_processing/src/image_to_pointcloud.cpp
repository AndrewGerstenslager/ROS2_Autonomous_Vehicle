#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <cmath>
class ImageToPCDPublisher : public rclcpp::Node
{

public:
    ImageToPCDPublisher()
    : Node("image_to_pcd_publisher")
    {
        //this->declare_parameter<float>("scale", 1.0f / 255.0f);
        this->declare_parameter<std::string>("input_topic", "/img_processed");
        this->declare_parameter<std::string>("output_topic", "pcd");
        this->declare_parameter<float>("yaw", 0.0f);
        this->declare_parameter<float>("trans_x",0.0f);
        this->declare_parameter<float>("trans_y",0.0f);
        this->declare_parameter<std::string>("ipm_file_path", "calibration_data/test.txt");

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        name_debug=input_topic;
        file_path = this->get_parameter("ipm_file_path").as_string();
        scale = 3.0/255.0;
        loadCal();
        //trans_x-=old_bottom_center.x*scale*(cos((yaw-180.0)*M_PI/180.0));
        //trans_y-=(old_bottom_center.x)*scale*(sin((yaw-180.0)*M_PI/180.0));
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic,
            10,
            std::bind(&ImageToPCDPublisher::image_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    }

private:
void loadCal()
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("cpp_image_processing");

        std::string calibration_data_file = package_share_directory + file_path;

        // Check if file exists
        if (!std::filesystem::exists(calibration_data_file)) {
            RCLCPP_ERROR(this->get_logger(), "File does not exist: %s", calibration_data_file.c_str());
            return;
        }
        else{
            RCLCPP_INFO(this->get_logger(),"File Exists");
        }

        RCLCPP_INFO(this->get_logger(), "Attempting to read from file: %s", calibration_data_file.c_str());

        std::ifstream file(calibration_data_file);

        std::string line;
        int row = 0;
        while (getline(file, line))
        {
            if (row==3){            
                scale=std::stod(line);
            }
            else if(row==4){
                std::stringstream ss(line);
                std::string x,y;
                ss>>x;
                ss>>y;
                old_bottom_center.x=std::stod(x);
                old_bottom_center.y=std::stod(y);
            }
            row++;
        }

    }

    cv:: Mat rotate_image(cv::Mat source, float angle)
    {
        cv::Point2f center((source.cols - 1) / 2.0, (source.rows - 1) / 2.0);
        cv::Mat rotation_matrix = getRotationMatrix2D(center, angle, 1.0);
        cv::Mat rotated_image;
        cv::warpAffine(source, rotated_image, rotation_matrix, cv::Size(source.rows*2.0,source.cols*2.0));

        std::vector<cv::Point2f> old_point, new_point;
        old_point.push_back(old_bottom_center); 
        old_point.push_back(cv::Point2f(source.cols-1,source.rows-1));   
        cv::transform(old_point, new_point, rotation_matrix);
        cv::Point2f flipped_point=new_point[0];
        flipped_point.x+=2*(rotated_image.cols/2.0-flipped_point.x);
        trans_x-=(flipped_point.x)*scale;
        trans_y-=(flipped_point.y)*scale;   

        // cv::Point2f new_point;
        // new_point.x=(old_bottom_center.x-center.x)*cos(yaw*M_PI/180.0)-(old_bottom_center.y-center.y)*sin(yaw*M_PI/180.0);
        // new_point.y=(old_bottom_center.x-center.x)*sin(yaw*M_PI/180.0)+(old_bottom_center.y-center.y)*cos(yaw*M_PI/180.0);
        // new_point.x+=2*(rotated_image.cols/2.0-new_point.x);
        // trans_x-=(new_point.x)*scale;
        // trans_y-=(new_point.y)*scale;  

        return rotated_image;
    }
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
        yaw=this->get_parameter("yaw").as_double();
        trans_x=this->get_parameter("trans_x").as_double();
        trans_y=this->get_parameter("trans_y").as_double();
        //Rotating cloudpoints is hard man so rotate image instead.
        //Potential issue: rotated image may be cropped
        cv_image=rotate_image(cv_image,yaw);
        //Mirror image since point cloud of original image is mirrored
        cv::flip(cv_image, cv_image, 1);

        std::vector<cv::Point2f> points;
        cv::Point2f original_point;

        for (int y = 0; y < cv_image.rows; y++)
        {
            for (int x = 0; x < cv_image.cols; x++)
            {
                if (cv_image.at<uchar>(y, x) == 255)
                {

                    original_point=cv::Point2f(x * scale, y * scale);
                    //Translate
                    original_point.x += trans_x;
                    original_point.y += trans_y;
                    points.push_back(original_point);
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
    cv::Mat rotation_matrix_;
    std::string file_path;
    double scale;
    cv::Point2f old_bottom_center,new_bottom_center;
    cv::Mat rot_mat;
    float trans_x,trans_y,yaw;
    std::string name_debug;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageToPCDPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
