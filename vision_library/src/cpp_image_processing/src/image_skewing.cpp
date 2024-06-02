#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std;
using namespace cv;

class IpmNode : public rclcpp::Node {
public:
    IpmNode() : Node("ipm_node"), ipm_matrix(cv::Mat::eye(3, 3, CV_64F)) {
        // Retrieve the topic names from the ROS2 parameter server
        this->declare_parameter<std::string>("subscribed_topic", "input_image");
        this->declare_parameter<std::string>("published_topic", "output_image");
        this->declare_parameter<std::string>("ipm_file_path", "calibration_data/test.txt");

        std::string subscribed_topic = this->get_parameter("subscribed_topic").as_string();
        std::string published_topic = this->get_parameter("published_topic").as_string();
        file_path = this->get_parameter("ipm_file_path").as_string();

        loadIPMMatrix();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(published_topic, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subscribed_topic, 10, bind(&IpmNode::image_callback, this, placeholders::_1));
    }   

private:
    void loadIPMMatrix()
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

        string line;
        string floats[3];
        int row = 0;
        while (getline(file, line))
        {
            if (row<=2){
                stringstream ssin(line);
                int i = 0;
                while (ssin.good() && i < 3){
                    ssin >> floats[i];
                    ++i;
                }
                //RCLCPP_INFO(this->get_logger(), line);
                for (int col = 0; col < 3; col++) {
                    ipm_matrix.at<double>(row,col) = stold(floats[col]);
                }
            }
            if (row==5){
                std::stringstream ss(line);
                std::string w,h;
                ss>>w;
                ss>>h;
                output_size.width=std::stoi(w);
                output_size.height=std::stoi(h);
            }
            if (row>=6){
                break;
            }
            row++;
        }
        file.close();
        //printIPMMatrix();
    }

    void printIPMMatrix()
    {
        RCLCPP_INFO(this->get_logger(), "IPM Matrix:");
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
                RCLCPP_INFO(this->get_logger(), "%f ", ipm_matrix.at<double>(i,j));
            RCLCPP_INFO(this->get_logger(), "\n");
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS Image message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
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

        // Apply the IPM transformation
        cv::Mat output_image;
        cv::warpPerspective(input_image, output_image, ipm_matrix, output_size);

        // Convert the processed OpenCV image to a ROS Image message
        sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(msg->header, "bgr8", output_image).toImageMsg();

        // Publish the processed image
        publisher_->publish(*processed_msg);
    }

    std::string file_path;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::Mat ipm_matrix;
    cv::Size output_size;
    };

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create the IpmNode
    auto node = std::make_shared<IpmNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}