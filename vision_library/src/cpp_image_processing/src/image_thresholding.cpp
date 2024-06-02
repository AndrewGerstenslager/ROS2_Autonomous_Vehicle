#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std;
using namespace cv;

class ImageProcessor : public rclcpp::Node {
public:
  ImageProcessor() 
    : Node("image_processor") {

    // Retrieve the topic names from the ROS2 parameter server
    this->declare_parameter<std::string>("subscribed_topic", "input_image");
    this->declare_parameter<std::string>("published_topic", "output_image");
    this->declare_parameter<std::string>("published_debug_topic","debug_threshold_image");
    this->declare_parameter<int>("min_area",100);
    this->declare_parameter<bool>("debug_mode",false);
    this->declare_parameter<bool>("remove_orange",false);


    std::string subscribed_topic = this->get_parameter("subscribed_topic").as_string();
    std::string published_topic = this->get_parameter("published_topic").as_string();
    std::string debug_published_topic = this->get_parameter("published_debug_topic").as_string();
    min_area=this->get_parameter("min_area").as_int();
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(published_topic, 10);
    debug_publisher_=this->create_publisher<sensor_msgs::msg::Image>(debug_published_topic, 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subscribed_topic, 10, bind(&ImageProcessor::image_callback, this, placeholders::_1));

  }

private:
  cv::Mat thresholdHSV(const cv::Mat& image, const cv::Scalar& lower_bound, const cv::Scalar& upper_bound) {
    // Convert the image from BGR to HSV
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // Threshold the image
    cv::Mat thresholded_image;
    cv::inRange(hsv_image, lower_bound, upper_bound, thresholded_image);

    return thresholded_image;
  }

  cv::Mat nuke_cone(const cv::Mat& image, const cv::Scalar& lower_cone, const cv::Scalar& upper_cone,cv::Mat& debug_img) {
        // Convert the image from BGR to HSV
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // Threshold the image to get a binary mask
    cv::Mat thresholded_image;
    cv::inRange(hsv_image, lower_cone, upper_cone, thresholded_image);

    // Clone the original image to create the result
    cv::Mat result = image.clone();

    debug_img=image.clone();

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresholded_image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    Mat mask = Mat::zeros(thresholded_image.size(), thresholded_image.type());
    for (const auto &contour : contours) {
      double area = contourArea(contour);
      if (area >= 50) {
        drawContours(mask, vector<vector<Point>>(1, contour), -1, Scalar(255), FILLED);
      }
    }
    // Process each column to replace pixels from the detected point and above with black
    for (int j = 0; j < result.cols; j++) {
        bool detected = false;
        for (int i = result.rows - 1; i >= 0; i--) {
            // If a pixel within the range is detected
            if (mask.at<uchar>(i, j) >=125 ) {
                detected = true;
                debug_img.at<cv::Vec3b>(i,j)=cv::Vec3b(0,0,0);
            }
            // If detected, set the pixel and all above pixels to black
            if (detected) {
                result.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    return result;
  }

    void writefile(const std::string& filename) {
        std::ofstream file(filename);
        if (file.is_open()) {
            file << low_h << std::endl;
            file << low_s << std::endl;
            file << low_v << std::endl;
            file << high_h << std::endl;
            file << high_s << std::endl;
            file << high_v << std::endl;
            file<<std::endl;
            file << cone_low_h << std::endl;
            file << cone_low_s << std::endl;
            file << cone_low_v << std::endl;
            file << cone_high_h << std::endl;
            file << cone_high_s << std::endl;
            file << cone_high_v << std::endl;
            file.close();
        }
    }
    void readfile(const std::string& filename) {
        std::ifstream file(filename);
        if (std::filesystem::exists(filename)) {
            string a;
            file >> low_h >> low_s >> low_v >> high_h >> high_s >> high_v;
            file>>a;
            file >> cone_low_h >> cone_low_s >> cone_low_v >> cone_high_h >> cone_high_s >> cone_high_v;
            file.close();
        }
        else{
                writefile("/home/uc_jetson/repo/dokalman/vision_library/src/cpp_image_processing/calibration_data/calibrate_hsv.txt");
            }
    }

    void load_params()
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("cpp_image_processing");

        std::string calibration_data_file = package_share_directory + file_path;

        // Check if file exists
        if (!std::filesystem::exists(calibration_data_file)) {
            //RCLCPP_ERROR(this->get_logger(), "File does not exist: %s", calibration_data_file.c_str());
            return;
        }
        else{
            //RCLCPP_INFO(this->get_logger(),"File Exists");
        }

        //RCLCPP_INFO(this->get_logger(), "Attempting to read from file: %s", calibration_data_file.c_str());

        readfile("/home/uc_jetson/repo/dokalman/vision_library/src/cpp_image_processing/calibration_data/calibrate_hsv.txt");
        
        //writefile("/home/uc_jetson/repo/dokalman/vision_library/src/cpp_image_processing/calibration_data/test_hsv.txt");
        
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

    Mat img = cv_ptr->image;

    // Check if the image is loaded successfully
    if (img.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Error loading the image");
      return;
    }
    load_params();
    cv::Scalar lower_bound(low_h, low_s, low_v); // Lower bound of HSV values
    cv::Scalar upper_bound(high_h, high_s, high_v); // Upper bound of HSV values
    cv::Scalar lower_cone(cone_low_h,cone_low_s,cone_low_v);
    cv::Scalar upper_cone(cone_high_h,cone_high_s,cone_high_v);

    cv::Mat nuked_img=img.clone();
    cv::Mat debug_img;
    if (this->get_parameter("remove_orange").as_bool()){
      nuked_img=nuke_cone(img,lower_cone,upper_cone,debug_img);
    }

    if (this->get_parameter("debug_mode").as_bool()){
      sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg();

      // Publish the processed image
      debug_publisher_->publish(*debug_msg);
    }
    cv::Mat img_thr=thresholdHSV(nuked_img,lower_bound,upper_bound);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_thr, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    Mat mask = Mat::zeros(img_thr.size(), img_thr.type());
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
  int min_area;
  int low_h=0;
  int low_s=0;
  int low_v=0;
  int high_h=255;
  int high_s=255;
  int high_v=255;

  int cone_low_h=0;
  int cone_low_s=0;
  int cone_low_v=0;
  int cone_high_h=255;
  int cone_high_s=255;
  int cone_high_v=255;
  string file_path="/calibration_data/test_hsv.txt";
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
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



