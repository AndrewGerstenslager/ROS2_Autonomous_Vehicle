#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <filesystem>

cv::Size checkerboard_shape=cv::Size(8,6);

class CameraImageProcessor : public rclcpp::Node {
public:
    CameraImageProcessor() : Node("camera_image_processor") {
        this->declare_parameter<std::string>("camera_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("save_file_path", "./ipm_matrix.txt");

        this->get_parameter("camera_topic", camera_topic_);
        this->get_parameter("save_file_path", save_file_path_);

        //image_transport::ImageTransport it(this);
        //image_sub_ = it.subscribe(camera_topic_, 1, 
                                  //std::bind(&CameraImageProcessor::imageCallback, this, std::placeholders::_1));
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, 10, bind(&CameraImageProcessor::image_callback, this, placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        try {
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            // Process the image, find chessboard, apply IPM, etc.
            cv::Mat processed_image = processImage(image, ipm_matrix_);
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

    bool check_horizontal_checkerboard(cv::Point2f top_left,cv::Point2f top_right,cv::Point2f bottom_right,cv::Point2f bottom_left,int threshold=5){
        if ((top_left.y<bottom_right.y) and (top_left.x<bottom_right.x) and (abs(top_left.y-top_right.y)<=threshold))
            return true;
        if ((top_left.y<bottom_right.y) and (top_left.x>bottom_right.x) and (abs(top_left.y-bottom_left.y)<threshold))
            checkerboard_shape=cv::Size(checkerboard_shape.height,checkerboard_shape.width);
        return false;
    }

    cv::Mat processImage(cv::Mat& image, cv::Mat& ipm_matrix_) {
            cv::Mat im = image.clone();
            cv::Mat processed_image;

            //Checking if image is parallel to the camera

            int width=checkerboard_shape.width;
            int height=checkerboard_shape.height;
            std::vector<cv::Point2f> corners;
            bool ret = cv::findChessboardCorners(im, checkerboard_shape, corners);
            bool checkerboard_horizontal=false;
            if (ret){
                cv::Point2f top_left = cv::Point2f(corners[0]);
                cv::Point2f top_right = cv::Point2f(corners[width-1]);
                cv::Point2f bottom_left = cv::Point2f(corners[width*(height-1)]);
                cv::Point2f bottom_right = cv::Point2f(corners[width*height-1]);

                checkerboard_horizontal = check_horizontal_checkerboard(top_left,top_right,bottom_right,bottom_left);

                cv::Point2f points[4]={top_left,bottom_left,bottom_right,top_right};
                int y_mid = im.rows / 2;
                int x_mid = im.cols / 2;
                int size=100;
                
                cv::Point2f ipm_top_left = cv::Point2f(x_mid - size, y_mid - size);
                cv::Point2f ipm_bottom_left = cv::Point2f(x_mid - size, y_mid + size);
                cv::Point2f ipm_bottom_right = cv::Point2f(x_mid + size, y_mid + size);
                cv::Point2f ipm_top_right = cv::Point2f(x_mid + size, y_mid - size);

                cv::Point2f ipm_points[4] = {ipm_top_left, ipm_bottom_left, ipm_bottom_right, ipm_top_right};

                ipm_matrix_ = cv::getPerspectiveTransform(points, ipm_points);
                cv::warpPerspective(im,processed_image, ipm_matrix_, cv::Size(im.size[1],im.size[0]));
            }
        return processed_image;

    }

    void saveIPMMatrix(const std::string& file_path, const cv::Mat& matrix) {
        // Save the IPM matrix to file
    }

    std::string camera_topic_;
    std::string save_file_path_;
    cv::Mat ipm_matrix_;
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
