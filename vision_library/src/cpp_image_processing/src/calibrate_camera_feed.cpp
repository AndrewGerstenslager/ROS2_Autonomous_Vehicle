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

cv::Size checkerboard_shape=cv::Size(6,8);  //size of checkerboard (order of width and height doesnt matter)
int width=checkerboard_shape.width;
int height=checkerboard_shape.height;

class CameraImageProcessor : public rclcpp::Node {
public:
    CameraImageProcessor() : Node("camera_image_processor"), ipm_matrix(cv::Mat::eye(3, 3, CV_64F)) {
        // Retrieve the topic names from the ROS2 parameter server
        this->declare_parameter<std::string>("subscribed_topic", "input_image");
        this->declare_parameter<std::string>("published_topic", "output_image");
        this->declare_parameter<std::string>("global_ipm_file_path", "/dokalman/vision_library/src/cpp_image_processing/calibration_data/test_cal.txt");

        std::string subscribed_topic = this->get_parameter("subscribed_topic").as_string();
        std::string published_topic = this->get_parameter("published_topic").as_string();
        global_file_path = this->get_parameter("global_ipm_file_path").as_string();

        //loadIPMMatrix();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(published_topic, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            subscribed_topic, 10, bind(&CameraImageProcessor::image_callback, this, placeholders::_1));
    }   

private:

    bool check_horizontal_checkerboard(cv::Point2f& top_left,cv::Point2f& top_right,cv::Point2f& bottom_right,cv::Point2f& bottom_left,int threshold=20){
        if ((top_left.y<bottom_right.y) and (top_left.x<bottom_right.x) and (abs(top_left.y-top_right.y)<=threshold))
            return true;
        if ((top_left.y<bottom_right.y) and (top_left.x>bottom_right.x) and (abs(top_left.y-bottom_left.y)<=threshold)){
            checkerboard_shape=cv::Size(checkerboard_shape.height,checkerboard_shape.width);
            width=checkerboard_shape.width;
            height=checkerboard_shape.height;
            cv::Point2f temp=top_left;
            top_left =  bottom_left;
            top_right =  temp;
            bottom_right = top_right;
            bottom_left =  bottom_right;
            return true;
        }
        return false;
    }

    void saveIPMMatrix(const std::string& file_path, const cv::Mat& matrix, float sc, cv::Point2f bottom_left_corner, cv::Size output_size) {
        // Save the IPM matrix to file
        ofstream outfile;
        outfile.open (file_path);
        for (int row = 0; row < matrix.rows; row++) {
            for (int col = 0; col < matrix.cols; ++col) {
                outfile << matrix.at<double>(row, col);
                if (col < matrix.cols - 1) outfile<< " ";
            }
            if (row<matrix.rows-1) outfile << endl;
        }
        outfile<<endl<<sc;
        outfile<<endl<<bottom_left_corner.x<<" "<<bottom_left_corner.y;
        outfile<<endl<<output_size.width<<" "<<output_size.height;
        outfile.close();
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

        cv::Mat im = input_image.clone();
        cv::Mat processed_image=im.clone();
        std::vector<cv::Point2f> corners;
        cv::Mat output_image=processed_image.clone();

        bool ret = cv::findChessboardCorners(im, checkerboard_shape, corners);
        //bool checkerboard_horizontal=false;

        //if checkerboard found
        if (ret){

            //Getting 4 corners from detected checkerboard
            cv::Point2f top_left = cv::Point2f(corners[0]);
            cv::Point2f top_right = cv::Point2f(corners[width-1]);
            cv::Point2f bottom_left = cv::Point2f(corners[width*(height-1)]);
            cv::Point2f bottom_right = cv::Point2f(corners[width*height-1]);

            //Checking if image is parallel to the camera
            check_horizontal_checkerboard(top_left,top_right,bottom_right,bottom_left);

            if (true){//checkerboard_horizontal){
                //warp image but not correct crop
                cv::Point2f points[4]={top_left,bottom_left,bottom_right,top_right};
                int x_size=25;
                int y_size= x_size*height/width;
                int y_mid= processed_image.rows*0.5;
                int x_mid = processed_image.cols*0.5;
                //Getting 4 corners of checkerboard warped to top-down view
                cv::Point2f ipm_top_left = cv::Point2f(x_mid - x_size, y_mid - y_size);
                cv::Point2f ipm_bottom_left = cv::Point2f(x_mid - x_size, y_mid + y_size);
                cv::Point2f ipm_bottom_right = cv::Point2f(x_mid + x_size, y_mid + y_size);
                cv::Point2f ipm_top_right = cv::Point2f(x_mid + x_size, y_mid - y_size);

                cv::Point2f ipm_points[4] = {ipm_top_left, ipm_bottom_left, ipm_bottom_right, ipm_top_right};

                //Warp image
                ipm_matrix = cv::getPerspectiveTransform(points, ipm_points);

                std::vector<Point2f> old_corners,new_corners;
                old_corners.push_back(Point2f(0,0));
                old_corners.push_back(Point2f(0,im.rows));
                old_corners.push_back(Point2f(im.cols,im.rows));
                old_corners.push_back(Point2f(im.cols,0));
                cv::perspectiveTransform(old_corners, new_corners, ipm_matrix);

                //---------------------------------------------------------------------------------
                //warp image again but correct crop
                x_mid-=new_corners[0].x;
                y_mid-=new_corners[0].y;

                ipm_top_left = cv::Point2f(x_mid - x_size, y_mid - y_size);
                ipm_bottom_left = cv::Point2f(x_mid - x_size, y_mid + y_size);
                ipm_bottom_right = cv::Point2f(x_mid + x_size, y_mid + y_size);
                ipm_top_right = cv::Point2f(x_mid + x_size, y_mid - y_size);

                cv::Point2f ipm_cropped_points[4] = {ipm_top_left, ipm_bottom_left, ipm_bottom_right, ipm_top_right};

                //Warp image
                //new_corners.clear();
                ipm_matrix = cv::getPerspectiveTransform(points, ipm_cropped_points);
                cv::perspectiveTransform(old_corners, new_corners, ipm_matrix);

                //--------------------------------------------------------------
                
                cv::warpPerspective(im,processed_image, ipm_matrix,  cv::Size(new_corners[3].x,new_corners[1].y));//cv::Size(im.cols/prev_sc , im.rows/prev_sc));
                output_image=processed_image.clone();
                double sc=width*0.108/(x_size*2.0);//get_scale(processed_image);
                cv::Point2f bottom_center_corner;
                bottom_center_corner.x=(new_corners[1].x+new_corners[2].x)/2.0;
                bottom_center_corner.y=(new_corners[1].y+new_corners[2].y)/2.0;
                saveIPMMatrix(global_file_path,ipm_matrix,sc,bottom_center_corner,cv::Size(new_corners[3].x,new_corners[1].y));
                
                cv::circle(output_image, ipm_cropped_points[0], 3, Scalar(255, 0, 0));
                cv::circle(output_image, ipm_cropped_points[1], 3, Scalar(255, 0, 0));
                cv::circle(output_image, ipm_cropped_points[2], 3, Scalar(255, 0, 0));
                cv::circle(output_image, ipm_cropped_points[3], 3, Scalar(255, 0, 0));
                
                cv::line(output_image,(ipm_cropped_points[0]+ipm_cropped_points[3])*0.5,(ipm_cropped_points[1]+ipm_cropped_points[2])*0.5, Scalar(255, 0, 0),2);
                cv::line(output_image,(new_corners[0]+new_corners[3])*0.5,(new_corners[1]+new_corners[2])*0.5, Scalar(0, 255, 0),1);
                
                float y_mid_debug=((new_corners[0]+new_corners[1])*0.5).y;
                cv::line(output_image,cv::Point2f(0,y_mid_debug),cv::Point2f(output_image.cols,y_mid_debug),Scalar(255, 0, 0),2);
                cv::line(output_image,(new_corners[0]+new_corners[1])*0.5,(new_corners[2]+new_corners[3])*0.5,Scalar(0, 255, 0),1);
            }
             //if checkerboard not parallel, output img is not warped
        }


        // Apply the IPM transformation
        //cv::warpPerspective(im, output_image, ipm_matrix, im.size());

        // Convert the processed OpenCV image to a ROS Image message
        sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(msg->header, "bgr8", output_image).toImageMsg();

        // Publish the processed image
        publisher_->publish(*processed_msg);
    }

    std::string local_file_path,global_file_path;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::Mat ipm_matrix;
    };

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create the CameraImageProcessor
    auto node = std::make_shared<CameraImageProcessor>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
