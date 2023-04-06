#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main() {
    // Read in the image
    Mat img = imread("data/test_img1.png");

    // Check if the image is loaded successfully
    if (img.empty()) {
        cout << "Error loading the image" << endl;
        return -1;
    }

    // Convert the image to grayscale
    Mat img_bw;
    cvtColor(img, img_bw, COLOR_BGR2GRAY);

    // Threshold the image to detect the white line
    Mat img_thr;
    threshold(img_bw, img_thr, 215, 255, THRESH_BINARY);

    // Find contours in the thresholded image
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_thr, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Initialize an empty mask of the same size as the thresholded image
    Mat mask = Mat::zeros(img_thr.size(), img_thr.type());

    // Filter blobs by size and draw them on the mask
    int min_area = 250;
    for (const auto& contour : contours) {
        double area = contourArea(contour);
        if (area >= min_area) {
            drawContours(mask, vector<vector<Point>>(1, contour), -1, Scalar(255), FILLED);
        }
    }

    // Create windows for displaying images
    namedWindow("Original Image", WINDOW_NORMAL);
    namedWindow("Grayscale Image", WINDOW_NORMAL);
    namedWindow("Thresholded Image", WINDOW_NORMAL);
    namedWindow("Filtered Blobs", WINDOW_NORMAL);

    // Show the images
    imshow("Original Image", img);
    imshow("Grayscale Image", img_bw);
    imshow("Thresholded Image", img_thr);
    imshow("Filtered Blobs", mask);

    // Wait for a key press and close the windows
    waitKey(0);
    destroyAllWindows();

    return 0;
}
