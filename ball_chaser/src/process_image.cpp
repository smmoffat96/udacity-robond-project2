#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;

    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    client.call(srv);
}

// Name of OpenCV image
static const std::string OPENCV_WINDOW = "Image window";

// This function takes in an OpenCV image in BGR color channels and outputs to a window
void view_image(const cv::Mat img) {
    cv::namedWindow(OPENCV_WINDOW);
    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(3);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    float lin_x;
    float ang_z;
    cv::Vec3b rgbpix;
    
    // Convert ROS image message into OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::Mat cv_img = cv_ptr->image;
    
    // Parameters of image
    int height = cv_img.rows;
    int width = cv_img.cols;
    int tot_pixels = height * width;

    // Convert image to gray scale
    cv::Mat gray;
    cv::cvtColor(cv_img, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);

    // Set threshold and maxValue
    double thresh = 240;
    double maxValue = 255;

    // Binary threshold
    cv::Mat binary;
    cv::threshold(gray, binary, thresh, maxValue, cv::THRESH_BINARY);

    // Find white pixels
    std::vector<cv::Point> white_pixels;
    cv::findNonZero(binary, white_pixels);
    int num_white_pixels = white_pixels.size();
    
    // If too many white pixels, stop driving
    if (num_white_pixels > tot_pixels/4) {
        lin_x = 0.0;
        ang_z = 0.0;
    }
    // If not too many white pixels, go to drive commands
    else if (num_white_pixels != 0) {
        // Find moments of the image
        cv::Moments m = cv::moments(binary, true);
        // Position of centroid
        cv::Point p(m.m10/m.m00, m.m01/m.m00);

        // If on left side of image, turn left
        if (p.x <= width/3) {
            lin_x = 0.0;
            ang_z = 0.5;
        }
        // If in middle of image, drive forward
        else if (p.x <= 2*width/3) {
            lin_x = 0.5;
            ang_z = 0.0;
        }
        // If on right side of image, turn right
        else if (p.x <= width) {
            lin_x = 0.0;
            ang_z = -0.5;
        }
    }
    // If no white pixels detected, stop driving
    else {
        lin_x = 0.0;
        ang_z = 0.0;
    }

    // Pass velocity parameters to drive_robot
    drive_robot(lin_x, ang_z);
}


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
