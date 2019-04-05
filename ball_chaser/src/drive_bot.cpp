#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular z velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

bool handle_drive_request(ball_chaser::DriveToTarget::Request &req,
	ball_chaser::DriveToTarget::Response &res)
{
    // Publish the requested velocities
    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities, forward
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);

    // Handle ROS communication events
    res.msg_feedback = "linear x = " + std::to_string(req.linear_x) + ", angular z = " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Create publisher object
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer command_robot = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    ROS_INFO("Waiting for motor command...");
    ros::spin();

    return 0;
}
