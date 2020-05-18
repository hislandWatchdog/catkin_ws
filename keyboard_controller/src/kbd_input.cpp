//ROS libraries
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "opencv2/highgui.hpp"	    //Interfaz --GUI
#include "opencv2/core.hpp"	    //Cv Mat
#include "opencv2/imgproc.hpp"	    //Filters

#include <iostream>
#include <string>
#include <cmath> //for PI constant


const char* window_name	    = "Keyboard Controller";
const int width		    = 400;
const int height	    = 130;
const float font_scale	    = 2;
const int text_thickness    = 2;
const int text_font	    = cv::FONT_HERSHEY_PLAIN;
const int line_type	    = cv::FILLED;
const cv::Scalar text_color = cv::Scalar(79,14,150);

const cv::Point text_origin		= cv::Point(110,30);
const cv::Point speed_lvl_text_origin   = cv::Point(0,70);
const cv::Point speed_text_origin	= cv::Point(0,110);
const cv::Point units_text_origin	= cv::Point(300,110);

const float DIST_BETWEEN_WHEELS	= 0.225;  // This is the distance between the wheels in m
const float WHEEL_RADIUS	= 0.0509; // radius of the wheels in m
const int MAX_RPM		= 150;
const int LEVELS		= 10;

const float MAX_LINEAR_SPEED	= (MAX_RPM * (2*M_PI)/60) * WHEEL_RADIUS ; // m/s
const float MAX_ANGULAR_SPEED	= MAX_LINEAR_SPEED / (DIST_BETWEEN_WHEELS/2); //rad/s

const float MIN_LINEAR_SPEED = MAX_LINEAR_SPEED / LEVELS;
const float MIN_ANGULAR_SPEED = MAX_ANGULAR_SPEED / LEVELS;

int main(int argc, char **argv)
{
    cv::Mat static_frame(height, width, CV_8UC3, cv::Scalar(150,150,150));
    cv::Mat text_frame; 
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    ros::init(argc, argv, "keyboardController");
    ros::NodeHandle nh;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist msg;
    
    char key	    = 0;
    int speed_lvl   = 0;
    float speed	    = 0.0;
    std::string units_msg = "";

    //initialize message on screen
    std::string text_msg = "IDLE";

    //initialize speed
    msg.linear.x    = 0;
    msg.linear.y    = 0;
    msg.linear.z    = 0;
    msg.angular.x   = 0;
    msg.angular.y   = 0;
    msg.angular.z   = 0;

    while(ros::ok())
    {
	static_frame.copyTo(text_frame);    //This is done so that the window is refreshed

	cv::putText(text_frame, text_msg, text_origin, text_font, font_scale,
		    text_color, text_thickness, line_type);
	//putText overwrites pixels in the image to write text 
	cv::putText(text_frame, "speed_lvl: "+std::to_string(speed_lvl), speed_lvl_text_origin,
		text_font, font_scale, text_color, text_thickness, line_type);
	
	cv::putText(text_frame, "speed: "+std::to_string(speed), speed_text_origin,
		text_font, font_scale, text_color, text_thickness, line_type);
	
	cv::putText(text_frame, units_msg, units_text_origin,
		text_font, font_scale, text_color, text_thickness, line_type);

	imshow(window_name, text_frame);
	
	key = cv::waitKey(1);
	switch(key)
	{ 
	    case 'a':
		speed = MIN_ANGULAR_SPEED * speed_lvl;

		msg.linear.x	= 0;
		msg.angular.z	= speed;

		text_msg = "Left Turn";
		units_msg = "rad/s";
		break;
	    case 'd':
		speed = -MIN_ANGULAR_SPEED * speed_lvl;

		msg.linear.x	= 0;
		msg.angular.z	= speed;

		text_msg = "Right Turn";
		units_msg = "rad/s";
		break;
	    case 's':
		speed = -MIN_LINEAR_SPEED * speed_lvl;

		msg.linear.x	= speed; 
		msg.angular.z	= 0;

		text_msg = "Reverse";
		units_msg = "m/s";
		break;
	    case 'w':
		speed = MIN_LINEAR_SPEED * speed_lvl;

		msg.linear.x	= speed;
		msg.angular.z	= 0;

		text_msg = "Forward";
		units_msg = "m/s";
		break;
	    case 'z':
		if(speed_lvl > 1)
		    speed_lvl--;
		break;
	    case 'x':
		if(speed_lvl < LEVELS)
		    speed_lvl++;
		break;
	    case -1:
		//Do nothing because no key was pressed
		break;
	    case -23:
		// Do nothing alt key was pressed
		break;
	    case 9:
		// Do nothing tab key was pressed
		break;
	    case 27:
		//ESC was pressed close program
		msg.linear.x	= 0;
		msg.angular.z	= 0;
		vel_pub.publish(msg);

		ros::shutdown();
		break;
	    default:
		text_msg = "IDLE";
		speed_lvl = 0;
		msg.linear.x	= 0;
		msg.angular.z	= 0;
		std::cout << "Key: " << int(key) << std::endl;
	}

	vel_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
