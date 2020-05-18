#include "twist2RPM.h"

#include <iostream>
#include <string.h>
#include <cmath>

Twist2MotorRef::Twist2MotorRef(ros::NodeHandle *node_handle)
{
    nh_p_ = node_handle;

    //Subscription setup
    vel_sub_ = nh_p_->subscribe("/cmd_vel", 1, &Twist2MotorRef::velCallback, this);

    //Publication setup
    right_rpm_pub_  = nh_p_->advertise<std_msgs::Float64>("/setpoint_r",1);
    left_rpm_pub_   = nh_p_->advertise<std_msgs::Float64>("/setpoint_l",1);

    left_ref_.data  = 0.0;
    right_ref_.data = 0.0;
}

void Twist2MotorRef::velCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    right_ref_.data = msg->linear.x + (msg->angular.z * DIST_BETWEEN_WHEELS)/2.0;	// linear velocity m/s
    left_ref_.data  = msg->linear.x - (msg->angular.z * DIST_BETWEEN_WHEELS)/2.0;	// linear velocity m/s

    right_ref_.data /= WHEEL_RADIUS;	//angular velocity rad/s
    left_ref_.data /= WHEEL_RADIUS;	//angular velocity rad/s

    right_ref_.data *= 60/(2*M_PI);   //RPM
    left_ref_.data *= 60/(2*M_PI);	//RPM

    int sign_r, sign_l;
    sign_r = right_ref_.data < 0 ? -1 : 1;
    sign_l = left_ref_.data < 0 ? -1 : 1;

    right_ref_.data = abs(right_ref_.data) > MAX_RPM ? MAX_RPM * sign_r : right_ref_.data;
    left_ref_.data = abs(left_ref_.data) > MAX_RPM ? MAX_RPM * sign_l: left_ref_.data;

    right_rpm_pub_.publish(right_ref_);
    left_rpm_pub_.publish(left_ref_);
}
