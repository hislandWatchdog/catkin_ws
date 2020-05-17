#include "../include/diff_odometry/diff_odometry.h"

void calculateOdometry(double delta_time) {
    double left = (double)speed_left.data;
    double right = (double)speed_right.data;

    if(left > 100.0) {
        left -= 100.0;
    }
    else {
        left *= -1.0;
    }
    
    double left_diff = abs(abs(left) - abs(prev_left));
    if(abs(left) < 15.0 || left_diff > 30) {
	left = 0.0;
    }
	
    if(right > 100.0) {
        right -= 100.0;
    }
    else {
        right *= -1.0;
    }

    double right_diff = abs(abs(right) - abs(prev_right));
    if(abs(right) < 15.0 || right_diff > 30) {
	right = 0.0;
    }
    //////////////////////////////////////////////////////////////////////////// Obtain distance traveled
    double vl = (left  / 60.0); // rad/s left-wheel * _WHEEL_PERIMETER; // m/s left-wheel
    double vr = (right / 60.0); // rad/s right-wheel * _WHEEL_PERIMETER; // m/s right-wheel
    double v  =  (vr + vl) / 2.0; // rad/s of the center of the robot
    //////////////////////////////////////////////////////////////////////////// Obtain velocities
    vth = (_WHEEL_RADIUS / _SEPARATION_BETWEEN_WHEELS) * (vr - vl); // rad/s
    th = th + vth * delta_time; // rad
    vx = _WHEEL_RADIUS * v * cos(th); // m/s
    vy = _WHEEL_RADIUS * v * sin(th); // m/s
    //////////////////////////////////////////////////////////////////////////// Obtain new positions
    x = x + vx * delta_time;
    y = y + vy * delta_time;

    prev_left = left;
    prev_right = right;
}

void initializeGlovalVariables() {
    speed_left.data = 0;
    speed_right.data = 0;
    // Positions
    x  = 0.0;    // Robot starts at the origin of the "odom" coordinate frame initially
    y  = 0.0;
    th = 0.0;
    // Velocities
    vx  = 0.0;   // 0.0 m/s in x
    vy  = 0.0;   // 0.0 m/s in y
    vth = 0.0;   // 0.0 rad/s in theta

    prev_left = 0.0;
    prev_right = 0.0;
}

void spd_msgCallback_left(const std_msgs::UInt8& msg) {
    speed_left = msg;
}
void spd_msgCallback_right(const std_msgs::UInt8& msg) {
    speed_right = msg;
}
