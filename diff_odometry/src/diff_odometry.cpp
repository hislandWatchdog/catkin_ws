#include "../include/diff_odometry/diff_odometry.h"

void calculateOdometry(double delta_time) {
    double left = (double)speed_left.data;
    double right = (double)speed_right.data;
    //////////////////////////////////////////////////////////////////////////// Obtain distance traveled
    double vl = (left * _WHEEL_PERIMETER / 60.0); // m/s left-wheel
    double vr = (right * _WHEEL_PERIMETER / 60.0); // m/s right-wheel
    double v  =  (vr + vl) / 2.0; // m/s of the center of the robot
    //////////////////////////////////////////////////////////////////////////// Obtain velocities
    vth = (vr - vl) / _SEPARATION_BETWEEN_WHEELS; // rad/s
    th = th + vth * delta_time; // rad
    vx = v * cos(th); // m/s
    vy = v * sin(th); // m/s
    //////////////////////////////////////////////////////////////////////////// Obtain new positions
    x = x + vx * delta_time;
    y = y + vy * delta_time;
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
}

void spd_msgCallback_left(const std_msgs::Float64& msg) {
    speed_left = msg;
}
void spd_msgCallback_right(const std_msgs::Float64& msg) {
    speed_right = msg;
}
