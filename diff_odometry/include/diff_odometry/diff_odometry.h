/* Prototype of class Collage */

#ifndef DIFF_ODOM_H
#define DIFF_ODOM_H

    #include <ros/ros.h>
    #include <std_msgs/Float64.h>
    #include <tf/transform_broadcaster.h>
    #include <nav_msgs/Odometry.h>
    #include <iostream>

    void calculateOdometry(double delta_time);
    void initializeGlovalVariables();
    void spd_msgCallback_left(const std_msgs::Float64& msg);
    void spd_msgCallback_right(const std_msgs::Float64& msg);

    const double _WHEEL_RADIUS = 0.0509; // meters
    const double _WHEEL_PERIMETER = 2 * 3.14159265 * _WHEEL_RADIUS;
    const double _SEPARATION_BETWEEN_WHEELS = 0.225; // meters

    extern std_msgs::Float64 speed_left;
    extern std_msgs::Float64 speed_right;
    //////////////////////////////////////////////////////////////////////////// ODOM SETUP
    // Positions
    extern double x;    // Robot starts at the origin of the "odom" coordinate frame initially
    extern double y;
    extern double th;
    // Velocities
    extern double vx;   // 0.0 m/s in x
    extern double vy;   // 0.0 m/s in y
    extern double vth;  // 0.0 rad/s in theta

#endif
