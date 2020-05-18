#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

int const MAX_RPM		= 170;
float const DIST_BETWEEN_WHEELS	= 0.225;  // This is the distance between the wheels in m
float const WHEEL_RADIUS	= 0.0509; // radius of the wheels in m

class Twist2MotorRef
{
    private:
	ros::NodeHandle *nh_p_;

	//private members for publishing
	std_msgs::Float64 left_ref_, right_ref_;

	//subscriber for cmd_vel
	ros::Subscriber vel_sub_;

	ros::Publisher right_rpm_pub_;
	ros::Publisher left_rpm_pub_;

	void velCallback(const geometry_msgs::Twist::ConstPtr &msg);

    public:
	Twist2MotorRef(ros::NodeHandle *nh);
};
