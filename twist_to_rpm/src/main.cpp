#include "twist2RPM.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Twist2RPM");
    ros::NodeHandle nh;

    Twist2MotorRef vel_converter(&nh);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
	ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;
}
