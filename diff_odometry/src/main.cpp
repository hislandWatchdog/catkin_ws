// Publisher and subscriber in a single node: https://gist.github.com/PrieureDeSion/77c109a074573ce9d13da244e5f82c4d
// nav_msgs: https://answers.ros.org/question/241602/get-odometry-from-wheels-encoders/
// nav_msgs: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
// nav_msgs: https://www.youtube.com/watch?v=I_5leJK8vhQ&t=382s
// calculations: https://www.youtube.com/watch?v=oeTxkIWdd9I
// diff robot model: https://www.youtube.com/watch?v=aE7RQNhwnPQ
// dc motor: https://www.pololu.com/product/4754
// for cartographer: https://google-cartographer-ros.readthedocs.io/en/latest/going_further.html

#include "../include/diff_odometry/diff_odometry.h"

std_msgs::UInt8 speed_left;
std_msgs::UInt8 speed_right;
//////////////////////////////////////////////////////////////////////////// ODOM SETUP
// Positions
double x;    // Robot starts at the origin of the "odom" coordinate frame initially
double y;
double th;
// Velocities
double vx;   // 0.0 m/s in x
double vy;   // 0.0 m/s in y
double vth;  // 0.0 rad/s in theta

double prev_left;
double prev_right;

int main(int argc, char **argv) {
    //////////////////////////////////////////////////////////////////////////// ROS SETUP
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle nodeHandle;

    ros::Publisher  publisher = nodeHandle.advertise<nav_msgs::Odometry>("odom", 1);
    ros::Subscriber subscriber_left_spd = nodeHandle.subscribe("speed_left",1,spd_msgCallback_left);
    ros::Subscriber subscriber_right_spd = nodeHandle.subscribe("speed_right",1,spd_msgCallback_right);

    tf::TransformBroadcaster odom_broadcaster;

    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    geometry_msgs::Quaternion odom_quat;

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    ros::Rate loopRate(100);  // Frequency of repetition in Hz
    //////////////////////////////////////////////////////////////////////////// USAGE
    initializeGlovalVariables();

    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();

    while( ros::ok() ){
        ros::spinOnce(); // Checks for received messages
        current_time = ros::Time::now();

        //////////////////////////////////////////////////////////////////////// Odometry
        calculateOdometry((current_time - last_time).toSec());
        odom.header.stamp = current_time;
        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom_quat = tf::createQuaternionMsgFromYaw(th);
        odom.pose.pose.orientation = odom_quat;
        //set the velocity
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //////////////////////////////////////////////////////////////////////// Transform
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //publish the message
        odom_broadcaster.sendTransform(odom_trans);
        publisher.publish(odom);

        last_time = current_time;
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}












//
