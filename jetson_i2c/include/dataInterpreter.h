#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>

#include <iostream>
#include <string>
#include <vector>

#include "i2c.h"

class SlaveDevice
{
    public:
	SlaveDevice(int read_b = 0, int write_b = 0, int address = 0, ros::NodeHandle *nh = 0)
	    : read_bytes(read_b), write_bytes(write_b), addr(address)
	{
	    p_node_handle = nh;
	}

	void initDevice(const int bus);

    protected:
	int read_bytes, write_bytes, addr;
	
	//ROS Node Handle
	ros::NodeHandle *p_node_handle;

	I2CDevice device;
};

class MotorData: public SlaveDevice
{
    public:
	MotorData(int read_b, int write_b, int addr, std::vector<std::string> sub_topics,
		std::vector<std::string> pub_topics, ros::NodeHandle *nh);

	bool readDevice();
	bool writeDevice();
    private:
	//Callbacks
	void callbackDutyCycleReference( const std_msgs::Float64::ConstPtr& msg );

	//private data members
	int duty_cycle_ref;
	bool desired_wheel_dir;
	bool real_wheel_dir;
	std_msgs::Float64 rpm;

	//ROS Subscribers
	ros::Subscriber sub_duty_cycle_reference;

	//ROS Publishers
	ros::Publisher pub_rpm;
};

class UltrasonicData: public SlaveDevice
{
    public:
	UltrasonicData(int read_b, int write_b, int addr, std::vector<std::string> pub_topics,
		std::vector<std::string> sensor_frames, float min_range, float max_range,
		ros::NodeHandle *nh);
	bool readDevice();
    private:
	//No callbacks
	//No ROS Subscribers
	
	//private data members
	sensor_msgs::Range sensor1, sensor2, sensor3;

	//ROS Publishers
	ros::Publisher pub_distance1; //in mm
	ros::Publisher pub_distance2; //in mm
	ros::Publisher pub_distance3; //in mm
};
