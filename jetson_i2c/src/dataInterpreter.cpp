//dataInterpreter.cpp

#include "dataInterpreter.h"

void SlaveDevice::initDevice(const int bus)
{
    device.bus = bus;
    device.addr = addr;
    device.tenbit = 0;
    device.delay = 1;
    device.flags = 0;
    device.page_bytes = write_bytes;	//This is the number of bytes to write
    device.iaddr_bytes = 0;
    /* If i2c device do not have internal address,
     * please use i2c_ioctl_read/write function
     * for read/write, set'iaddr_bytes=0.
     * */
}


/******Motor Data******/
MotorData::MotorData(int read_b, int write_b, int addr, std::vector<std::string> sub_topics,
	std::vector<std::string> pub_topics, ros::NodeHandle *nh)
    : SlaveDevice(read_b, write_b, addr, nh)
{
    //Subscriptions
    sub_duty_cycle_reference = p_node_handle->subscribe(sub_topics[0], 1, &MotorData::callbackDutyCycleReference, this);

    //Publications
    pub_rpm = p_node_handle->advertise<std_msgs::Float64>(pub_topics[0], 1);

    duty_cycle_ref = 0.0;
    desired_wheel_dir = 0;
}

bool MotorData::readDevice()
{
    unsigned char buffer[read_bytes] = {0};

    if( i2c_ioctl_read(&device, 0x0, buffer, read_bytes) == read_bytes )
    {
	rpm.data = buffer[0] + buffer[1]/100.0F;    //integer value added with a floating value
	real_wheel_dir = buffer[2];	    // 1/0 sets to true/false

	rpm.data = real_wheel_dir ? rpm.data * -1 : rpm.data;
	pub_rpm.publish(rpm);
	return true;
    }
    else
	return false;
}

bool MotorData::writeDevice()
{
    unsigned char buffer[write_bytes] = {0};

    buffer[0] = duty_cycle_ref & 0xFF;
    buffer[1] = (duty_cycle_ref & 0xFF00) >> 8;
    buffer[2] = desired_wheel_dir;

    if( i2c_ioctl_write(&device, 0x0, buffer, write_bytes) == write_bytes)
	return true;
    else
	return false;
}

void MotorData::callbackDutyCycleReference( const std_msgs::Float64::ConstPtr& msg )
{
    unsigned int magnitude = msg->data < 0 ? msg->data*-1 : msg->data;
    int sign = msg->data < 0 ? -1 : 1;
    
    if(magnitude > 1000)
	duty_cycle_ref = 1000;
    else if(magnitude < 50)
	duty_cycle_ref = 0;
    else
	duty_cycle_ref = magnitude;

    desired_wheel_dir = sign < 0 ? 1 : 0; 
}


/******Ultrasonic Data******/
UltrasonicData::UltrasonicData(int read_b, int write_b, int addr,
	std::vector<std::string> pub_topics, std::vector<std::string> sensor_frames,
       	float min_range, float max_range, ros::NodeHandle *nh)
    : SlaveDevice(read_b, write_b, addr, nh)
{
    //Publications
    pub_distance1 = p_node_handle->advertise<sensor_msgs::Range>(pub_topics[0], 1);
    pub_distance2 = p_node_handle->advertise<sensor_msgs::Range>(pub_topics[1], 1);
    pub_distance3 = p_node_handle->advertise<sensor_msgs::Range>(pub_topics[2], 1);

    sensor1.radiation_type = sensor_msgs::Range::ULTRASOUND;
    sensor1.min_range = min_range;
    sensor1.max_range = max_range;
    sensor1.header.frame_id = sensor_frames[0];

    sensor2.radiation_type = sensor_msgs::Range::ULTRASOUND;
    sensor2.min_range = min_range;
    sensor2.max_range = max_range;
    sensor2.header.frame_id = sensor_frames[1];

    sensor3.radiation_type = sensor_msgs::Range::ULTRASOUND;
    sensor3.min_range = min_range;
    sensor3.max_range = max_range;
    sensor3.header.frame_id = sensor_frames[2];
}

bool UltrasonicData::readDevice()
{
    unsigned char buffer[read_bytes] = {0};

    if( i2c_ioctl_read(&device, 0x0, buffer, read_bytes) == read_bytes )
    {
	//sensor1.range = buffer[0] + (buffer[1] << 8) + buffer[2]/100.0F;
	//sensor2.range = buffer[3] + (buffer[4] << 8) + buffer[5]/100.0F;
	//sensor3.range = buffer[6] + (buffer[7] << 8) + buffer[8]/100.0F;
	
	sensor1.range = float(buffer[0]) + float(buffer[1] << 8);
	sensor2.range = float(buffer[2]) + float(buffer[3] << 8);
	sensor3.range = float(buffer[4]) + float(buffer[5] << 8);

	pub_distance1.publish(sensor1);
	pub_distance2.publish(sensor2);
	pub_distance3.publish(sensor3);
	return true;
    }
    else
	return false;
}





