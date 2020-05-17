#include <ros/ros.h>            // Most common public pieces of the ROS system
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <iostream>
#include <signal.h> // To catch ctrl-c
#include "i2c.h"

#define RESET   "\033[0m"
#define RED	"\u001b[31m"	/* Red */
#define CYAN    "\033[36m"      /* Cyan */
#define YELLOW  "\u001b[33m"	/* Yellow */

typedef struct
{
    int age;
    int brothers;
    int height;
}person;

const unsigned short addr1 = 0x1D;
const unsigned short addr2 = 0x1C;
const unsigned short addr3 = 0x1E;
const char bus_name[] = "/dev/i2c-0";
int bus;

void signalCallbackHandler(int signum);	
void initDevice(I2CDevice *device, const unsigned short addr, const int bus, int write_bytes);
void fillPersonStruct(person *obj, unsigned char *buffer);
bool readDevice(I2CDevice *device, unsigned char *buffer, int len_buf);
bool writeDevice(I2CDevice *device, unsigned char *buffer, int len_buf);
void printData(unsigned char *buffer, int len);
void printData(person *obj);

int main(int argc, char **argv)
{
    //***ROS Setup***//
    ros::init(argc, argv, "i2c_test");
    ros::NodeHandle nh;

    ros::Publisher pub_right_vel = nh.advertise<std_msgs::UInt8>("rpm_right",1);
    ros::Rate loopRate(100);

    std_msgs::UInt8 speed_right;
    
    //***I2C Setup***//
    signal(SIGINT, signalCallbackHandler);

    I2CDevice	device1, device2, device3;

    const int buf_len = 6;
    unsigned char i2c_buffer[buf_len] = {0};

    memset(i2c_buffer, 0, buf_len);
    memset(&device1, 0, sizeof(device1));
    
    // Open i2c bus
    if ((bus = i2c_open(bus_name)) == -1)
    {
	std::cout << RED << "UNABLE TO OPEN BUS" << RESET << std::endl;
	return EXIT_FAILURE;
    }

    // Initiate Device
    initDevice(&device1, addr3, bus, buf_len);

    memset(i2c_buffer, 0, sizeof(i2c_buffer));
    std::cout << "Set to: ";
    printData(i2c_buffer, buf_len);

    while(ros::ok())
    {
	ros::spinOnce();

	//if(writeDevice(&device1, i2c_buffer, 3));
	
	if(readDevice(&device1, i2c_buffer, 6))
	{
	    //fillPersonStruct(&orlando, i2c_buffer);
	    printData(i2c_buffer, 3);
	    //i2c_buffer[1]++;
	    //i2c_buffer[2]++;

	    speed_right.data = i2c_buffer[0];
	}
	else
	    speed_right.data = 0;
	
	pub_right_vel.publish(speed_right);

	loopRate.sleep();
     }

    return EXIT_SUCCESS;
} 

void signalCallbackHandler(int signum) 
{
   std::cout << YELLOW << "\n\nClosing BUS.\n\n" << RESET << std::endl;
   i2c_close(bus);

   // Terminate program
   exit(signum);
}

void initDevice(I2CDevice *device, const unsigned short addr, const int bus, const int write_bytes)
{
    device->bus		= bus;
    device->addr	= addr;
    device->tenbit	= 0;
    //device->delay	= 25;
    device->delay	= 1;
    device->flags	= 0;
    device->page_bytes	= write_bytes;	//This is the number of bytes to write
    device->iaddr_bytes	= 0;
    /*If i2c device do not have internal address,
     * please use i2c_ioctl_read/write function 
     * for read/write, set'iaddr_bytes=0.
     * */
}

void fillPersonStruct(person *obj, unsigned char *buffer)
{
    obj->age	    = int(buffer[0]);
    obj->brothers   = int(buffer[1]);
    obj->height	    = int(buffer[2]);
}

void printData(unsigned char *buffer, int len)
{ 
    int i;

    std::cout << "Data0: " << int(buffer[0]);
    for(i = 1; i < len; i++){
	std::cout << " \tData" << i << ": " << int(buffer[i]);
    }
    std::cout << std::endl;
}  

void printData(person *obj)
{
    std::cout << "Age: " << obj->age
	<< " \tBrothers: " << obj->brothers
	<< " \tHeight: " << obj->height
	<< std::endl;
}

bool readDevice(I2CDevice *device, unsigned char *buffer, int len)
{
    if( (i2c_ioctl_read(device, 0x0, buffer, len)) == len)
	return true;
    else
	return false;
}

bool writeDevice(I2CDevice *device, unsigned char *buffer, int len)
{
    if( (i2c_ioctl_write(device, 0x0, buffer, len)) == len )
	return true;
    else
	return false;
}
