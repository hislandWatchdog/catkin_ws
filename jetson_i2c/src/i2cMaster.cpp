#include <signal.h> // To catch ctrl-c
#include "dataInterpreter.h"

#define RESET   "\033[0m"
#define RED	"\u001b[31m"	/* Red */
#define CYAN    "\033[36m"      /* Cyan */
#define YELLOW  "\u001b[33m"	/* Yellow */

const char bus_name[] = "/dev/i2c-0";

void signalCallbackHandler(int signum);	
int bus;

int main(int argc, char **argv)
{
    //***ROS Setup***//
    ros::init(argc, argv, "i2cMasterNode");
    ros::NodeHandle nh;

    ros::Rate loopRate(100);

    //Handle keybvoard interrupt Ctrl-c
    signal(SIGINT, signalCallbackHandler);
    
    //Open bus
    if((bus = i2c_open(bus_name)) == -1)
    {
	std::cout << RED << "UNABLE TO OPEN BUS" << RESET << std::endl;
	return EXIT_FAILURE;
    }
    else
	std::cout << CYAN << "BUS SUCCESSFULLY OPENED" << RESET << std::endl;

    //Name all topics
    std::vector<std::string> right_motor_subs = {"duty_cycle_ref_r"};
    std::vector<std::string> right_motor_pubs = {"rpm_r"};

    std::vector<std::string> left_motor_subs = {"duty_cycle_ref_l"};
    std::vector<std::string> left_motor_pubs = {"rpm_l"};

    std::vector<std::string> right_sensors_pubs = {"ultrasonic_dist1", "ultrasonic_dist2",
	"ultrasonic_dist3"};
    std::vector<std::string> right_sensors_frames = {"ult_sen1","ult_sen2","ult_sen3"};

    //Create devices
    MotorData right_motor(3,3,0x1D,right_motor_subs, right_motor_pubs, &nh);
    MotorData left_motor(3,3,0x1B,left_motor_subs, left_motor_pubs, &nh);
    UltrasonicData right_ultrasonics(6,0,0x1E,right_sensors_pubs,right_sensors_frames,30,2000,&nh);

    right_motor.initDevice(bus);
    left_motor.initDevice(bus);
    right_ultrasonics.initDevice(bus);

    while(ros::ok())
    {
	ros::spinOnce();

	right_motor.writeDevice();
	right_motor.readDevice();
	left_motor.writeDevice();
	left_motor.readDevice();
	//right_ultrasonics.readDevice();

	loopRate.sleep();
    }

    return EXIT_SUCCESS;
}

void signalCallbackHandler(int signum) 
{
   std::cout << CYAN << "Closing BUS" << RESET << std::endl;
   i2c_close(bus);

   // Terminate program
   exit(signum);
}

