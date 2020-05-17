#! /usr/bin/env python 

import rospy
from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
	#samples of laser scan over angles (10 for 360)
	"""if no object is detected,the distance will be set as the maximum
	value of the sensor, 12 meters in this case"""
	rospy.loginfo(msg.ranges)
	##msg.ranges is the variable where we can obtain distance to object

def main():
	rospy.init_node("reading_laser")

	sub = rospy.Subscriber("/scan", LaserScan, clbk_laser)

	rospy.spin()

if __name__ == "__main__":
	main()
