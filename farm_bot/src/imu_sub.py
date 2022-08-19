#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

def callback(msg):
	yaw = math.atan2(2.0*(msg.orientation.y*msg.orientation.z + msg.orientation.w*msg.orientation.x), msg.orientation.w*msg.orientation.w - msg.orientation.x*msg.orientation.x - msg.orientation.y*msg.orientation.y + msg.orientation.z*msg.orientation.z);
	pitch = math.asin(-2.0*(msg.orientation.x*msg.orientation.z - msg.orientation.w*msg.orientation.y));
	roll = math.atan2(2.0*(msg.orientation.x*msg.orientation.y + msg.orientation.w*msg.orientation.z), msg.orientation.w*msg.orientation.w + msg.orientation.x*msg.orientation.x - msg.orientation.y*msg.orientation.y - msg.orientation.z*msg.orientation.z);
	print ('Yaw angle :- ',yaw)
	print ('\tPitch:- ',pitch)
	print ('\tRoll:- ',roll)
	
	
rospy.init_node('topic_subscriber')
sub =  rospy.Subscriber('imu', Imu, callback)
rospy.spin()
