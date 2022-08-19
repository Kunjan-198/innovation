#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def callback(msg):
	print (msg)
	left=msg.linear.x*(0.07)+(msg.angular.z*(0.07))
	right=msg.linear.x*(0.07)-(msg.angular.z*(0.07))
	pub1.publish(left)
	pub2.publish(left)
	pub3.publish(right)
	pub4.publish(right)
	
	
rospy.init_node('topic_subscriber')
pub1 = rospy.Publisher('simple_model/joint1_position_controller/command', Float64,queue_size=10)
pub2 = rospy.Publisher('simple_model/joint2_position_controller/command', Float64,queue_size=10)
pub3 = rospy.Publisher('simple_model/joint3_position_controller/command', Float64,queue_size=10)
pub4 = rospy.Publisher('simple_model/joint4_position_controller/command', Float64,queue_size=10)
sub =  rospy.Subscriber('cmd_vel', Twist, callback)
rospy.spin()

