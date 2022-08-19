#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def callback(msg):
	right=-msg.linear.x-msg.angular.z
	left=-msg.linear.x+msg.angular.z
	print ('left : ',left,'right : ',right)
	pub1.publish(right*0.1)
	pub2.publish(right*0.1)
	pub3.publish(left*0.1)
	pub4.publish(left*0.1)

rospy.init_node('topic_subscriber_t')
pub1 = rospy.Publisher('/simple_model/joint1_position_controller/command', Float64,queue_size=10)
pub2 = rospy.Publisher('simple_model/joint2_position_controller/command', Float64,queue_size=10)
pub3 = rospy.Publisher('simple_model/joint3_position_controller/command', Float64,queue_size=10)
pub4 = rospy.Publisher('simple_model/joint4_position_controller/command', Float64,queue_size=10)
sub = rospy.Subscriber('cmd_vel', Twist, callback)
rospy.spin()
