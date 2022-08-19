#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from collections import deque
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
q=deque()
q.append(0)
q.append(0)
q.append(0)
q.append(0)
p=deque()
p.append(0)
p.append(0)
p.append(0)
p.append(0)
r=deque()
r.append(0)
r.append(0)
r.append(0)
r.append(0)
t=deque()
t.append(0)
t.append(0)
t.append(0)
t.append(0)
def callback1(msg):
    global left
    global right
    left=10*(msg.linear.x+msg.angular.z)
    right=10*(msg.linear.x-msg.angular.z)
    sub2 = rospy.Subscriber('simple_model/joint_states',JointState, callback2)
    	
def callback2(msg):
    var2=msg.velocity
    error1=left-var2[0]
    error2=left-var2[1]
    error3=right-var2[2]
    error4=right-var2[3]
    q.popleft()
    q.append(error1)
    p.popleft()
    p.append(error2)
    r.popleft()
    r.append(error3)
    t.popleft()
    t.append(error4)      
    s=np.sum(q)
    Kp=0.001
    Ki=0
    Kd=0
    PID1 = Kp * error1+ Ki * s
    PID2 = Kp * error2+ Ki * s
    PID3 = Kp * error3+ Ki * s
    PID4 = Kp * error4+ Ki * s
    print(left+PID1)
    print(left+PID2)
    print(right+PID3)
    print(right+PID4)
    pub1.publish(left+PID1)
    pub2.publish(left+PID2)
    pub3.publish(right+PID3)
    pub4.publish(right+PID4)
    
pub1 = rospy.Publisher('/simple_model/joint1_position_controller/command', Float64,queue_size=10)
pub2 = rospy.Publisher('simple_model/joint2_position_controller/command', Float64,queue_size=10)
pub3 = rospy.Publisher('simple_model/joint3_position_controller/command', Float64,queue_size=10)
pub4 = rospy.Publisher('simple_model/joint4_position_controller/command', Float64,queue_size=10)
rospy.init_node('PID')
sub = rospy.Subscriber('cmd_vel', Twist, callback1)
rospy.spin()
#PID
