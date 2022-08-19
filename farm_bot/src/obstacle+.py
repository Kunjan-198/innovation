#! /usr/bin/env python3
from numpy import ones
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

def main():
    global pub
    rospy.init_node('reading_laser_1')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/simple_model/laser/scan', LaserScan, clbk_laser)
    rospy.spin()
    
def clbk_laser(msg):
    regions = {
           'r1': min(min(msg.ranges[0:150]), 0.5),
           'r2': min(min(msg.ranges[151:350]), 0.5),
           'r3': min(min(msg.ranges[351:500]), 0.5)
        }
    global temp
    temp=(-1*(regions['left']+regions['fleft']))+(regions['right']+regions['fright'])+regions['front']
    take_action(regions)
    
    global k
    k=regions['r1']-regions['r2']+regions['r3']-regions['r2']

    global one
    one=temp/abs

def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''
    
    if k>0 :
        state_description = 'Case - 1: left turn '
        linear_x = 0.3
        angular_z = 0.9
    elif temp>0.55:
        state_description = 'Case - 2: right turn '
        linear_x = 0.3
        angular_z = -0.9
    else :
        state_description = 'Case - 3: front '
        linear_x = 0.3
        angular_z = 0

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    msg.linear.y=0
    msg.linear.z=0
    msg.angular.y=0
    msg.angular.x=0
    pub.publish(msg)
    
if __name__ == '__main__':
    main()
    state_description = 'case 1 - nothing'
linear_x = 0.6
angular_z = 0
