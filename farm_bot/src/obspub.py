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
           'left': min(min(msg.ranges[0:100]), 0.5),
           'fleft': min(min(msg.ranges[101:200]), 0.5),
           'front': min(min(msg.ranges[201:300]), 0.5),
           'fright': min(min(msg.ranges[301:400]), 0.5),
           'right': min(min(msg.ranges[401:500]), 0.5)
        }
    global temp
    temp=(-1*(regions['left']+regions['fleft']))+(regions['right']+regions['fright'])+regions['front']
    take_action(regions)
    
    global abs
    abs = regions['left']+regions['fleft']+regions['right']+regions['fright']+regions['front']

    global one
    one=temp/abs

def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''
    
    if temp<0.45 :
        state_description = 'Case - 1: left turn '
        linear_x = -0.4
        angular_z = 0.9
    elif temp>0.55:
        state_description = 'Case - 2: right turn '
        linear_x = -0.4
        angular_z = -0.9
    else :
        state_description = 'Case - 3: front '
        linear_x = -0.6
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
