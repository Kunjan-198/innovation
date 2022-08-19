
import rospy
import tf
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import math
from math import atan2,sin,cos,asin,sqrt
from numpy import ones
from sensor_msgs.msg import LaserScan
pub = None

x = 0.0
y = 0.0
theta = 0.0 
latitude=49.9001
longitude=8.8999

speed=Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def main():
    global pub
    rospy.init_node('reading_laser_1')
    sub = rospy.Subscriber('/simple_model/laser/scan', LaserScan, clbk_laser)

def callback(msg):
    global x
    global y
    global theta
    x = msg.orientation.x
    y = msg.orientation.y
    rot_q = msg.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
# distance and angle_of_the_goal           
def distance(msg):
    lat1=msg.latitude
    lon1=msg.longitude
    lat2=latitude
    lon2=longitude
    dy = lat2 - lat1
    dx = math.cos(3.14159/180*lat1)*(lon2 - lon1)
    global angle_to_goal
    global dist
    angle_to_goal = atan2(dy, dx)
    radius = 6365.668 # km
    dlat = math.radians(lat2-lat1)
    dlon = math.radians(lon2-lon1)
    a = (math.sin(dlat/2) * math.sin(dlat/2)) + (math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2) * math.sin(dlon/2))
    c = 2 * math.asin(math.sqrt(a))
    dist = radius * c*1000
    print("distance         ->    ",dist)
    
    # angle difference
    turn = angle_to_goal - theta        
    
    # minimum angle turn
    if abs(turn)>3.14 :
        turn=-1*turn
    
    print("Theta            ->    " ,theta)
    print("Angle to Goal    ->    ", angle_to_goal)
    print("Angle Difference ->    ",turn)
    
    
    # threshold turn (difference) = 0.05
    if abs(angle_to_goal - theta) < 0.05:   
        move_forward = True
    else: 
        # move with only angular velocity
        move_forward=False

        # direction of turn is depend on the sign of turn (difference)
        if turn<0:
            a=-1
        else:
            a=1
        speed.angular.z = 0.5 * a  
        speed.linear.x=0

    if move_forward == True:

        # move with only linear velocity
        # threshold distance = 2.0
        if dist>2.0:
            speed.linear.x=-0.6
            speed.angular.z=0
        else:
            speed.linear.x=0
            speed.angular.z=0
    pub.publish(speed)

def g2g():
    global angle_to_goal
    sub2=rospy.Subscriber('/imu',Imu , callback)
    sub3 =rospy.Subscriber('/sensor_msgs/NavSatFix', NavSatFix,distance)
    

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
    
    
def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''
    
    if temp<0.45 :
        state_description = 'Case - 1: left turn '
        linear_x = -0.7
        angular_z = 0.9
        rospy.loginfo(state_description)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        msg.linear.y=0
        msg.linear.z=0
        msg.angular.y=0
        msg.angular.x=0
        pub.publish(msg)
    elif temp>0.55:
        state_description = 'Case - 2: right turn '
        linear_x = -0.7
        angular_z = -0.9
        rospy.loginfo(state_description)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        msg.linear.y=0
        msg.linear.z=0
        msg.angular.y=0
        msg.angular.x=0
        pub.publish(msg)
    else :
        state_description = 'Case - 3: front '
        g2g()



if __name__ == '__main__':
    main()
    rospy.Rate(5)
    rospy.spin()
