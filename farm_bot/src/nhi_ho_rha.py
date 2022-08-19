import rospy
import tf
import geodesy.props
import geodesy.utm
import geodesy.wu_point
import geodesy
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import math
from math import atan2,sin,cos,asin,sqrt


#start is x:0, y:0
x = 0.0
y = 0.0
theta = 0.0     #current angle of robot
global angle_to_goal
latitude=49.89999
longitude=8.89999
#import ipdb; ipdb.set_trace()

rospy.init_node ('subscriber')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
speed=Twist()

def callback(msg):
    global x
    global y
    global theta
    x = msg.orientation.x
    y = msg.orientation.y
    rot_q = msg.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    


def navsatfix_callback(latitude,longitude):
    # point = geodesy.utm.fromLatLong(42.546895, -72.609342).toPoint()
    # point = geodesy.utm.fromLatLong(latitude, longitude).toPoint()
    x_1 = float(point.x) - 696400.1
    y_1 = float(point.y) - 4713254.8
    #print(Anatan2(y_1,x_1))
    return atan2(y_1,x_1)
            
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
    print("distance - ",dist)
    
    turn = angle_to_goal - theta
    if angle_to_goal < 0 and theta > 0:
        turn=-1*turn
    print("Theta -" ,theta)
    print("Angle to Goal -", angle_to_goal)
    print("Angle Difference - ",turn)
    
    

    if abs(angle_to_goal - theta) < 0.05:    #0.05 because it too exact for a robot if both angles should be exactly 0
        move_forward = True
    else:
        move_forward=False
        if turn<0:
            a=-1
        else:
            a=1
        speed.angular.z = 0.5 * a
        
        speed.linear.x=0

    if move_forward == True:
        #keep speed between 0.3 and 0.7
        if dist>0.2:
            speed.linear.x=-0.6
            speed.angular.z=0
        else:
            speed.linear.x=0
            speed.angular.z=0
    pub.publish(speed)
    #return dist



def main():
    global angle_to_goal
    sub2=rospy.Subscriber('/imu',Imu , callback)
    #angle_to_goal=navsatfix_callback(latitude,longitude)

    sub3 =rospy.Subscriber('/sensor_msgs/NavSatFix', NavSatFix,distance)

    
    #find out which turndirection is better
    # the bigger the angle, the bigger turn, - when clockwise
    
    
   
    
if __name__ == '__main__':
    global angle_to_goal
    main()
    rospy.Rate(5)
    rospy.spin()









