import rospy
import tf
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import math
from math import atan2,sin,cos,asin,sqrt


# start is x:0, y:0 ,initial yaw angle of robot ----> initialization 
left=0.0
right=0.0
PWM=[0,0]
    
rospy.init_node ('subscriber')

# create publisher for send the velocity
pub = rospy.Publisher('/pwm', Int32MultiArray, queue_size=1)
speed=Twist()

# obstacle avoid
def obs_avoid(msg):
    temp=msg.data[4]+(2*msg.data[3])-(2*msg.data[1])-msg.data[0]
    temp=temp/(msg.data[0]+msg.data[1]+msg.data[3]+msg.data[4])
    speed.linear.x=1.2*(1-abs(temp))
    speed.angular.z=1.2*temp
    if(msg.data[2]<50):
        speed.linear.x=-1
        speed.angular.z=1.5
    left = speed.linear.x+speed.angular.z
    right = speed.linear.x-speed.angular.z
    PWM[0]=int(((left+1)/3)*510)
    PWM[1]=int(((right+1)/3)*510)
    pub.publish(PWM)       

def main():
    sub1=rospy.Subscriber('/obs_data',Float32MultiArray,obs_avoid)
    
if __name__ == '__main__':
    main()
    rospy.Rate(5)
    rospy.spin()
