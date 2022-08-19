#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, NavSatFix, LaserScan, Range
from tf.transformations import euler_from_quaternion
import math
import time
from geometry_msgs.msg import Twist
from geographiclib.geodesic import Geodesic
yaw =0 
global i
global p
i=float(input('latitude : '))   # destination
p=float(input('longitude : '))
	
def calc_goal(origin_lat, origin_long, goal_lat, goal_long):   #doubtful
        geod = Geodesic.WGS84  # define the WGS84 ellipsoid
        g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
        hypotenuse = distance = g['s12'] 
        global azimuth 
        azimuth = g['azi1']
        if azimuth<0 :
        	azimuth = 360 + azimuth
        #azimuth = math.radians(azimuth)
        
        imdata =  rospy.Subscriber('/imu', Imu, imu)
        return azimuth

def imu(pose):
    global yaw
    yaw = math.atan2(2.0*(pose.orientation.y*pose.orientation.z + pose.orientation.w*pose.orientation.x), pose.orientation.w*pose.orientation.w - pose.orientation.x*pose.orientation.x - pose.orientation.y*pose.orientation.y + pose.orientation.z*pose.orientation.z)
    yaw=yaw*180/math.pi
    if yaw < 0:
        yaw=yaw+360
    #yaw = abs(yaw-360)
    #yaw = yaw%360
    print(yaw)
    print('Azimuth : ', azimuth)
    v=distanceInKmBetweenEarthCoordinates(lat1, lon1, i, p)
    print(v)
    msg = Twist()
    if lat1!=i and lon1!=p: 
        if (yaw < azimuth-5) :
            msg.angular.z=0.5
            msg.linear.x=0
            print('1')
        elif (yaw > azimuth+5) :
            msg.angular.z=-0.5
            msg.linear.x=0
            print('2')
        else:
            msg.angular.z=0
            msg.linear.x=-0.5
            print('3')
    else :
        msg.angular.z=0
        msg.linear.x=0
        print('4')
    pub.publish(msg)
	     
def gps(data):
	global lat1 
	lat1= data.latitude
	global lon1 
	lon1= data.longitude
	calc_goal(lat1, lon1, i, p)

def degreesToRadians(degrees) :
	return degrees * 3.147 / 180

def distanceInKmBetweenEarthCoordinates(lat1, lon1, lat2, lon2):
        earthRadiusKm = 6371
        dLat = degreesToRadians(lat2-lat1)
        dLon = degreesToRadians(lon2-lon1)
        lat1 = degreesToRadians(lat1)
        lat2 = degreesToRadians(lat2)
        a = math.sin(dLat/2) * math.sin(dLat/2) + math.sin(dLon/2) * math.sin(dLon/2) * math.cos(lat1) * math.cos(lat2) 
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return earthRadiusKm * c
        
if __name__ == '__main__':
	rospy.init_node('topic_subscriber')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	sub =  rospy.Subscriber('/sensor_msgs/NavSatFix', NavSatFix, gps)
	rospy.spin()
