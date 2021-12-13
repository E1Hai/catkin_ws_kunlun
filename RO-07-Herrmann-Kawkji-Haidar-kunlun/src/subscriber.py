#!/usr/bin/env python

import math
import numpy as np
import rospy
import tf2_ros
import autominy_msgs.msg 
import sensor_msgs.point_cloud2
import tf.transformations
from std_msgs.msg import String
from autominy_msgs.msg._SpeedCommand import SpeedCommand
from autominy_msgs.msg._NormalizedSteeringCommand import NormalizedSteeringCommand
from sensor_msgs.msg._LaserScan import LaserScan
from sensor_msgs.msg._PointCloud2 import PointCloud2
from sensor_msgs.msg._JointState import JointState
from geometry_msgs.msg._TransformStamped import TransformStamped
from nav_msgs.msg._OccupancyGrid import OccupancyGrid
from nav_msgs.msg._MapMetaData import MapMetaData
from std_msgs.msg._Header import Header
from nav_msgs.msg._Odometry import Odometry

points = []
#header = None
#pos = None
#rot = None

def callback(data):
    #global header
    #header = Header(seq=data.header.seq , stamp=data.header.stamp , frame_id="map")
    
    global points
    points = []
    for i in range(len(data.ranges)):
        if data.ranges[i] != math.inf:
            angle = data.angle_min + (i * data.angle_increment)
            x = data.ranges[i] * math.cos(angle)
            y = data.ranges[i] * math.sin(angle)
            points.append([x,y,0])
            
"""def get_odometry(data):
    global pos, rot
    pos = data.pose.pose.position
    rot = data.pose.pose.orientation"""
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/sensors/rplidar/scan", LaserScan, callback)
    #rospy.Subscriber("/simulation/odom_ground_truth", Odometry, get_odometry)
    pub_steering = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    #pub_help = rospy.Publisher('/hit_front', PointCloud2, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    width = 0.15
    length = 1
    
    while not rospy.is_shutdown():
        global points, header, pos, rot
        
        if len(points) > 0: #and pos != None and rot != None and header != None:
            
            """position = np.array([pos.x,pos.y,pos.z], dtype=np.float_)
            rotation = np.array([rot.x,rot.y,rot.z,rot.w], dtype=np.float_)
            rotation = tf.transformations.quaternion_matrix(rotation)[0:-1, 0:-1]
            transform = np.array([[0.3,-width,0],[length,-width,0],[0.3,width,0],[length,width,0]  ,  [0,(width+0.5),0],[0,-(width+0.5),0],[-0.3,(width+0.5),0],[-0.3,-(width+0.5),0]], dtype=np.float_)
            for t in range(len(transform)):
                transform[t] = np.array(np.dot(rotation,transform[t]), dtype=np.float_)
                transform[t] = np.array(np.add(transform[t],position), dtype=np.float_)
            
            cloud = sensor_msgs.point_cloud2.create_cloud_xyz32(header, transform)
            pub_help.publish(cloud)"""
            
            obstacle = False
            left , right = 0.0 , 0.0
            pub_speed.publish(None, 0.3)
            
            for point in points:
                if abs(point[1]) <= width and point[0] <= -0.3 and point[0] >= -length:
                    obstacle = True
                    break
            
            new_dist , old_dist_l , old_dist_r = 0.0 , 0.0 , 0.0
            if obstacle:
                #new_dist , new_dist_r , old_dist_r = 0.0 , 0.0 , 0.0
                for i in range(len(points)):
                    if points[i][0] >= 0: 
                        new_dist = math.sqrt(points[i][0]**2 + points[i][1]**2)
                        if points[i][1] <= 0 and points[i][0] <= points[i][1]/2:
                            if old_dist_l < new_dist:
                                old_dist_l = new_dist
                        elif points[i][1] >= 0 and points[i][0] <= -points[i][1]/2:
                            if old_dist_r < new_dist:
                                old_dist_r = new_dist
                    
            if obstacle:
                for point in points:   
                    if point[1] <= 0 and point[1] >= -(width + 0.5) and point[0] >= -0.3 and point[0] <= 0:
                        left += 1 
                    elif point[1] >= 0 and point[1] <= (width + 0.5) and point[0] >= -0.3 and point[0] <= 0:
                        right += 1 
                    
                if old_dist_r < old_dist_l or left > right:
                    pub_steering.publish(None,-1.0)
                else:
                    pub_steering.publish(None,1.0)
            else:
                #print('drive')
                pub_steering.publish(None,0.0)

        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
     try:
         listener()
     except rospy.ROSInterruptException:
         pass
