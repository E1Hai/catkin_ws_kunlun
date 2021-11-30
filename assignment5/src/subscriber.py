#!/usr/bin/env python

import math
import rospy
import autominy_msgs.msg 
#from autominy_msgs.msg import Speed
from autominy_msgs.msg._Speed import Speed
from autominy_msgs.msg._Tick import Tick
from nav_msgs.msg._Odometry import Odometry
from std_msgs.msg import String
from autominy_msgs.msg._SpeedCommand import SpeedCommand
from autominy_msgs.msg._NormalizedSteeringCommand import NormalizedSteeringCommand

odometry = None
speed = None
tick = None

dist = 0.0
counter = 0.0
helper = 0.0
iterations = 0.0
last = None

def get_odometry(data):
    global odometry
    odometry = data.pose.pose.position
    
def get_speed(data):
    global speed
    speed = data.value
    
def get_tick(data):
    global tick
    tick = data.value
    
    global odometry, speed, dist, helper, last, counter, iterations
    #print(odometry, '\n', speed, '\n', tick, '\n')
    
    if helper <= 1:
        #print('dist = ', dist)
        if dist < 2:
            if last != None:
                n = math.sqrt(abs(last.x - odometry.x)**2 + abs(last.y - odometry.y)**2 + abs(last.z - odometry.z)**2)
                dist += n
                #print('tick = ', tick)
                if tick != 0:
                    #print(n/tick)
                    counter += tick #n/tick
                    #iterations += 1
                #else:
                    #print('infinity')
            last = odometry
        else:
            helper += 0.5
            dist = 0
            print(counter/2) #iterations
            counter = 0
            iterations = 0
    #else:
        #print('finished!')

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print(data)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/simulation/odom_ground_truth", Odometry, get_odometry)
    rospy.Subscriber("/sensors/speed", Speed, get_speed)
    rospy.Subscriber("/sensors/arduino/ticks", Tick, get_tick)
    
    pub_steering = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    global helper
    while not rospy.is_shutdown() and helper <= 1:
        pub_steering.publish(None,helper)
        pub_speed.publish(None,0.3)
        rate.sleep()
    
    #global odometry, speed, tick
    #print(odometry, '\n', speed, '\n', tick, '\n')
    
    rospy.spin()

if __name__ == '__main__':
     try:
         listener()
     except rospy.ROSInterruptException:
         pass
