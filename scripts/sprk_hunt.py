#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import message_filters
from math import atan2, sqrt

predator = sys.argv[1]
prey = sys.argv[2]
sign = sys.argv[3]
s = 1
if sign == 'pos':
    s = 1
elif sign == 'neg':
    s = -1
node_name = 'hunting_' + str(prey)
rospy.init_node(node_name,anonymous=True)
rate = rospy.Rate(0.5)

def callback(prey_data,predator_data):
    msg = Twist()
    dist = sqrt( (prey_data.x - predator_data.x)**2 + (prey_data.y - predator_data.y)**2 )
    print(dist)
    if dist >= 20:
        x_vel = s * 0.1 * (prey_data.x - predator_data.x)
        y_vel = s * 0.025 * (prey_data.y - predator_data.y)
    else: 
        x_vel = 0
        y_vel = 0
    msg.linear.x = x_vel
    msg.linear.y = y_vel
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo(msg)
    rate.sleep()

cmd_topic = '/sphero_' + str(predator) + '/cmd_vel'
pub = rospy.Publisher(cmd_topic,Twist,queue_size=10)
prey_topic = '/sphero_' + str(prey) + '/pose'
prey_sub = message_filters.Subscriber(prey_topic, Pose2D)
predator_topic = '/sphero_' + str(predator) + '/pose'
predator_sub = message_filters.Subscriber(predator_topic, Pose2D)

ts = message_filters.ApproximateTimeSynchronizer([prey_sub,predator_sub],1,1,allow_headerless=True)
ts.registerCallback(callback)
rospy.spin() 