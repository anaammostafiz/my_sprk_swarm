#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
import message_filters
from math import atan2, sqrt 

predator = sys.argv[1]
prey = sys.argv[2]

node_name = 'hunting_' + str(predator) + '_' + str(prey)
rospy.init_node(node_name,anonymous=False)
rate = rospy.Rate(10)

def callback(prey_data,predator_data):
    msg = Twist()
    dist = sqrt( (prey_data.vector.x - predator_data.vector.x)**2 + (prey_data.vector.y - predator_data.vector.y)**2 )
    print('dist: ' + str(dist))
    if dist >= 20:
        x_vel = 0.1 * (prey_data.vector.x - predator_data.vector.x)
        y_vel = 0.05 * (prey_data.vector.y - predator_data.vector.y)
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

    cx_prey = (prey_data.vector.x - 1.79)/0.112
    cy_prey = (prey_data.vector.y - 69.9)/-0.113
    cx_predator = (predator_data.vector.x - 1.79)/0.112
    cy_predator = (predator_data.vector.y - 69.9)/-0.113

    rate.sleep()

cmd_topic = '/sphero_' + str(predator) + '/cmd_vel'
pub = rospy.Publisher(cmd_topic,Twist,queue_size=10)
prey_topic = '/sphero_' + str(prey) + '/position'
prey_sub = message_filters.Subscriber(prey_topic, Vector3Stamped)
predator_topic = '/sphero_' + str(predator) + '/position'
predator_sub = message_filters.Subscriber(predator_topic, Vector3Stamped)

ts = message_filters.ApproximateTimeSynchronizer([prey_sub,predator_sub],1,1)
ts.registerCallback(callback)
rospy.spin() 