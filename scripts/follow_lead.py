#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
import message_filters
from math import atan2, sqrt 

follower = sys.argv[1]
leader = sys.argv[2]

node_name = 'leader_' + str(leader) + '_follower_' + str(follower)
rospy.init_node(node_name,anonymous=False)
rate = rospy.Rate(10)

def callback(leader_data,follower_data):
    msg = Twist()
    dist = sqrt( (leader_data.point.x - follower_data.point.x)**2 + (leader_data.point.y - follower_data.point.y)**2 )
    print('dist: ' + str(dist))
    ## real
    # Kx = 0.1
    # Ky = 0.05
    ## gazebo
    # Kx = 0.01
    # Ky = 0.01
    Kx = 0.01
    Ky = 0.01
    if dist >= 20:
        x_vel = Kx * (leader_data.point.x - follower_data.point.x)
        y_vel = Ky * (leader_data.point.y - follower_data.point.y)
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

cmd_topic = '/sphero_' + str(follower) + '/cmd_vel'
pub = rospy.Publisher(cmd_topic,Twist,queue_size=10)
leader_topic = '/sphero_' + str(leader) + '/position'
leader_sub = message_filters.Subscriber(leader_topic, PointStamped)
follower_topic = '/sphero_' + str(follower) + '/position'
follower_sub = message_filters.Subscriber(follower_topic, PointStamped)

ts = message_filters.ApproximateTimeSynchronizer([leader_sub,follower_sub],1,1)
ts.registerCallback(callback)
rospy.spin() 