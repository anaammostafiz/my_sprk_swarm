#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import sys

def tracker():
    node_name = 'tracker_' + str(sphero_num)
    rospy.init_node(node_name, anonymous=False)
    sub_topic = 'sphero_' + str(sphero_num) + '/odom'
    sub = rospy.Subscriber(sub_topic,Odometry,odom_callback)
    rospy.spin()

def odom_callback(data):
    msg = PointStamped()
    msg.point.x = data.pose.pose.position.x + float(x_offset)
    msg.point.y = data.pose.pose.position.y + float(y_offset)
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'sphero_' + str(sphero_num)

    rospy.loginfo(msg)
    pub.publish(msg)

if __name__ == "__main__":
    args = rospy.myargv(sys.argv)
    sphero_num = args[1]
    if len(args) > 2:
        x_offset = args[2]
        y_offset = args[3]
    else:
        x_offset = 0
        y_offset = 0

    pub_topic = 'sphero_' + str(sphero_num) + '/position'
    pub = rospy.Publisher(pub_topic,PointStamped,queue_size=10)

    tracker()

    