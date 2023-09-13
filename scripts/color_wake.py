#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import ColorRGBA

def talker(n,c):
    topic = '/sphero_' + str(n) + '/set_color'
    pub = rospy.Publisher(topic, ColorRGBA, queue_size=10)
    node_name = 'talker_' + str(n)
    rospy.init_node(node_name,anonymous=True)
    rate = rospy.Rate(0.1) # hz
    if c == 'red':
        msg = [1, 0, 0, 0]
    elif c == 'green':
        msg = [0, 1, 0, 0]
    elif c == 'blue':
        msg = [0, 0, 1, 0]
    else:
        msg = [0, 1, 0, 0]
    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg[0],msg[1],msg[2],msg[3])
        rate.sleep()

if __name__ == '__main__':
    sphero_num = sys.argv[1]
    color = sys.argv[2]
    try: 
        talker(sphero_num,color)
    except rospy.ROSInterruptException:
        pass