#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import ColorRGBA

def waker(n,c):
    topic = '/sphero_' + str(n) + '/set_color'
    pub = rospy.Publisher(topic, ColorRGBA, queue_size=10)
    node_name = 'waker_' + str(n)
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
        rospy.loginfo('sphero ' + str(n) + ' set to ' + c)
        pub.publish(msg[0],msg[1],msg[2],msg[3])
        rate.sleep()

if __name__ == '__main__':
    args = rospy.myargv(sys.argv)
    sphero_num = args[1]
    color = args[2]
    try: 
        waker(sphero_num,color)
    except rospy.ROSInterruptException:
        pass