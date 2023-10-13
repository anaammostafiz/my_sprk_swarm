#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import ColorRGBA
import numpy as np

def rander(n):
    topic = '/sphero_' + str(n) + '/set_color'
    pub = rospy.Publisher(topic, ColorRGBA, queue_size=10)
    node_name = 'rander_' + str(n)
    rospy.init_node(node_name,anonymous=True)
    msg = np.random.uniform(0,1,3)
    for i in range(1,3):
        pub.publish(msg[0],msg[1],msg[2],1)
        rospy.loginfo('sphero ' + str(n) + ' set to ' + str(msg))
        rospy.sleep(1)

    rospy.signal_shutdown("Finished color randomizer")

if __name__ == '__main__':
    args = rospy.myargv(sys.argv)
    sphero_num = args[1]
    
    try: 
        rander(sphero_num)
    except rospy.ROSInterruptException:
        pass