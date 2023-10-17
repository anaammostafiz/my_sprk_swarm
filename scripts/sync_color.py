#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import ColorRGBA
from sphero_swarm.msg import TargetColorInfo
import numpy as np

color = None

def syncer():
    rospy.init_node('sync_{}'.format(sphero_num), anonymous=False)
    
    for i in range(1,3):
        pub.publish(color[0],color[1],color[2],1)
        rospy.loginfo('sphero ' + str(sphero_num) + ' initialized to ' + str(color))
        rospy.sleep(1)

    sub = rospy.Subscriber('/target_colors', TargetColorInfo, target_callback)
    rospy.spin()

def target_callback(data):
    global color
    props = np.array(data.proportions)
    major_i = props.argmax()
    major_color = np.array([data.r_values[major_i],data.g_values[major_i],data.b_values[major_i]])
    if np.max(props) != 1 and color is not None and not np.array_equal(color, major_color):
        p = np.random.uniform(0,1)
        if p <= props[0]:
            color = np.array([data.r_values[0],data.g_values[0],data.b_values[0]])
        elif p<= props[0] + props[1]:
            color = np.array([data.r_values[1],data.g_values[1],data.b_values[1]])
        else:
            color = np.array([data.r_values[2],data.g_values[2],data.b_values[2]])

    for i in range(1,3):
        pub.publish(color[0],color[1],color[2],1)
        rospy.loginfo('sphero ' + str(sphero_num) + ' set to ' + str(color))
        rospy.sleep(1)            

if __name__ == '__main__':
    
    args = rospy.myargv(sys.argv)
    sphero_num = args[1]

    color = np.random.uniform(0,1,3)
    pub = rospy.Publisher('/sphero_{}/set_color'.format(sphero_num), ColorRGBA, queue_size=10)

    try:    
        syncer()
    except rospy.ROSInterruptException:
        pass
