#!/usr/bin/env python3

import rospy
import sys
import cv2
import random
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
import numpy as np


class LightSync:
    def __init__(self, sphero_num):
        rospy.init_node('{}_'.format(color) + '{}_'.format(sphero_num) + 'sync_{}'.format(sync_color), anonymous=False)
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
        self.light_pub = rospy.Publisher('/sphero_{}/set_color'.format(sphero_num), ColorRGBA, queue_size=1)
        self.bridge = CvBridge()
        
        self.natural_frequency = 0.5
        self.phase_shift = 0
        self.phase = 0
        self.last_time = 0
        self.K = 1

    def image_callback(self, data):
        
        flash_val = np.sin(2*self.natural_frequency*np.pi*rospy.get_time() + self.phase_shift)
        if flash_val < 0.9:
            self.light_pub.publish(0,0,0,0)
        else:
            if color == 'red':
                self.light_pub.publish(r = 1)
            elif color == 'green':
                self.light_pub.publish(g = 1)
            elif color == 'blue':
                self.light_pub.publish(b = 1)
            else:
                self.light_pub.publish(g = 1)
            rospy.loginfo('sphero_' + str(sphero_num) + ' flashing')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting Image message: {}".format(str(e)))
            return

        if flash_val < 0.9 and self.detect_sync_color(cv_image):
            delta_t = rospy.get_time() - self.last_time
            self.phase = 2*self.natural_frequency*np.pi*(rospy.get_time()-np.floor(rospy.get_time()))
            self.phase_shift += delta_t * (self.natural_frequency + self.K * np.sin(np.pi/2 - self.phase))
            self.last_time = rospy.get_time()

        

    def detect_sync_color(self, image):

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
        if sync_color == 'red':
            lower_color = np.array([0,173,209])
            upper_color = np.array([33,255,255])
        elif sync_color == 'green':
            lower_color = np.array([32,0,35])
            upper_color = np.array([94,255,133])
        elif sync_color == 'blue':
            lower_color = np.array([72,72,0])
            upper_color = np.array([255,93,255])
        else:
            lower_color = np.array([255,255,255])
            upper_color = np.array([255,255,255])
        
        mask = cv2.inRange(hsv, lower_color, upper_color)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask)
        print(maxVal)
        if maxVal > 50:
            return True
        else:
            return False

if __name__ == '__main__':
    try:
        args = rospy.myargv(sys.argv)
        sphero_num = args[1]
        color = args[2]
        sync_color = args[3]
        light_sync = LightSync(sphero_num)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
