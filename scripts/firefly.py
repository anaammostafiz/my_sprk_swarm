#!/usr/bin/env python3

import rospy
import sys
import cv2
import random
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA

class LightSynchronization:
    def __init__(self, sphero_num, color):
        rospy.init_node('firefly_{}'.format(sphero_num), anonymous=True)
        #self.robot_number = sphero_num
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
        self.light_pub = rospy.Publisher('/sphero_{}/set_color'.format(sphero_num), ColorRGBA, queue_size=10)
        self.bridge = CvBridge()
        
        # Initialize period
        self.initial_period = random.randrange(2,5,1)
        self.current_period = self.initial_period
        self.last_synchronization_time = rospy.Time.now()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting Image message: {}".format(str(e)))
            return

        if self.detect_synchronization_signal(cv_image):
            # Decrease period slightly upon synchronization
            self.current_period -= 0.01

        # Flash the LED with the current period
        if rospy.Time.now() - self.last_synchronization_time > rospy.Duration(self.current_period):
            # Set the desired LED color (e.g., red)
            if color == 'red':
                self.light_pub.publish(r = 1)
            elif color == 'green':
                self.light_pub.publish(g = 1)
            elif color == 'blue':
                self.light_pub.publish(b = 1)
            else:
                self.light_pub.publish(g = 1)
            
            rospy.loginfo('sphero_' + str(sphero_num) + ' blinked')
            self.last_synchronization_time = rospy.Time.now()
        else:
            self.light_pub.publish(0,0,0,0)

    def detect_synchronization_signal(self, image):

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (41, 41), 0)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
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
        light_sync = LightSynchronization(sphero_num,color)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
