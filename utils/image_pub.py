#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

def publish_image():
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('/image_raw', Image, queue_size=10)
    rate = rospy.Rate(10)  # Publish at 10 Hz

    # Create a CV bridge
    bridge = CvBridge()

    # Get the current directory of your script
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Specify the relative or full path to your image file
    image_file = os.path.join(script_dir, '..', 'pics', 'rgb_draw.png')
    # Load the image
    image = cv2.imread(image_file)

    while not rospy.is_shutdown():
        # Convert the image to a ROS Image message
        image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")

        # Publish the image message
        pub.publish(image_msg)
        rospy.loginfo('Publishing image')

        # Sleep to maintain the desired publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass
