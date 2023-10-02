#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Imu
import sys
from geometry_msgs.msg import Vector3Stamped

def tracker(num,color):

    node_name = 'tracker_' + str(sphero_num)
    rospy.init_node(node_name, anonymous=False)
    sub = rospy.Subscriber("/image_raw",Image,image_callback)
    rospy.spin()

def image_callback(data):

    try:
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = CvBridge().imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
    # We get image dimensions and crop the parts of the image we don't need
    # Bear in mind that because the first value of the image matrix is start and second value is down limit.
    # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
    # To make process faster.
    height, width, channels = cv_image.shape
    # Noetic integer conversion
    height = int(height)
    width = int(width)
    channels = int(channels)

    crop_image = cv_image[70:height-60,200:width-210]
        
    # Convert from RGB to HSV
    hsv = cv2.cvtColor(crop_image, cv2.COLOR_BGR2HSV)
        
    if color == 'red':
        lower_color = np.array([166,47,113])
        upper_color = np.array([255,255,255])
    elif color == 'green':
        lower_color = np.array([32,0,35])
        upper_color = np.array([94,255,133])
    elif color == 'blue':
        lower_color = np.array([0,253,150])
        upper_color = np.array([255,255,255])
    elif color == 'test':
        lower_color = np.array([0,91,135])
        upper_color = np.array([62,255,255])
    else:
        lower_color = np.array([71,177,144])
        upper_color = np.array([92,255,214])
        

    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_color, upper_color)
        
    # Calculate centroid of the blob of binary image using ImageMoments
    m = cv2.moments(mask, False)
    try:
       cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
    except ZeroDivisionError:
        cy, cx = height/2, width/2
        
    rx = 0.112*cx + 1.79
    ry = -0.113*cy + 69.9

    msg = Vector3Stamped()
    msg.vector.x = rx
    msg.vector.y = ry
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'sphero_' + str(sphero_num)

    rospy.loginfo(msg)
    pub.publish(msg)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(crop_image,crop_image, mask= mask)
        
    # Draw the centroid in the resultut image
    # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
    cv2.circle(res,(int(cx), int(cy)), 10,(255,0,0),-1)

    #cv2.imshow("Original", cv_image)
    #cv2.imshow("HSV", hsv)
    #cv2.imshow("MASK", mask)
    #cv2.imshow("RES", res)
    #cv2.imshow("Cropped", crop_image)
        
    cv2.waitKey(1)    
    
if __name__ == '__main__':
    args = rospy.myargv(sys.argv)
    sphero_num = args[1]
    color = args[2]
    pub_topic = '/sphero_' + str(sphero_num) + '/position'
    pub = rospy.Publisher(pub_topic, Vector3Stamped, queue_size=10)
    try:
        tracker(sphero_num,color)
    except rospy.ROSInterruptException:
        pass

