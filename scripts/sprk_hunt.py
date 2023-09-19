#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import message_filters
from math import atan2, sqrt
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

predator = sys.argv[1]
prey = sys.argv[2]

node_name = 'hunting_' + str(prey)
rospy.init_node(node_name,anonymous=True)
rate = rospy.Rate(0.5)

def callback(prey_data,predator_data,image_data):
    msg = Twist()
    dist = sqrt( (prey_data.x - predator_data.x)**2 + (prey_data.y - predator_data.y)**2 )
    print(dist)
    if dist >= 20:
        x_vel = 0.1 * (prey_data.x - predator_data.x)
        y_vel = 0.05 * (prey_data.y - predator_data.y)
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

    cx_prey = (prey_data.x - 1.79)/0.112
    cy_prey = (prey_data.y - 69.9)/-0.113
    cx_predator = (predator_data.x - 1.79)/0.112
    cy_predator = (predator_data.y - 69.9)/-0.113

    try:
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = CvBridge().imgmsg_to_cv2(image_data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)

    height, width, channels = cv_image.shape
    # Noetic integer conversion
    height = int(height)
    width = int(width)

    crop_image = cv_image[70:height-60,200:width-210]
    
    cv2.circle(crop_image,(int(cx_prey), int(cy_prey)), 10,(128,128,128),-1)
    cv2.circle(crop_image,(int(cx_predator), int(cy_predator)), 10,(128,128,128),-1)
    cv2.line(crop_image,(int(cx_prey), int(cy_prey)),(int(cx_predator), int(cy_predator)), (128,128,128), 9)

    if dist >= 20:
        vel_vecp = (int(cx_predator + 30*x_vel),int(cy_predator - 30*y_vel))
        cv2.arrowedLine(crop_image,(int(cx_predator), int(cy_predator)),vel_vecp, (153,255,255), 4)

    text = str("dist = " + str(round(dist,2)))
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (50,50)
    fontScale = 0.5
    text_color = (255,255,255)
    text_thick = 2
    cv2.putText(crop_image,text,org,font,fontScale,text_color,text_thick)

    window_name = str(predator) + " hunting " + str(prey)
    cv2.imshow(window_name, crop_image)
    cv2.waitKey(1000)

    rate.sleep()

cmd_topic = '/sphero_' + str(predator) + '/cmd_vel'
pub = rospy.Publisher(cmd_topic,Twist,queue_size=10)
prey_topic = '/sphero_' + str(prey) + '/pose'
prey_sub = message_filters.Subscriber(prey_topic, Pose2D)
predator_topic = '/sphero_' + str(predator) + '/pose'
predator_sub = message_filters.Subscriber(predator_topic, Pose2D)
image_sub = message_filters.Subscriber("/image_raw",Image)

ts = message_filters.ApproximateTimeSynchronizer([prey_sub,predator_sub,image_sub],1,1,allow_headerless=True)
ts.registerCallback(callback)
rospy.spin() 