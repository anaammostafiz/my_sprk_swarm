#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from sphero_swarm.msg import ObjsAndVerts, ObjectVertices
from std_msgs.msg import Header
from geometry_msgs.msg import Point

pub = None

def image_callback(data):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    except Exception as e:
        print("Error processing depth image:", str(e))

    height, width = cv_image.shape
    # Noetic integer conversion
    height = int(height)
    width = int(width)

    crop_image = cv_image[:, 20:width - 10]

    # Normalize the depth image
    crop_image = cv2.normalize(crop_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # Preprocess the depth image (e.g., apply a depth threshold)
    threshold = 50  # Set your depth threshold value (in meters)
    crop_image = np.where(crop_image < threshold, 255, 0).astype(np.uint8)

    contours, _ = cv2.findContours(crop_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    msg = ObjsAndVerts()
    objs_list = []

    n = 1
    # Iterate over detected objects
    for contour in contours:
        # Filter objects based on size
        if cv2.contourArea(contour) > 1000:
            # Approximate the shape of the object by simplifying the contour
            epsilon = 0.02 * cv2.arcLength(contour, True)
            #epsilon = 10
            approx = cv2.approxPolyDP(contour, epsilon, True)

            vertices = [Point(x=vertex[0][0], y=vertex[0][1]) for vertex in approx]
            inner_msg = ObjectVertices()
            inner_msg.vertices = vertices
            inner_msg.header.frame_id = 'object_' + str(n)
            objs_list.append(inner_msg)

            # Draw the vertices of the object
            for vertex in approx:
                x, y = vertex[0]
                cv2.circle(crop_image, (x, y), 5, (127, 127, 127), -1)
            
            # Draw lines connecting the vertices of the object on the image, including a line from the last to the first vertex
            for i in range(len(vertices)):
                x1, y1 = vertices[i].x, vertices[i].y
                if i == len(vertices) - 1:
                    x2, y2 = vertices[0].x, vertices[0].y  # Connect the last vertex to the first vertex
                else:
                    x2, y2 = vertices[i + 1].x, vertices[i + 1].y
                cv2.line(crop_image, (x1, y1), (x2, y2), (127, 127, 127), 2)
        n += 1

    msg.objects = objs_list
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rospy.loginfo(msg)

    cv2.imshow("Depth Image", crop_image)
    cv2.waitKey(1)  # You can adjust the wait time (milliseconds) as needed

def main():
    rospy.init_node("depth_subber")
    global pub
    pub = rospy.Publisher("/objects", ObjsAndVerts, queue_size=10)
    rospy.Subscriber("/camera/depth/image", Image, image_callback)
    rospy.spin()

if __name__ == "__main__":
    main()