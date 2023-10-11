#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from sklearn.cluster import KMeans


def cluster():

    node_name = 'cluster'
    rospy.init_node(node_name, anonymous=False)
    sub = rospy.Subscriber("/image_raw",Image,image_callback)
    rospy.spin()

def image_callback(data):

    try:
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = CvBridge().imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)

    image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    # Flatten the image into a 2D array of RGB values
    pixels = image_rgb.reshape(-1, 3)

    k = 4
    
    # Perform k-means clustering
    kmeans = KMeans(n_clusters=k)
    kmeans.fit(pixels)

    # Get cluster centers and labels
    cluster_centers = kmeans.cluster_centers_
    labels = kmeans.labels_

    # Define color names for each cluster
    color_names = ['Black', 'Red', 'Green', 'Blue']

    # Create a mask for each cluster
    masks = []
    for i in range(k):
        mask = (labels == i).reshape(cv_image.shape[:2])
        masks.append(mask)

    # Initialize a new image for visualization
    segmented_image = np.zeros_like(cv_image)

    # Initialize a dictionary to store proportions
    proportions = {color_name: 0.0 for color_name in color_names}

    # Assign color names to each cluster and calculate proportions
    for i, color_name in enumerate(color_names):
        mask = masks[i]
        if color_name == 'Black':
            color = [0, 0, 0]
        elif color_name == 'Red':
            color = [255, 0, 0]
        elif color_name == 'Green':
            color = [0, 255, 0]
        elif color_name == 'Blue':
            color = [0, 0, 255]

        # Calculate the proportion of this color in the image
        proportion = np.sum(mask) / np.sum(np.any(masks, axis=0))
        proportions[color_name] = proportion

        print(proportions)
    
if __name__ == '__main__':

    pub_topic = '/color_clusters'
    pub = rospy.Publisher(pub_topic, Vector3Stamped, queue_size=10)
    try:
        cluster()
    except rospy.ROSInterruptException:
        pass

