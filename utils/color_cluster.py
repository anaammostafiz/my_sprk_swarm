#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from sklearn.cluster import KMeans
from std_msgs.msg import ColorRGBA
from sphero_swarm.msg import TargetColorInfo  # Import your custom message
import time

class ColorAnalysisNode:
    def __init__(self):
        rospy.init_node('color_analysis_node')
        self.cv_bridge = CvBridge()

        # Subscribe to the /image_raw topic
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
        
        # Initialize variables
        self.dominant_colors = None
        self.target_colors = None
        self.is_first_image = True

        # Create a publisher for color information
        self.color_pub = rospy.Publisher('/target_colors', TargetColorInfo, queue_size=1)

    def image_callback(self, data):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            rospy.logerr("Error converting ROS image message to OpenCV: %s" % str(e))
            return
        
        # Convert the image to HSV
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        if self.is_first_image:
            start_time = time.time()
            (h,w,c) = rgb_image.shape
            #print(rgb_image.shape)
            # Perform k-means clustering on the mask to find initial dominant colors
            flattened_pixels = rgb_image.reshape((h*w, c))
            kmeans = KMeans(n_clusters=4,n_init=10)
            kmeans.fit(flattened_pixels)
            self.dominant_colors = kmeans.cluster_centers_.round(0).astype(int)

            # Exclude black as a dominant color
            #self.dominant_colors = [color for color in self.dominant_colors if color[2] > 20]
            exclude_row = np.array([1,1,1])
            match = np.all(self.dominant_colors == exclude_row,axis=1)
            self.dominant_colors = self.dominant_colors[~match]

            # Initialize self.target_colors with self.dominant_colors
            self.target_colors = self.dominant_colors

            self.is_first_image = False
            print("--- %s seconds for clustering ---" % (time.time() - start_time))
        else:
            start_time = time.time()
            # Convert self.target_colors to a NumPy array for efficient indexing
            target_colors_np = np.array(self.target_colors)
            
            # Convert the flattened image pixels to a NumPy array
            flattened_pixels = rgb_image.reshape((-1, 3))
            
            # Calculate a mask to efficiently count the colors
            color_mask = np.all(flattened_pixels[:, None] == target_colors_np, axis=2)
            
            # Calculate proportions and RGB values
            total_pixels = np.count_nonzero(color_mask)
            #print('total_pixels: ' + str(total_pixels))
            proportions = np.sum(color_mask, axis=0) / total_pixels
            r_values = target_colors_np[:, 0]
            g_values = target_colors_np[:, 1]
            b_values = target_colors_np[:, 2]

            # Publish the target color information in a custom message
            self.publish_target_colors(proportions, r_values, g_values, b_values)
            print(self.dominant_colors)
            print("--- %s seconds for counting ---" % (time.time() - start_time))

    def publish_target_colors(self, proportions, r_values, g_values, b_values):
        target_color_msg = TargetColorInfo()
        target_color_msg.header.stamp = rospy.Time.now()

        target_color_msg.color_names = [str(i) for i in range(len(self.target_colors))]
        target_color_msg.proportions = proportions
        target_color_msg.r_values = r_values
        target_color_msg.g_values = g_values
        target_color_msg.b_values = b_values

        self.color_pub.publish(target_color_msg)
        rospy.loginfo(target_color_msg)
        
def main():
    color_analysis_node = ColorAnalysisNode()
    rospy.spin()

if __name__ == '__main__':
    main()