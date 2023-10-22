#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from ruamel.yaml import YAML
from geometry_msgs.msg import PointStamped
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from sort import Sort

yaml = YAML()
restart_script = False

def np_to_points(np_array):
    x1, y1, x2, y2, id = np_array
    # TODO: Named tuple
    center = ((x1 + x2) / 2, (y1 + y2) / 2)
    size = (abs(x1 - x2) + abs(y1 - y2)) / 2  # average of width and height
    size /= 2  # radius
    return center, size, int(id - 1)

def blob_detect(frame):
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.minThreshold = 200
    params.maxThreshold = 255
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 300
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.2
    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.5
    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.5

    hsv_mask = (
                (0, 0, 220),
                (255, 255, 255)
                )
    
    detector = cv2.SimpleBlobDetector_create(params)

    label_kwargs = {
                    'fontFace': cv2.FONT_HERSHEY_SIMPLEX,
                    'fontScale': 0.75,
                    'color': (0, 0, 255),
                    'thickness': 2,
                    }
    
    blurred = cv2.GaussianBlur(frame, (3, 3), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_mask[0], hsv_mask[1])
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)  # Erode and dilate are used to remove excess small blobs.
    mask = cv2.bitwise_not(mask)

    keypoints = detector.detect(mask)

    # Draw detected blobs as red circles and store positions.
    for kpt in keypoints:
        cnt = (int(kpt.pt[0]), int(kpt.pt[1]))
        cv2.circle(frame, cnt, int(kpt.size / 2), (0, 255, 255), 2)
        frame = cv2.putText(frame, f'({int(0.112*cnt[0] + 1.79)}, {int(-0.113*cnt[1] + 69.9)})', (cnt[0] + 40, cnt[1]), **label_kwargs)

    return keypoints, frame, mask

class BlobTracker(object):
    def __init__(self):
        self.num_robots = rospy.get_param('/num_of_robots', 3)
        self.robot_name = rospy.get_param('/robot_name', 'sphero')

        # Create publishers for positions.
        self.pos_pubs = [rospy.Publisher(f'/{self.robot_name}_{i}/position', PointStamped, queue_size=1)
                         for i in range(self.num_robots)]

        # Create publishers for sending color commands.
        self.color_pubs = [rospy.Publisher(f'/{self.robot_name}_{i}/set_color', ColorRGBA, queue_size=1)
                           for i in range(self.num_robots)]

        rospy.sleep(0.5)

        # Initially turn off LEDs on all robots.
        for pub in self.color_pubs:
            pub.publish(0.0, 0.0, 0.0, 0.0)
            print('turning off all lights')

        self.tracker = Sort(max_age=5, min_hits=1, iou_threshold=-1)
        self.freq = rospy.get_param('/data_stream_freq',1)

        # Initialize ROS topics and CvBridge.
        self.cv_bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
    
    def image_callback(self, data):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            rospy.logerr("Error converting ROS image message to OpenCV: %s" % str(e))
            return

        height, width, channels = frame.shape
        # Noetic integer conversion
        height = int(height)
        width = int(width)
        channels = int(channels)

        frame = frame[70:height-60,200:width-210]

        # Detect blobs in the frame.
        blobs, frame, mask = blob_detect(frame)

        # Prepare detections for tracking.
        detections = np.ndarray((len(blobs), 5))
        for i, blob in enumerate(blobs):
            score = 1
            bbox_size = blob.size / 2
            x1 = blob.pt[0] - bbox_size
            x2 = blob.pt[0] + bbox_size
            y1 = blob.pt[1] - bbox_size
            y2 = blob.pt[1] + bbox_size
            detections[i] = np.array([x1, y1, x2, y2, score])

        # Track blobs using SORT.
        tracked, frame = self.tracker.update(detections, frame)
        ok = True

        for obj in tracked:
            cnt, size, id = np_to_points(obj)
            if id >= self.num_robots:
                rospy.loginfo_throttle(5,
                                       "\033[33mOh no! "
                                       "Tracking algorithm returned ID larger than the number of robots. "
                                       "Assigned positions may be incorrect.\n"
                                       "Restart the Gazebo sim and this script.\033[0m")
                ok = False

            # Draw detections on the camera feed.
            image_cnt = int(cnt[0]), int(cnt[1])
            cv2.circle(frame, image_cnt, 5, (0, 0, 255), -1)
            label_kwargs = {
                    'fontFace': cv2.FONT_HERSHEY_SIMPLEX,
                    'fontScale': 0.75,
                    'color': (0, 0, 255),
                    'thickness': 2,
                    }
            frame = cv2.putText(frame, f'ID={int(id)}', (image_cnt[0] + 40, image_cnt[1] + 40),
                                **label_kwargs)
            
            # Publish real world coordinates.
            if ok:
                msg = PointStamped()
                msg.point.x = 0.112*cnt[0] + 1.79
                msg.point.y = -0.113*cnt[1] + 69.9
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'sphero_' + str(id)
                self.pos_pubs[id].publish(msg)

        # Show the frames
        cv2.imshow('Camera', frame)
        #cv2.imshow('Mask', mask)
        #cv2.resizeWindow('Camera', frame.shape[1], frame.shape[0])  # Resize based on frame size
        #cv2.resizeWindow('Mask', mask.shape[1], mask.shape[0])  # Resize based on mask size

        key = cv2.waitKey(1) & 0xFF

    def run(self):
        r = rospy.Rate(self.freq)
        led_initialized = [False] * self.num_robots
        led_countdown = [2 * self.freq] * self.num_robots

        ok = True

        while not rospy.is_shutdown():
            # Turn on LEDs one by one.
            if not all(led_initialized):
                for i in range(self.num_robots):
                    if led_countdown[i] > 0:
                        led_countdown[i] -= 1
                        break
                    else:
                        self.color_pubs[i].publish(0.0, 1.0, 0.0, 1.0)
                        led_initialized[i] = True
                        print('turning on light for sphero_' + str(i))

            r.sleep()

    def shutdown(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("blob_tracker")

    try:
        node = BlobTracker()
        rospy.on_shutdown(node.shutdown)
        node.run()
        node.shutdown()
    except rospy.ROSInterruptException:
        pass
