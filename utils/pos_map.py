#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
import message_filters
from PIL import Image, ImageDraw
import numpy as np

# Dictionary to store robot positions
sphero_locs = {}

# Array to store cumulative robot positions
cumulative_positions = []

# Get the total number of robots (N) as a parameter or constant
N = rospy.get_param('/num_of_robots', 7)

# Image size
image_size = (1000, 700)

# Circle radius
circle_radius = 21

# Create an image for plotting
plot_image = Image.new('RGB', (image_size[0], image_size[1]), color='black')
draw = ImageDraw.Draw(plot_image)

def callback(*pos_datas):
    global sphero_locs
    for id, data in enumerate(pos_datas, start=0):
        sphero_locs[id] = [int(data.point.x)*10, int(data.point.y)*10]

    # Append cumulative positions
    cumulative_positions.append(sphero_locs.copy())

def save_image():
    global cumulative_positions, plot_image, draw

    # Clear the plot image
    plot_image = Image.new('RGB', (image_size[0], image_size[1]), color='black')
    draw = ImageDraw.Draw(plot_image)

    # Plot the cumulative positions of all robots
    for positions in cumulative_positions:
        for loc in positions.values():
            draw.ellipse([loc[0] - circle_radius, loc[1] - circle_radius,
                          loc[0] + circle_radius, loc[1] + circle_radius], fill='white')

    # Save the plot as an image
    plot_image.save('robot_positions.png')

def main():
    rospy.init_node('plotter', anonymous=False)

    # Create subscribers for all robot position topics
    pos_subs = [message_filters.Subscriber(f'sphero_{i}/position', PointStamped) for i in range(0, N)]

    # Synchronize the position messages from all robots
    ts = message_filters.ApproximateTimeSynchronizer(pos_subs, 1, 1)
    ts.registerCallback(callback)

    try:
        while not rospy.is_shutdown():
            user_input = input("Press 'k' to save the image: ")
            if user_input.lower() == 'k':
                save_image()
                print("Image saved as robot_positions.png")
                break

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt detected. Saving final robot positions image.")
        save_image()

if __name__ == '__main__':
    main()
