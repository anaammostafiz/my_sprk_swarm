#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Twist
import message_filters
import math
import numpy as np
import sys

shutdown_in_progress = False

# Dictionary to store robot positions and goals
sphero_locs = {}
sphero_goals = {}

# store publishers
cmd_publishers = {}

# Get the total number of robots (N) as a parameter or constant
N = rospy.get_param('/num_of_robots',7)

turning_angles = np.zeros(N)

# Callback function to update robot positions
def callback(*pos_datas):
    print('called back')
    for id, data in enumerate(pos_datas, start=0):
        sphero_locs[id] = [data.point.x,data.point.y]
    
    global cmd_publishers
    move_robots(cmd_publishers)
    rospy.sleep(0.25)

def move_robots(cmd_publishers):
    # Implement logic to command the robots to move to their respective goals with collision avoidance

    for sphero_id, loc in sphero_locs.items():
        avoid_locs = {loc_id: loc for loc_id, loc in sphero_locs.items() if loc_id != sphero_id}
        # Calculate the desired velocity based on potential fields
        turning_angles[sphero_id] += np.random.uniform(-math.pi/4, math.pi/4)
        desired_velocity = calculate_desired_velocity(loc, turning_angles[sphero_id], avoid_locs)

        # Publish the desired velocity to control the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = desired_velocity[0]
        cmd_vel.linear.y = desired_velocity[1]
        cmd_publishers[sphero_id].publish(cmd_vel)

def calculate_desired_velocity(current_loc, angle, avoid_locs):
    # Implement a simple random walk for the desired velocity
    step_size = np.random.uniform(10,30)
    
    # Calculate the new position based on the random walk
    goal_x = current_loc[0] + step_size * math.cos(angle)
    goal_y = current_loc[1] + step_size * math.sin(angle)
    goal_loc = [goal_x,goal_y]

    # Add repulsive potential to avoid other robots
    attractive_potential = calculate_attractive_potential(current_loc, goal_loc)
    repulsive_potential = calculate_repulsive_potential(current_loc, avoid_locs)
    gradient_x = attractive_potential[0] + repulsive_potential[0]
    gradient_y = attractive_potential[1] + repulsive_potential[1]

    return (gradient_x, gradient_y)

def calculate_attractive_potential(current_loc, goal_loc):
    # Calculate the attractive potential field
    k_att = 0.005
    gradient_x = k_att * (goal_loc[0] - current_loc[0])
    gradient_y = k_att * (goal_loc[1] - current_loc[1])
    return (gradient_x, gradient_y)

def calculate_repulsive_potential(current_loc, avoid_locs):
    # Calculate the repulsive potential field based on robot positions
    k_rep = 2.5  # Repulsion gain
    min_dist = 15
    gradient_x = 0.0
    gradient_y = 0.0

    for robot_id, robot_loc in avoid_locs.items():
        distance = math.sqrt((current_loc[0] - robot_loc[0]) ** 2 + (current_loc[1] - robot_loc[1]) ** 2)
        if distance < min_dist:
            angle = math.atan2(current_loc[1] - robot_loc[1], current_loc[0] - robot_loc[0])
            gradient_x += k_rep * ((1.0 / distance) - (1.0 / min_dist)) * math.cos(angle)
            gradient_y += k_rep * ((1.0 / distance) - (1.0 / min_dist)) * math.sin(angle)

    return (gradient_x, gradient_y)

def on_shutdown():
    global shutdown_in_progress
    shutdown_in_progress = True

    # Stop the robots by sending zero velocity
    for sphero_id in range(0, N):
        cmd_vel = Twist()
        cmd_publishers[sphero_id].publish(cmd_vel)

    rospy.loginfo("Shutting down. Stopping robots.")

def main():
    rospy.init_node('walker',anonymous=False)

    # Register the shutdown callback
    rospy.on_shutdown(on_shutdown)

    # Create a publisher for control commands for each robot
    global cmd_publishers
    for id in range(0, N):
        cmd_topic = f'sphero_{id}/cmd_vel'
        cmd_publishers[id] = rospy.Publisher(cmd_topic, Twist, queue_size=10)

    # Create subscribers for all robot position topics
    pos_subs = [message_filters.Subscriber(f'sphero_{i}/position', PointStamped) for i in range(0, N)]

    # Synchronize the position messages from all robots
    ts = message_filters.ApproximateTimeSynchronizer(pos_subs,1,1)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt detected. Stopping robots.")
        on_shutdown()
        sys.exit(0)