#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Twist
import message_filters
import math

# Dictionary to store robot positions and goals
sphero_locs = {}
sphero_goals = {}

# store publishers
cmd_publishers = {}

# Get the total number of robots (N) as a parameter or constant
N = rospy.get_param('/num_of_robots',7)

# Define the circle parameters
circle_center = (50, 35)  # Center of the circle
circle_radius = 20  # Radius of the circle
sphero_radii = 3.6

# Callback function to update robot positions
def callback(*pos_datas):
    for id, data in enumerate(pos_datas, start=0):
        sphero_locs[id] = [data.point.x,data.point.y]
    
    # Only assign goals if they haven't been assigned already
    if not sphero_goals:
        assign_goals()
    else:
        global cmd_publishers
        move_robots(cmd_publishers)

def assign_goals():
    global sphero_goals

    # Calculate equally spaced goal points around the circle
    goal_points = []
    for i in range(N):
        angle = 2 * math.pi * i / N
        goal_x = circle_center[0] + circle_radius * math.cos(angle)
        goal_y = circle_center[1] + circle_radius * math.sin(angle)
        goal_points.append((goal_x, goal_y))

    # Create a list of unassigned goal points
    unassigned_goals = list(range(N))

    # Assign each robot to the farthest unique goal point
    for id, loc in sphero_locs.items():
        max_distance = float('-inf')
        assigned_goal = None
        for goal_id in unassigned_goals:
            goal_loc = goal_points[goal_id]
            distance = math.sqrt((loc[0] - goal_loc[0]) ** 2 + (loc[1] - goal_loc[1]) ** 2)
            if distance > max_distance:
                max_distance = distance
                assigned_goal = goal_id
        sphero_goals[id] = goal_points[assigned_goal]
        unassigned_goals.remove(assigned_goal)
    print('goals assigned')

def move_robots(cmd_publishers):
    # Implement logic to command the robots to move to their respective goals with collision avoidance

    for sphero_id, loc in sphero_locs.items():
        goal_loc = sphero_goals[sphero_id]
        avoid_locs = {loc_id: loc for loc_id, loc in sphero_locs.items() if loc_id != sphero_id}
        # Calculate the desired velocity based on potential fields
        desired_velocity = calculate_desired_velocity(loc, goal_loc, avoid_locs)

        # Publish the desired velocity to control the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = desired_velocity[0]
        cmd_vel.linear.y = desired_velocity[1]
        cmd_publishers[sphero_id].publish(cmd_vel)

        # Check if the robot has reached its goal
        distance_to_goal = math.sqrt((loc[0] - goal_loc[0]) ** 2 + (loc[1] - goal_loc[1]) ** 2)
        print(f"sphero_{sphero_id} distance to goal: {round(distance_to_goal,1)}")

def calculate_desired_velocity(current_loc, goal_loc, avoid_locs):
    # Implement logic to calculate the desired velocity based on potential fields
    attractive_potential = calculate_attractive_potential(current_loc, goal_loc)
    repulsive_potential = calculate_repulsive_potential(current_loc, avoid_locs)
    
    # Calculate the gradient of the total potential field
    gradient_x = attractive_potential[0] + repulsive_potential[0]
    gradient_y = attractive_potential[1] + repulsive_potential[1]

    # Return the desired velocity components
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

# Function to calculate the distance to the goal
def distance_to_goal(current_loc, goal_loc):
    return math.sqrt((current_loc[0] - goal_loc[0]) ** 2 + (current_loc[1] - goal_loc[1]) ** 2)

def main():
    rospy.init_node('circler',anonymous=False)

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
