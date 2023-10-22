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
    # Implement logic to command the robots to move to their respective goals
    # You should also implement collision avoidance here

    for id, loc in sphero_locs.items():
        goal_loc = sphero_goals[id]

        # Calculate the desired velocity to move towards the goal
        desired_velocity = calculate_desired_velocity(loc, goal_loc)

        # Publish the desired velocity to control the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = desired_velocity[0]
        cmd_vel.linear.y = desired_velocity[1]
        cmd_publishers[id].publish(cmd_vel)

        # Check if the robot has reached its goal
        if distance_to_goal(loc, goal_loc) < 2:  # You can adjust the threshold as needed
            print(f"Sphero {id} has reached its goal!")

def calculate_desired_velocity(current_loc, goal_loc):
    # Implement logic to calculate the desired velocity based on current and goal locations
    # You can use PID control or other methods for smooth movement

    # Example: simple proportional control
    kp = 0.01
    desired_velocity_x = kp * (goal_loc[0] - current_loc[0])
    desired_velocity_y = kp * (goal_loc[1] - current_loc[1])

    return (desired_velocity_x, desired_velocity_y)

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
