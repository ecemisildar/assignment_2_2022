#!/usr/bin/env python3

"""
.. module:: subscriber
   :platform: Unix
   :synopsis: Python module for subscribing the robot state
.. moduleauthor:: Ecem Isildar

Ros node subscribes to a custom message containing the robot's pose and velocity, and 
prints the distance between the robot's current position and the goal position, as well as 
the robot's average velocity.

The distance and average velocity are calculated when a goal message is received.

The node listens to the 'robot_state' topic for the robot's current position and velocity, 
and the 'goal_message' topic for the goal position.

Subscriber:
/robot_state
/goal_meesage

"""



import rospy
import math
from assignment_2_2022.msg import RobotState
from geometry_msgs.msg import Point

# Initialize global variables for distance and average velocity
distance = 0
avg_speed = 0
current_position = 0
goal_position = 0

def state_callback(msg):
    """
    Callback function for the 'robot_state' subscriber.

    Extracts the position and velocity data from the message and updates the current_position global variable.

    Args:
        msg (RobotState): The message containing the robot's pose and velocity.
    """
    x = msg.x
    y = msg.y
    
    # Update global variable
    global current_position
    current_position = math.sqrt(x**2 + y**2)
    

def goal_callback(goal_msg):
    """
    Callback function for the 'goal_message' subscriber.

    Extracts the goal position data from the message and calculates the distance and average velocity 
    between the current position and goal position.

    Args:
        goal_msg (Point): The message containing the goal position.
    """
    x_ = goal_msg.x
    y_ = goal_msg.y
    
    # Update global variables
    global goal_position, distance, avg_speed
    
    # Calculate distance and average velocity
    goal_position = math.sqrt(float(x_**2 + y_**2))
    distance = abs(goal_position - current_position)
    avg_speed = distance*rospy.get_param("freq")
    
    rospy.loginfo("Received goal position: (%f, %f)", x_, y_)

def listener():
    """
    Initializes the node, subscribes to the 'robot_state' and 'goal_message' topics, and 
    prints the remaining distance and average velocity until the node is shut down.
    """
    # Initialize the ROS node
    rospy.init_node('subscriber')
    
    # Subscribe to the "robot_state" and "goal_message" topics
    rospy.Subscriber('robot_state', RobotState, state_callback)
    rospy.Subscriber('goal_message', Point, goal_callback)

    # Get the rate parameter
    rate = rospy.Rate(rospy.get_param("freq"))

    while not rospy.is_shutdown():
        # Print the remaining distance and average velocity
        rospy.loginfo('Remaining distance: %f', distance)
        rospy.loginfo('Average velocity: %f', avg_speed)
        rate.sleep()

if __name__ == "__main__":
    listener()