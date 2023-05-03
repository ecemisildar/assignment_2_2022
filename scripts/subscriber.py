#!/usr/bin/env python3

#Node C
#subscribes custom msg (robot pose and vel) 
#prints robot distance from the target and avg vel of the robot
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
    # Extract the position and velocity data from the message
    x = msg.x
    y = msg.y
    x_vel = msg.linear_vel
    z_vel = msg.angular_vel
    
    # Update global variables
    global current_position
    current_position = math.sqrt(x**2 + y**2)
    

def goal_callback(goal_msg):
    # Extract the goal position data from the message
    x_ = goal_msg.x
    y_ = goal_msg.y
    global goal_position
    goal_position = math.sqrt(float(x_**2 + y_**2))
    # Update global variables
    global  distance, avg_speed
    #distance and average velocity calculation
    distance = abs(goal_position - current_position)
    avg_speed = distance*rospy.get_param("freq")
    
    rospy.loginfo("Received goal position: (%f, %f)", x_, y_)

def listener():
    # Initialize a ROS node and subscribers to the "state_message" and "goal_message" topics
    rospy.init_node('subscriber')
    rospy.Subscriber('robot_state', RobotState, state_callback)
    rospy.Subscriber('goal_message', Point, goal_callback)

    #gets rate as a parameter
    rate = rospy.Rate(rospy.get_param("freq"))

    while not rospy.is_shutdown():
        rospy.loginfo('Remaining distance: %f', distance)
        rospy.loginfo('Average velocity: %f', avg_speed)
        rate.sleep()



if __name__ == "__main__":
    listener()