#!/usr/bin/env python3

#Node B
#take odom data and publish as a custom msg

import rospy
from nav_msgs.msg import Odometry #to take values to publish a custom msg
from assignment_2_2022.msg import RobotState #for custom msg
	

def odom_callback(msg):
    # Extract the position and velocity data from the message
	rospy.loginfo(msg.pose.pose.position.x)
	x_ = msg.pose.pose.position.x
	y_ = msg.pose.pose.position.y
	linear_vel = msg.twist.twist.linear.x
	angular_vel = msg.twist.twist.angular.z

	# Create a custom message with the position and velocity data
	robot_state = RobotState()
	robot_state.x = x_
	robot_state.y = y_
	robot_state.linear_vel = linear_vel
	robot_state.angular_vel = angular_vel
	pub = rospy.Publisher('robot_state', RobotState, queue_size = 10)
	pub.publish(robot_state)
    
# Initialize the node	
rospy.init_node('publisher')
# Subscribe to the /odom topic
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback) 
# Spin the node to keep it running
rospy.spin()
		



    
    
         
