#!/usr/bin/env python3

"""
.. module:: publisher
   :platform: Unix
   :synopsis: Python module for publishing the robot state
.. moduleauthor:: Ecem Isildar

Ros node takes the odom data and publishes a custom message containing position and velocity data.

Subscriber:
/odom

Publisher:
/robot_state
"""


import rospy
from nav_msgs.msg import Odometry  # To extract position and velocity data
from assignment_2_2022.msg import RobotState  # Custom message type for position and velocity data


def odom_callback(msg):
    """
    Callback function for the */odom* topic subscriber. Extracts the position and velocity data from the message,
    creates a custom RobotState message and publishes it to the '/robot_state' topic.

    :param msg: Odometry message containing position and velocity data
    :type msg: nav_msgs.msg.Odometry
    """
    rospy.loginfo(msg.pose.pose.position.x)

    # Extract position and velocity data from the message
    x_ = msg.pose.pose.position.x
    y_ = msg.pose.pose.position.y
    linear_vel = msg.twist.twist.linear.x
    angular_vel = msg.twist.twist.angular.z

    #Create a custom message with the position and velocity data
    robot_state = RobotState()
    robot_state.x = x_
    robot_state.y = y_
    robot_state.linear_vel = linear_vel
    robot_state.angular_vel = angular_vel

    #Publish the custom message to the */robot_state* topic
    pub = rospy.Publisher('robot_state', RobotState, queue_size=10)
    pub.publish(robot_state)


def talker():
    """
    Initializes the node, subscribes to the 'odom' topic
    """
    rospy.init_node('publisher')

    # Subscribe to the /odom topic
    odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

    # Spin the node to keep it running
    rospy.spin()

if __name__ == "__main__":
    talker()		



    
    
         
