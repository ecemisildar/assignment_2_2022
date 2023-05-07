#!/usr/bin/env python3

"""
.. module:: client
   :platform: Unix
   :synopsis: Python module for action client
.. moduleauthor:: Ecem Isildar

Ros Node for taking user input and canceling or adjusting goal position

Client action:
/reaching_goal

Publisher:
/goal_message
"""

import rospy
import time
import actionlib # Import the actionlib library for creating and using actions
import actionlib.msg # Import the message class for the action server
from assignment_2_2022.msg import PlanningAction, PlanningGoal # Import the custom message classes for the action
from geometry_msgs.msg import Point # Import the Point message class from the geometry_msgs package


class GoalClient:
    """
    A class used to ask, cancel and publish the goal positions.

    Attributes:
        client (:obj:`actionlib.SimpleActionClient`): The action client for the '/reaching_goal' action server.
        goal_pub (:obj:`rospy.Publisher`): The publisher for the 'goal_message' topic.
        goal_x (float): The x coordinate of the goal point.
        goal_y (float): The y coordinate of the goal point.
    """

    def __init__(self):
        """
        Initialize the 'GoalClient' class.
        This creates the action client and the publisher for goal positions.
        """
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        
        #Define the publisher for publishing the goal positions
        self.goal_pub = rospy.Publisher('goal_message', Point, queue_size=10)
        

    def cancel_goal(self):
        """
        Cancel all active goals
        """
        self.client.cancel_goal()
	
    def ask_goal(self):
        """
        Prompt the user to enter the x and y coordinates of the goal point, 
        and then send the goal to the action server
        """
		
        # Ask the user for the goal point
        goal_x = input("Enter the x coordinate of the goal point: ")
        self.goal_x = float(goal_x)
        goal_y = input("Enter the y coordinate of the goal point: ")
        self.goal_y = float(goal_y)

        # Assign the x and y positions as the target position of the goal
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = self.goal_x
        goal.target_pose.pose.position.y = self.goal_y

        # Send the goal to the action server
        self.client.send_goal(goal)
	
    def publish_goals(self):
        """
        Publish the goal position as a Point message on the 'goal_message' topic
        """
        # Create a message with the goal position
        msg = Point()
        msg.x = self.goal_x
        msg.y = self.goal_y
        # Publish the message
        self.goal_pub.publish(msg)

    
def client_control():
    """
    Initializes the node, creates the goal client object and also ask for user input,
    according to the user input it publish the goal position or cancels the position
    """
    # Create the goal client object
    client = GoalClient()

    # Initialize the node
    rospy.init_node('client')

    # Set the loop rate
    rate = rospy.Rate(10)

    # Run the loop
    while not rospy.is_shutdown():
        # Ask the user if they want to set a new goal or cancel the current goal
        action = input("Enter 'g' to set a new goal or 'c' to cancel the current goal: ")
        if action == 'g':
            client.ask_goal()
        elif action == 'c':
            client.cancel_goal()

        # Publish the current goal position
        client.publish_goals()

        # Sleep for the loop rate
        rate.sleep()

if __name__ == "__main__":
    client_control()


