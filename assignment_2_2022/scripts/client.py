#!/usr/bin/env python3

#Node A
#Takes from user input and cancels or adjusts goal position
 
import rospy
import actionlib #for action
import actionlib.msg  
from assignment_2_2022.msg import PlanningAction, PlanningGoal
from geometry_msgs.msg import Point


class GoalClient:
    def __init__(self):
        # Create the action client
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        
        # Define the publisher for publishing the goal positions
        self.goal_pub = rospy.Publisher('goal_message', Point, queue_size=10)
        

    def cancel_goal(self):
        # Cancel all goals
        self.client.cancel_goal()
	
    def ask_goal(self):
		
        # Ask the user for the goal point
        goal_x = input("Enter the x coordinate of the goal point: ")
        self.goal_x = float(goal_x)
        goal_y = input("Enter the y coordinate of the goal point: ")
        self.goal_y = float(goal_y)

        #assigns x and y positions as target
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = self.goal_x
        goal.target_pose.pose.position.y = self.goal_y


        # Send the goal to the action server
        self.client.send_goal(goal)
	
    def publish_goals(self):
        # Create a message with the goal position
        msg = Point()
        msg.x = self.goal_x
        msg.y = self.goal_y
        # Publish the message
        self.goal_pub.publish(msg)

    
    

# Create the action client
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

    client.publish_goals()
    # Sleep for the loop rate
    rate.sleep()
