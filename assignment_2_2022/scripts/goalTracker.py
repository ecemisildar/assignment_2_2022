#!/usr/bin/env python3

"""
.. module:: publisher
   :platform: Unix
   :synopsis: Python module for publishing the number of goals reached and cancelled
.. moduleauthor:: Ecem Isildar

This module contains a ROS node that provides a custom service to count how many goals have been reached
and how many have been cancelled. It also publishes the count as a message on a specific topic.

Service:
/Goal_count

Publisher:
/robot_state
"""

import rospy
from assignment_2_2022.srv import GoalCount, GoalCountResponse
from assignment_2_2022.msg import PlanningActionFeedback


# global variables to count how many goals have been reached and canceled
reached_goal = 0
cancelled_goal = 0

def check_feedback(data):
    """
    Callback function for subscribing to the feedback message of the robot's goal.
    
    This function increments the count of either the reached goals or cancelled goals based on the message received.

    Args:
        data (PlanningActionFeedback): The feedback message received from the robot.

    Returns:
        None
    """
    global reached_goal, cancelled_goal
    # check the feedback message 
    if data.feedback.stat == "Target cancelled!":
       cancelled_goal += 1
       print("Cancelled goal: {}".format(cancelled_goal))
    elif data.feedback.stat == "Target reached!":
       reached_goal += 1
       print("Reached goal: {}".format(reached_goal))
       
def handle_goal_tracker(req):
    """
    Service handler function for the custom service that provides the count of reached and cancelled goals.

    This function handles requests for the custom service by returning the current count of reached and cancelled goals.

    Args:
        req (GoalCountRequest): The request message received by the service.

    Returns:
        GoalCountResponse: The response message containing the current count of reached and cancelled goals.
    """

    global reached_goal, cancelled_goal
    # create variable of the custom service
    data = GoalCountResponse()
    data.reached = reached_goal 
    data.cancelled = cancelled_goal
    # print reached and cancelled goals
    print("Goals reached: {} Goals canceled: {}".format(reached_goal,cancelled_goal))
    
    return data
    


def main():
    """
    Main function to start the ROS node for goal tracking.
    
    This function initializes the ROS node and starts the custom service for goal tracking. It also subscribes to the feedback
    message from the robot's goal and calls the check_feedback function for each message received.
    
    Args:
        None

    Returns:
        None
    """
    rospy.init_node('goalTracker')
    # create a service that publishes the the goal position of the robot
    srv = rospy.Service('Goal_count', GoalCount, handle_goal_tracker)
    # subscribe the feedback of the goal to check which message is written
    rospy.Subscriber("/reaching_goal/feedback", PlanningActionFeedback, check_feedback)

    rospy.spin()


if __name__ == '__main__':
    main()

