#!/usr/bin/env python3

#Node B custom service that counts how many goals are reached and how many are cancelled
import rospy
from assignment_2_2022.srv import GoalCount, GoalCountResponse
from assignment_2_2022.msg import PlanningActionFeedback


# global variables to count how many goals have been reached and canceled
reached_goal = 0
cancelled_goal = 0

def check_feedback(data):
    
    global reached_goal, cancelled_goal
    # check the feedback message 
    if data.feedback.stat == "Target cancelled!":
       cancelled_goal += 1
       print("Cancelled goal: {}".format(cancelled_goal))
    elif data.feedback.stat == "Target reached!":
       reached_goal += 1
       print("Reached goal: {}".format(reached_goal))
       
def handle_goal_tracker(req):

    global reached_goal, cancelled_goal
    # create variable of the custom service
    data = GoalCountResponse()
    data.reached = reached_goal 
    data.cancelled = cancelled_goal
    # print reached and cancelled goals
    print("Goals reached: {} Goals canceled: {}".format(reached_goal,cancelled_goal))
    
    return data
    


def main():
    rospy.init_node('goalTracker')
    # create a service that publishes the the goal position of the robot
    srv = rospy.Service('Goal_count', GoalCount, handle_goal_tracker)
    # subscribe the feedback of the goal to check which message is written
    rospy.Subscriber("/reaching_goal/feedback", PlanningActionFeedback, check_feedback)

    rospy.spin()


if __name__ == '__main__':
    main()

