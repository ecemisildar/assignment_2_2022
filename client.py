#!/usr/bin/env python
# coding: utf-8

# In[1]:


#import jupyros as jr
import rospy
import actionlib
import actionlib.msg
from assignment_2_2022.msg import PlanningAction, PlanningGoal 
import ipywidgets as widgets
import time



def cancel(x):
    """
    Cancel all active goals
    """
    client.cancel_goal()

def go(x):
    """
    Prompt the user to enter the x and y coordinates of the goal point, 
    and then send the goal to the action server
    """
    # Assign the x and y positions as the target position of the goal
    goal = PlanningGoal()
    goal.target_pose.pose.position.x = goal_x.value
    goal.target_pose.pose.position.y = goal_y.value

    # Send the goal to the action server
    client.send_goal(goal)


    
        
	
time.sleep(2)
# Initialize the node
rospy.init_node('client')


client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
client.wait_for_server() #maybe comment

goal_x = 0.0
goal_y = 0.0

set_goal = widgets.Button(
    value = False,
    description = 'Set',
    button_style = '', # 'success', 'info', 'warning', 'danger' or ''
    disabled = False,
    )

set_goal.on_click(go)

cancel_goal = widgets.Button(
value = False,
description = 'Cancel',
button_style = '', # 'success', 'info', 'warning', 'danger' or ''
disabled = False,
)

cancel_goal.on_click(cancel)

goal_x = widgets.FloatText(
description = 'X:',
disabled = False,
)

goal_y = widgets.FloatText(
description = 'Y:',
disabled = False,
)

display(set_goal, cancel_goal, goal_x, goal_y)


# In[ ]:





# In[ ]:




