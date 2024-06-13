# Ros Action Server for Motion Planning

The project aims to write an action server to take input from the user to set a goal 
position or cancel it whenever user desires. The action server and the motion planning algorithms are already given. 
Four nodes are developed by me as follows:
Node A: client
Node A: publisher
Node B: goalTracker
Node C: subscriber

Node A: Is an action client which takes command from the server part bug-0 and the pseudocode of this node is as follows:


    class GoalClient
    
    def init():
    
      #create action client
      #goal position publisher
      
    def cancel_goal():
      #cancel_goal()
      
    def ask_goal():
      #ask user for the goal points
      #assign x and y positions to the PlanningGoal()
      #in order to access x and y
      #goal.target_pose.pose.position.x
      #goal.target_pose.pose.position.y
      #then send the goal to the server 
      
    def publish_goals(): 
      #create a standard message called point
      #assign x and y target points
      #publish

    while not shutdown():
      #ask input
      #if it is g call ask goal
      #if it is c call cancel goal
  
  
The remaining part of the node A just publishes the current position of the robot. It has a structure like below:
  
    def odom callback():
      #extracts position and velocity data from the topic
      #then creates a custom message called RobotState
      #then assign these variables to the custom message
      
After that,  there is a node initialization and odometry subscriber





















