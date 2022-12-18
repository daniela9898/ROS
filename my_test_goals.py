#Test for goals using navigation stack
#To run this first make a map with slam
#Save the map with the following command:
#rosrun map_server map_saver -f map
#Then launch the navigation stack with the following command:


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseResult
import numpy as np




def goal_point(point):
    goals = MoveBaseGoal()
    goals.target_pose.header.frame_id = 'odom'
    goals.target_pose.pose.position.x = point[0]
    goals.target_pose.pose.position.y = point[1]
    goals.target_pose.pose.position.z = 0
    goals.target_pose.pose.orientation.x = 0
    goals.target_pose.pose.orientation.y = 0
    goals.target_pose.pose.orientation.z = 0
    goals.target_pose.pose.orientation.w = 1
    return goals

if __name__ == '__main__':
    rospy.init_node('nav')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal_position = [[0.5, -0.5],[-0.5, -0.5]]

    for i in range(len(goal_position)): 
        while not rospy.is_shutdown():
            goal = goal_point(goal_position[i])
            client.send_goal(goal)
            client.wait_for_result()
            if client.get_state() == 3:
                print("goal %d reached" %i)
                break
            
