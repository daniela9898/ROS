#Test for goals using navigation stack
#To run this first make a map with slam
#Save the map with the following command:
#rosrun map_server map_saver -f map
#Then launch the navigation stack with the following command:


import rospy as ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseResult
import numpy as np
from std_msgs.msg import Int32MultiArray
import time 

class my_test_goals:

    def goal_point(self,point):
        print(point[0]*0.05,point[1]*0.05)
        goals = MoveBaseGoal()
        goals.target_pose.header.frame_id = 'odom'
        goals.target_pose.pose.position.x = point[0]*0.05
        goals.target_pose.pose.position.y = point[1]*0.05
        goals.target_pose.pose.position.z = 0
        goals.target_pose.pose.orientation.x = 0
        goals.target_pose.pose.orientation.y = 0
        goals.target_pose.pose.orientation.z = 0
        goals.target_pose.pose.orientation.w = 1
        return goals

    def robot_position(self,data):
        self.goal_position = []
        for i in range(0,len(data.data),2):
            self.goal_position.append(data.data[i:i+2])
        #print(self.goal_position)
        #print(self.goal_position)
    
    def points(self):
        
        #self.goal_position = self.goal_position[:2]
        #print("Only first two pos",self.goal_position)

        # for i in range(len(self.goal_position)):
        #     if self.goal_position[i] not in self.visited_mannequins:
        #         self.visited_mannequins.append(self.goal_position[i])
        
        print("Visited mannequins", self.visited_mannequins)

        for i in range(len(self.goal_position)): 
            print(self.goal_position)
            timeout = 0
            while not ros.is_shutdown() and timeout < 10:
                if self.goal_position not in self.visited_mannequins:
                    goal = self.goal_point(self.goal_position[i])
                    self.client.send_goal(goal)
                    self.client.wait_for_result()
                    timeout += 1
                    if self.client.get_state() == 3:
                        print("goal %d reached" %i)
                        break
            if timeout == 40:
                print("Goal not reached, going to next goal")
            self.visited_mannequins.append(self.goal_position[i])

    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        self.visited_mannequins = []
        self.goal_position = []

        self.sub = ros.Subscriber('/mannequins_position', Int32MultiArray, self.robot_position)
        time.sleep(10)
        self.sub.unregister()
        
        
            
if __name__ == '__main__':
    ros.init_node('nav')
    mtg = my_test_goals()
    time.sleep(3)
    mtg.points()
    ros.spin()
