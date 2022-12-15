#This code is tries to send the position of a mannequin as a goal
#For this turtlebot3_navigation package is needed
#When a goal is published, the robot moves to that position avoiding obstacles

import time
import rospy as ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseResult
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import matplotlib.pyplot as plt

class mannequins_goal:
    def goal_point(self,point):
        goals = MoveBaseGoal()
        goals.target_pose.header.frame_id = 'odom'
        goals.target_pose.pose.position.x = point[0]
        goals.target_pose.pose.position.y = point[1]
        goals.target_pose.pose.position.z = 0
        goals.target_pose.pose.orientation.x = 0
        goals.target_pose.pose.orientation.y = 0
        goals.target_pose.pose.orientation.z = 0
        goals.target_pose.pose.orientation.w = 1
        print("Goal point: ({},{})".format(point[0],point[1]))
        return goals

    def mannequin_position(self, data):
        mannequins = list(data.data)
        A = np.array(mannequins[::2])
        R = np.array(mannequins[1:][::2])

        X = R * np.cos(A)
        Y = R * np.sin(A)
        self.mannequin_pos = np.array([X,Y]).T
        self.mannequin_pos = self.mannequin_pos + np.array([self.robot_x,self.robot_y])
        print(self.mannequin_pos)
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        for i in range(len(self.mannequin_pos)): 
            while not ros.is_shutdown():
                print("Sending goal to mannequin at: ({},{})".format(self.mannequin_pos[i][0],self.mannequin_pos[i][1]))
                goal = self.goal_point(self.mannequin_pos[i])
                print("Sending goal......")
                client.send_goal(goal)
                print("Waiting for result......")
                client.wait_for_result()
                if client.get_state() == 3:
                    print("goal %d reached" %i)
                    break
                
    def robot_position(self, data):
        robot = list(data.pose.pose.position)
        self.robot_x = robot[0]
        self.robot_y = robot[1]
        print("Robot position: ({},{})".format(self.robot_x,self.robot_y))

    def __init__(self):
        #client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #client.wait_for_server()
        #goal_position = [[1.0,0],[-1.0,0]]
        self.mannequin_pos = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        ros.Subscriber("amcl_pose", numpy_msg(Floats), self.robot_position)
        ros.Subscriber("mannequins", numpy_msg(Floats), self.mannequin_position)
        """
        for i in range(len(self.mannequin_pos)): 
            while not ros.is_shutdown():
                print("Sending goal to mannequin at: ({},{})".format(self.mannequin_pos[i][0],self.mannequin_pos[i][1]))
                goal = self.goal_point(self.mannequin_pos[i])
                print("Sending goal......")
                client.send_goal(goal)
                print("Waiting for result......")
                client.wait_for_result()
                if client.get_state() == 3:
                    print("goal %d reached" %i)
                    break
        """
    
    def plot(self):
        while True:
            fig = plt.figure(figsize=(4.10, 4.10), )
            plt.ylim(-3.5, 3.5)
            plt.xlim(-3.5, 3.5)
            #plt.scatter(self.mannequin_pos[:,0],self.mannequin_pos[:,1])
            plt.scatter(self.robot_x,self.robot_y)
            plt.show()
            time.sleep(1)

if __name__ == '__main__':
    ros.init_node('nav')
    mg = mannequins_goal()
    mg.__init__()
    mg.plot()
    ros.spin()
            