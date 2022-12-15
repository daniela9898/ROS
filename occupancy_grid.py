import math
import time
import rospy as ros
import std_msgs.msg
from nav_msgs.msg import OccupancyGrid
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from typing import Union
from rospy import Subscriber
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseResult

class occupancy_grid:

    def data2matrix(self, data):
        print("Converting data to matrix")
        info = data.info
        d = data.data
        self.width = info.width
        self.height = info.height
        for i in range(self.width):
            for j in range(self.height):
                if d[i*self.width + j] != -1:
                    self.map_matrix[(i,j)] = d[i*self.width + j]
        #print("Robot pos: {}, {}".format(self.robot_x,self.robot_y))
        #print("Matrix robot pos: {}, {}".format(self.width/2 - int(self.robot_x),self.height/2 - int(self.robot_y)))
        self.map_matrix[(self.width/2 - int(self.robot_x),self.height/2 - int(self.robot_y))]= 200
        print("Robot in matrix",self.width/2 - int(self.robot_x), self.height/2 - int(self.robot_y))
        xmin=self.width
        xmax=0
        ymin=self.height
        ymax=0
        print(self.robot_x, self.robot_y)
        for (i,j) in self.map_matrix.keys():
            if (i < xmin):
                xmin = i
            if (i > xmax):
                xmax = i
            if (j < ymin):
                ymin = j
            if (j > ymax):
                ymax = j
        for i in range(xmin,xmax):
            for j in range(ymin,ymax):
                if (i,j) not in self.map_matrix:
                    print(" ", end = "")
                elif self.map_matrix[(i,j)] == 0:
                    print("-", end = "")
                elif self.map_matrix[(i,j)] == 100:
                    print("#", end = "")
                elif self.map_matrix[(i,j)] == 200:
                    print("@", end = "")
                if len(self.mannequins) != 0:
                    if (i,j) in self.mannequins:
                        print("M", end = "")
                    
            print('') 
        t = self.listener.getLatestCommonTime("odom", "base_footprint")
        p1 = PoseStamped()
        p1.header.frame_id = "odom"
        p_in_base = self.listener.transformPose("base_footprint", p1)
        #print(p_in_base)

    def mannequins2grid(self,data):
        mannequins = list(data.data)
        m = 0
        for i in range(0,len(mannequins)-1,2):
            angle = mannequins[i]
            distance = mannequins[i+1]
            # Calculate the manhatten distance split into x and y
            x = distance * math.cos(angle) 
            y = distance * math.sin(angle) 
            #print(x,y)
            try:
                rmx = self.robot_x + x
                rmy = self.robot_y + y
                #print(rmx,rmy)
                print("Robot pos: {}, {}".format(self.robot_x,self.robot_y))
                print("Mannequin {}: {},{}".format(m,rmx,rmy))
                mx,my = self.width/2 - rmx, self.height/2 - rmy
                self.mannequins.append((mx,my))
                print("Mannequin in matrix {}: {},{}".format(m,mx,my))

                self.map_matrix[(mx,my)] = 'M'
                #print(self.map_matrix[(mx,my)])
            except:
                print("Mannequin out of range")
            # Calculate the distance in the map
            m+=1
            #print("Distance of mannequin {}: {},{}".format(m,x,y))
            print("------------------------------------------------------------------------------------")

    def getpos(self, data):
        self.pose = data.pose.pose.position
        self.orient = data.pose.pose.orientation
        self.robot_x = round(self.pose.x, 4)
        self.robot_y = round(self.pose.y, 4)

    def __init__(self):
        self.map_matrix = {}
        self.width = 0
        self.height = 0
        self.robot_x = 0
        self.robot_y = 0
        self.mannequins = []
        self.listener = tf.TransformListener()
        #print("Listening to map topic")
        ros.Subscriber("odom", Odometry, self.getpos)
        #ros.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.getpos)
        ros.Subscriber("map", OccupancyGrid, self.data2matrix)
        #time.sleep(2)
        ros.Subscriber("mannequins", numpy_msg(Floats), self.mannequins2grid)
        # vgtrans,rot = listener.lookupTransform('map', 'odom', ros.Time(0))
        #your_pose = PoseStamped()
        #print("Pose: {},{}".format(your_pose.pose.position.x,your_pose.pose.position.y))
        #listener.transformPose('odom', your_pose)
        

if __name__ == '__main__':  
    ros.init_node('occupancy_grid')
    #rate = ros.Rate(10) 
    occupancy_grid()
    ros.spin()