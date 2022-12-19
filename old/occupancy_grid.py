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
import numpy as np
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseResult

class occupancy_grid:

    def data2matrix(self, data):
        print("Converting data to matrix")
        info = data.info
        d = data.data

        self.width = info.width
        self.height = info.height

        self.initial_pose = (int(self.width/2 - info.origin.position.x),int(self.width/2 - info.origin.position.y) ,0)

        for i in range(self.height):
            for j in range(self.width):
                if d[i*self.width + j] != -1:
                    self.map_matrix[(j, i)] = d[i*self.width + j]

        self.map_matrix[(self.initial_pose[0] + self.position[0], self.initial_pose[1] + self.position[1])]= 200
        for m in self.mannequins:
            self.map_matrix[self.initial_pose[0] + m[0], self.initial_pose[1] +  m[1]] = 300
        self.mannequins= []

        xmin=min(self.map_matrix.keys(), key= lambda x: x[0])[0]
        xmax=max(self.map_matrix.keys(), key= lambda x: x[0])[0]
        ymin=min(self.map_matrix.keys(), key= lambda x: x[1])[1]
        ymax=max(self.map_matrix.keys(), key= lambda x: x[1])[1]

        for i in range(xmax,xmin, -1):
            for j in range(ymax,ymin, -1):
                if (i, j) == (int(self.width/2), int(self.width/2)):
                    print("X", end = "")
                elif (i,j) not in self.map_matrix:
                    print(" ", end = "")
                elif self.map_matrix[(i,j)] == 0:
                    print("-", end = "")
                elif self.map_matrix[(i,j)] == 100:
                    print("#", end = "")
                elif self.map_matrix[(i,j)] == 200:
                    print("@", end = "")
                elif self.map_matrix[(i,j)] == 300:
                    print("M", end = "")
                elif self.map_matrix[(i,j)] == 300:
                    print("X", end = "")
                if len(self.mannequins) != 0:
                    if (i,j) in self.mannequins:
                        print("M", end = "")
                    
            print('') 

    def mannequins2grid(self,data):
        mannequins = list(data.data)
        print(mannequins)
        m = 0
        for i in range(0,len(mannequins)-1,2):
            angle = mannequins[i]
            distance = mannequins[i+1]
            # Calculate the manhatten distance split into x and y
            x = distance * np.cos(angle + self.orientation) 
            y = distance * np.sin(angle +self.orientation) 
            #print(x,y)
            try:
                rmx = self.position[0] + int(x/0.05)
                rmy = self.position[1] + int(y/0.05)
                print("Rob {}: {},{}".format(m,self.position[0],self.position[1]))
                print("Mannequin {}: {},{}".format(m,int(x/0.05),int(y/0.05)))
                print("Mannequin {}: {},{}".format(m,rmx,rmy))
                self.mannequins.append((rmx, rmy))
                #print("Mannequin in matrix {}: {},{}".format(m,mx,my))

                #self.map_matrix[(mx,my)] = 'M'
                #print(self.map_matrix[(mx,my)])
            except:
                print("Mannequin out of range")
            # Calculate the distance in the map
            m+=1
            #print("Distance of mannequin {}: {},{}".format(m,x,y))
            print("------------------------------------------------------------------------------------")

    def getpos(self, data):
        self.pose = data.pose.pose.position #x,y,z
        self.orient = data.pose.pose.orientation # x,y,z
        pos = self.getPoseStamped([self.pose.x, self.pose.y, self.pose.z, self.orient.x, self.orient.y, self.orient.z])
        self.position = (int(pos.pose.position.x/0.05), int(pos.pose.position.y/0.05))
        #print(self.position)
        self.orientation = np.arctan2(self.orient.z, self.orient.w)*2
        #print("---")

    def getPoseStamped(self, c):
        
        assert(len(c) == 6)
        
        p = PoseStamped()
        
        p.header.frame_id = "odom"
        p.header.stamp = ros.Time.now() - ros.Duration(0.5)
        
        p.pose.position.x = c[0]
        p.pose.position.y = c[1]
        p.pose.position.z = c[2]
        
        quat = tf.transformations.quaternion_from_euler(c[3], c[4], c[5])
        
        p.pose.orientation.x = quat[0]
        p.pose.orientation.y = quat[1]
        p.pose.orientation.z = quat[2]
        p.pose.orientation.w = quat[3]
        
        try:
            self.listener.waitForTransform(p.header.frame_id, "map", p.header.stamp, ros.Duration(2))
            p = self.listener.transformPose("map", p)
            
        except:
            ros.logerr("TF error!")
            return None
        
        return p

    def __init__(self):
        self.map_matrix = {}
        self.width = 0
        self.height = 0
        self.robot_x = 0
        self.robot_y = 0
        self.position = (0,0)
        self.mannequins = []
        self.listener = tf.TransformListener()
        #print("Listening to map topic")
        ros.Subscriber("odom", Odometry, self.getpos)
        ros.Subscriber("map", OccupancyGrid, self.data2matrix)
        ros.Subscriber("mannequins", numpy_msg(Floats), self.mannequins2grid)
        #print("Pose: {},{}".format(your_pose.pose.position.x,your_pose.pose.position.y))
        #listener.transformPose('odom', your_pose)
        

if __name__ == '__main__':  
    ros.init_node('occupancy_grid')
    #rate = ros.Rate(10) 
    occupancy_grid()
    ros.spin()
