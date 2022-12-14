import math
import time
import rospy as ros
import std_msgs.msg
from nav_msgs.msg import OccupancyGrid
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from typing import Union
from rospy import Subscriber
import tf
from geometry_msgs.msg import PoseStamped

class occupancy_grid:

    def data2matrix(self, data):
        print("Converting data to matrix")
        info = data.info
        self.robot_x = info.origin.position.x
        self.robot_y = info.origin.position.y
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

    def mannequins2grid(self,data):
        mannequins = list(data.data)
        m = 0
        for i in range(0,len(mannequins)-1,2):
            angle = mannequins[i]
            distance = mannequins[i+1]
            # Calculate the manhatten distance split into x and y
            x = int(distance * math.cos(angle))
            y = int(distance * math.sin(angle))
            try:
                rmx = int(self.robot_x + x)
                rmy = int(self.robot_y + y)
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


    def __init__(self):
        self.map_matrix = {}
        self.width = 0
        self.height = 0
        self.robot_x = 0
        self.robot_y = 0
        self.mannequins = []
        print("Listening to map topic")
        ros.Subscriber("map", OccupancyGrid, self.data2matrix)
        #time.sleep(2)
        ros.Subscriber("mannequins", numpy_msg(Floats), self.mannequins2grid)
        
        

if __name__ == '__main__':  
    ros.init_node('occupancy_grid')
    #rate = ros.Rate(10) 
    occupancy_grid()
    ros.spin()