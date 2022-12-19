#Script that plots the position of the robot and the mannequins in the environment
#Lots of noise in the plot, but it works
#It also gets the mannequins positon in the environment
#We need to publish this positions as goals

import matplotlib.pyplot as plt
import time
import rospy as ros 
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
import cv2
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import tf
import tf.transformations as tft
from scipy.signal import savgol_filter
from geometry_msgs.msg import PoseStamped


class Position:

    def robot_position(self, data):
        self.pose = data.pose.pose.position #x,y,z
        self.orient = data.pose.pose.orientation
        o = tf.transformations.euler_from_quaternion((self.orient.x, self.orient.y, self.orient.z, self.orient.w))
        #print("/////////////")
        #print(self.pose)
        #print(self.orient)
        pos = self.getPoseStamped([self.pose.x, self.pose.y, self.pose.z, o[0], o[1], o[2]])
        if pos is not None:
            self.pose = pos.pose.position
            #self.orient = pos.pose.orientation
            self.odom_pub.publish(pos)
        else:
			pose = PoseStamped()
			pose.header = data.header
			pose.pose = self.pose.pose
            self.odom_pub.publish(pose)
        #print("/////////////")
        #print(self.pose)
        #print(self.orient)
        #print("/////////////")


    def __beacon_ros_sub(self, data):
        if len(data.data) > 0:
            ori = np.arctan2(self.orient.z, self.orient.w)*2
            A = np.array(data.data[::2])
            R = np.array(data.data[1:][::2])
            # compute relative position of the mannequin
            x = R * np.cos(A)   #relative position
            y = R * np.sin(A)   #relative position

            # convert relative position to absolute position
            X = self.pose.x + np.cos(ori)*x - np.sin(ori)*y
            Y = self.pose.y + np.sin(ori)*x + np.cos(ori)*y

            Xmap = X
            Ymap = Y

            for i in range(len(Xmap)):
                self.updateMannequinMap(Xmap[i], Ymap[i])

            mann_list = Float32MultiArray()
            key_list = list(self.mannequins.keys())
            key_list.sort(key=lambda x: abs(x[0]-self.pose.x) + abs(x[1]-self.pose.y))
            mann_list.data = [item for t in key_list for item in t]
            print(self.mannequins.keys())
            #print(key_list)
            #print(mann_list)
            self.mannequins_pub.publish(mann_list)		


    def __init__(self):
        self.mannequins = {}
        self.transform_matrix = np.zeros((4, 4))

        self.listener = tf.TransformListener()
        self.odom_pub = ros.Publisher('odom_fixed', PoseStamped, queue_size=10)
        self.mannequins_pub = ros.Publisher('mannequins_position', Float32MultiArray, queue_size=10)


        ros.Subscriber('/mannequins', numpy_msg(Floats), self.__beacon_ros_sub)
        ros.Subscriber('/odom', Odometry, self.robot_position)

    def updateMannequinMap(self, Xmap, Ymap):
        self.mannequins[(Xmap, Ymap)] = 1.0

        for k in self.mannequins.keys():
            if k != (Xmap, Ymap):   #TODO: if distance  > X dont substract
                self.panalizeCell(k[0], k[1], 0.1/(abs(k[0]-Xmap) + abs(k[1] - Ymap) ))
        delkeys = []
        for k in self.mannequins.keys():
            if self.mannequins[k] <= 0.5:
                delkeys.append(k)
        for k in delkeys:
            del self.mannequins[k]

    def panalizeCell(self, X, Y, pen):
        if (X, Y) in self.mannequins.keys():
            self.mannequins[(X, Y)] -= pen 

       


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

if __name__ == '__main__':
    ros.init_node('plot_robot_position')
    Position()
    ros.spin()

    
