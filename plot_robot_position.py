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

class plot_robot_position:
    def plot(self):
            while True:
                if len(self.new_mannequins) > 0:
                    A = np.array(self.new_mannequins[::2])
                    R = np.array(self.new_mannequins[1:][::2])

                    # Consider also the robot orientation
                    A = A + np.arctan2(self.orient.z, self.orient.w) * 2
                    X = R * np.cos(A) + self.robot_x
                    Y = R * np.sin(A) + self.robot_y            

                    # Add the new mannequins to the list of visited mannequins
                    for i in range(len(X)):
                        self.visited_mannequins.append([X[i], Y[i]])
                        
                    fig = plt.figure(figsize=(4.10, 4.10), )
                    plt.ylim(-1.7, 1.7)
                    plt.xlim(-1.5, 1.5)
                    plt.scatter(X , Y, color='blue')
                    plt.scatter(self.robot_x,self.robot_y, color='red')
                    plt.scatter(np.array(self.visited_mannequins)[:,0], np.array(self.visited_mannequins)[:,1], color='green')
                
                    fig.canvas.draw()
                    #img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
                    img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
                    img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

                    cv2.imshow("Mannequins", img)
                    cv2.waitKey(100)

                    plt.close(fig)
                    
                else:
                    time.sleep(0.1)
    
    def robot_position(self, data):
        self.pose = data.pose.pose.position
        self.orient = data.pose.pose.orientation
        self.robot_x = round(self.pose.x, 4)
        self.robot_y = round(self.pose.y, 4)

    def __beacon_ros_sub(self, data):
        self.new_mannequins = data.data

    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.new_mannequins = []
        self.visited_mannequins = []
        ros.Subscriber('/mannequins', numpy_msg(Floats), self.__beacon_ros_sub)
        ros.Subscriber('/odom', Odometry, self.robot_position)
        while True:
                if len(self.new_mannequins) > 0:
                    A = np.array(self.new_mannequins[::2])
                    R = np.array(self.new_mannequins[1:][::2])

                    # Consider also the robot orientation
                    A = A + np.arctan2(self.orient.z, self.orient.w) * 2
                    X = R * np.cos(A) + self.robot_x
                    Y = R * np.sin(A) + self.robot_y            

                    # Add the new mannequins to the list of visited mannequins
                    for i in range(len(X)):
                        self.visited_mannequins.append([X[i], Y[i]])
                        
                    fig = plt.figure(figsize=(4.10, 4.10), )
                    plt.ylim(-1.7, 1.7)
                    plt.xlim(-1.5, 1.5)
                    plt.scatter(X , Y, color='blue')
                    plt.scatter(self.robot_x,self.robot_y, color='red')
                    plt.scatter(np.array(self.visited_mannequins)[:,0], np.array(self.visited_mannequins)[:,1], color='green')
                
                    fig.canvas.draw()
                    #img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
                    img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
                    img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

                    cv2.imshow("Mannequins", img)
                    cv2.waitKey(100)

                    plt.close(fig)
                    
                else:
                    time.sleep(0.1)

if __name__ == '__main__':
    ros.init_node('plot_robot_position')
    plot_robot_position()
    ros.spin()

    