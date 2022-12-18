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
from geometry_msgs.msg import PoseStamped
import tf

class plot_robot_position:
    
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
    
    def robot_position(self, data):
        self.pose = data.pose.pose.position #x,y,z
        self.orient = data.pose.pose.orientation # x,y,z
        pos = self.getPoseStamped([self.pose.x, self.pose.y, self.pose.z, self.orient.x, self.orient.y, self.orient.z])
        #print(pos)
        self.robot_x = pos.pose.position.x
        self.robot_y = pos.pose.position.y
        self.orient = pos.pose.orientation

    def __beacon_ros_sub(self, data):
        self.new_mannequins = data.data

    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.new_mannequins = []
        self.noise_plus_mannequins = []
        self.visited_mannequins = []
        self.listener = tf.TransformListener()
        time.sleep(2)
        ros.Subscriber('/mannequins', numpy_msg(Floats), self.__beacon_ros_sub)
        ros.Subscriber('/odom', Odometry, self.robot_position)
        while True:
            if len(self.new_mannequins) > 0:
                plt.switch_backend('agg')
                print(self.new_mannequins)
                
                A = np.array(self.new_mannequins[::2])
                R = np.array(self.new_mannequins[1:][::2])
                print("Angles:", A)
                print("Distances:", R)
                # Consider also the robot orientation
                A = A + np.arctan2(self.orient.z, self.orient.w) * 2
                X = R * np.cos(A) + self.robot_x
                Y = R * np.sin(A) + self.robot_y            

                # Add the new mannequins to the list of visited mannequins
                for i in range(len(X)):
                    self.noise_plus_mannequins.append([X[i], Y[i]])
                
                # Save a coordinate of noise_plus_mannequins in visited_mannequins only if the measurement apears more than 3 times in noise_plus_mannequins
                for i in range(len(self.noise_plus_mannequins)):
                    if self.noise_plus_mannequins.count(self.noise_plus_mannequins[i]) >= 2:
                        self.visited_mannequins.append(self.noise_plus_mannequins[i])
                
                #plotting the robot position and the mannequins position
                fig = plt.figure(figsize=(4.10, 4.10), )
                plt.ylim(-1.7, 1.7)
                plt.xlim(-1.5, 1.5)
                plt.scatter(X , Y, color='blue')
                plt.scatter(self.robot_x,self.robot_y, color='red')
                if self.visited_mannequins != []:
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

    