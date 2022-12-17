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
import tf
import tf.transformations as tft
from scipy.signal import savgol_filter
from geometry_msgs.msg import PoseStamped


class plot_robot_position:

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
            self.orientation = pos.pose.orientation
            self.odom_pub.publish(pos)
        else:
            self.odom_pub.publish(data.pose)
        #print("/////////////")
        #print(self.pose)
        #print(self.orient)
        #print("/////////////")
        


        #publish pos, or data in pos == None




    def __beacon_ros_sub(self, data):
        self.new_mannequins = data.data

    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.new_mannequins = []
        self.visited_mannequins = []
        self.pos_mannequins = []
        self.a = 0
        self.transform_matrix = np.zeros((4, 4))

        self.listener = tf.TransformListener()
        self.odom_pub = ros.Publisher('odom_fixed', PoseStamped, queue_size=10)


        ros.Subscriber('/mannequins', numpy_msg(Floats), self.__beacon_ros_sub)
        ros.Subscriber('/odom', Odometry, self.robot_position)

        trans = []
        rot = []


        while False:
                if len(self.new_mannequins) > 0:
                    #get translation and rotation matrix
                    # try:
                    #     # tranformation
                    #     (trans, rot) = t.lookupTransform('/map', '/camera_link', ros.Time.now())
                    #
                    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    #     continue

                    # print(trans)
                    # print(rot)


                    plt.switch_backend("agg")
                    A = np.array(self.new_mannequins[::2])
                    R = np.array(self.new_mannequins[1:][::2])
                    print(A)
                    # compute relative position of the mannequin
                    x = R * np.cos(A)   #relative position
                    y = R * np.sin(A)   #relative position
                    z = 0 #relative position
                    fig = plt.figure(figsize=(4.10, 4.10), )
                    plt.ylim(-1.7, 1.7)
                    plt.xlim(-1.5, 1.5)

                    # convert relative position to absolute position
                    X = self.robot_x + np.cos(self.orient)*x - np.sin(self.orient)*y
                    Y = self.robot_y + np.sin(self.orient)*x + np.cos(self.orient)*y


                    plt.scatter(X , Y, color='blue')
                    plt.scatter(self.robot_x,self.robot_y, color='red')

                    # M = np.eye(4)
                    # M[:3, :3] = rot
                    # M[:3, 3] = trans
                    # transform x, y, z with trans and rot


                    # fig = plt.figure(figsize=(4.10, 4.10), )
                    # plt.ylim(-1.7, 1.7)
                    # plt.xlim(-1.5, 1.5)

                    # plt.scatter(x_abs , y_abs, color='blue')
                    # plt.scatter(self.robot_x,self.robot_y, color='red')

                
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
    plot_robot_position()
    ros.spin()

    
