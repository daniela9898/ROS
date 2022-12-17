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


class plot_robot_position:

    def robot_position(self, data):


        self.pose = data.pose.pose.position
        quaternion = data.pose.pose.orientation
        self.orient = tft.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]

        self.robot_x = round(self.pose.x, 4)
        self.robot_y = round(self.pose.y, 4)

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

        ros.Subscriber('/mannequins', numpy_msg(Floats), self.__beacon_ros_sub)
        ros.Subscriber('/odom', Odometry, self.robot_position)

        t = tf.TransformListener()
        trans = []
        rot = []


        while True:
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

if __name__ == '__main__':
    ros.init_node('plot_robot_position')
    plot_robot_position()
    ros.spin()

    