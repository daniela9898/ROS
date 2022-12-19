#! /usr/bin/env python

# This file takes the data from the lidar, calculates the distance 
# to an obstacle and if it detects an object nears it 
# publiches a message to the topic 'crash_detection'
# This topic is Boolean, where True means that an object is near 
# scan_node.py uses Lidar.py to calculate the distance to an obstacle

import rospy
import matplotlib.pyplot as plt
from datetime import datetime
from itertools import product
import std_msgs.msg
import math

import sys
sys.path.insert(0, '/home/maestro/catkin_ws/src/master_rad/scripts')

from Lidar import *

import std_msgs.msg
from nav_msgs.msg import OccupancyGrid


#******extract robot position ***********
class occupancy_grid:

    def data2matrix(data):
        print("Converting data to matrix")
        info = data.info
        self.robot_x = info.origin.position.x
        self.robot_y = info.origin.position.y
        d = data.data
        width = info.width
        height = info.height
        res = {} #[[0 for x in range(width)] for y in range(height)]
        for i in range(width):
            for j in range(height):
                if d[i*width + j] != -1:
                    res[(i,j)] = d[i*width + j]
        res[(width/2 - int(self.robot_x),height/2 - int(self.robot_y))]= 200
        xmin=width
        xmax=0
        ymin=height
        ymax=0
        print(self.robot_x, self.robot_y)
        for (i,j) in res.keys():
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
                if (i,j) not in res:
                    print(" ", end = "")
                elif res[(i,j)] == 0:
                    print("-", end = "")
                elif res[(i,j)] == 100:
                    print("#", end = "")
                elif res[(i,j)] == 200:
                    print("@", end = "")
            print('') 

    def listener():
        print("Listening to map topic")
        rospy.Subscriber("map", OccupancyGrid, occupancy_grid.data2matrix)
        rospy.spin()
    
    def get_position():
        return self.robot_x, self.robot_y
        
ros.init_node('occupancy_grid', anonymous = False)
#rate = ros.Rate(10)
occupancy_grid.listener()

# Create an instance of the occupancy_grid class
grid = occupancy_grid()

# Access the robot position using get_position method
robot_x,robot_y=grid.get_position()

#****** end of extraction robot position ***********


ANGLE_MAX = 360 - 1
ANGLE_MIN = 1 - 1
HORIZON_WIDTH = 75

MIN_TIME_BETWEEN_SCANS = 0
MAX_SIMULATION_TIME = float('inf')

# Create state space for Q table
def createStateSpace():
    x1 = set((0,1,2))
    x2 = set((0,1,2))
    x3 = set((0,1,2,3))
    x4 = set((0,1,2,3))
    state_space = set(product(x1,x2,x3,x4))
    return np.array(list(state_space))

if __name__ == '__main__':
    try:
        
        pub = rospy.Publisher('crash_detection', std_msgs.msg.Bool, queue_size=None)
        
        state_space = createStateSpace()

        rospy.init_node('scan_node', anonymous = False)
        rate = rospy.Rate(10)

        now = datetime.now()
        dt_string_start = now.strftime("%d/%m/%Y %H:%M:%S")
        print('SCAN NODE START ==> ', dt_string_start ,'\r\n')

        scan_time = 0
        count = 0

        t_0 = rospy.Time.now()
        t_start = rospy.Time.now()

        # init timer
        while not (t_start > t_0):
            t_start = rospy.Time.now()

        t = t_start

        # Init figure - real time
        plt.style.use('seaborn-ticks')
        fig = plt.figure(1)
        ax = fig.add_subplot(1,1,1)
        
        # main loop
        while not rospy.is_shutdown():
            
            msgScan = rospy.wait_for_message('/scan', LaserScan)

            scan_time = (rospy.Time.now() - t).to_sec()
            sim_time = (rospy.Time.now() - t_start).to_sec()
            count = count + 1

            if scan_time > MIN_TIME_BETWEEN_SCANS:
                print('\r\nScan cycle:', count , '\r\nScan time:', scan_time, 's')
                print('Simulation time:', sim_time, 's')
                t = rospy.Time.now()

                # distances in [m], angles in [degrees]
                ( lidar, angles ) = lidarScan(msgScan) #function to parse the LIDAR data and extract the distance measurements for each angle                
                ( state_ind, x1, x2 ,x3 ,x4 ) = scanDiscretization(state_space, lidar) #discretize the LIDAR data into a grid of cells, with each cell representing a specific state
                
                #********* Daniela's code********
                
                # Find the minimum distance measurement
                min_distance = min(lidar)

                # Find the index of the minimum distance measurement
                min_index = lidar.index(min_distance)

                # Get the angle of the nearest obstacle
                angle = angles[min_index]
                
                # Calculate the x coordinate of the obstacle
                x = min_distance * math.cos(angle)

                # Calculate the y coordinate of the obstacle
                y = min_distance * math.sin(angle)
                

                # Calculate the difference between the current position and the obstacle coordinates
                delta_x = x - robot_x
                delta_y = y - robot_y

                # Calculate the direction and distance that the robot needs to move
                direction = math.atan2(delta_y, delta_x)
                distance = math.sqrt(delta_x**2 + delta_y**2)
                
                #*********END of Daniela's code********
                
                #check if the LIDAR data indicates that a crash or an object nearby
                crash = checkCrash(lidar)
                object_nearby = checkObjectNearby(lidar)

                print('state index:', state_ind)
                print('x1 x2 x3 x4')
                print(x1, '', x2, '', x3, '', x4)
                if crash:
                    print('CRASH !')
                    #pub.publish(True)
                if object_nearby:
                    print('OBJECT NEARBY !')
                    pub.publish(True)
                else:
                    pub.publish(False)

                lidar_horizon = np.concatenate((lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
                #angles_horizon = np.concatenate((angles[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],angles[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
                angles_horizon = np.linspace(90+HORIZON_WIDTH, 90-HORIZON_WIDTH, 150)

                # horizon in x-y plane
                x_horizon = np.array([])
                y_horizon = np.array([])
                for i in range(len(lidar_horizon)):
                    x_horizon = np.append(x_horizon,lidar_horizon[i] * np.cos(radians(angles_horizon[i])))
                    y_horizon = np.append(y_horizon,lidar_horizon[i] * np.sin(radians(angles_horizon[i])))

                ax.clear()
                plt.xlabel('distance[m]')
                plt.ylabel('distance[m]')
                plt.xlim((-1.0,1.0))
                plt.ylim((-0.2,1.2))
                plt.title('Lidar horizon')
                plt.axis('equal')
                ax.plot(x_horizon, y_horizon, 'b.', markersize = 8, label = 'obstacles')
                ax.plot(0, 0, 'r*', markersize = 20, label = 'robot')
                plt.legend(loc = 'lower right', shadow = True)
                plt.draw()
                plt.pause(0.0001)

            if sim_time > MAX_SIMULATION_TIME:
                now = datetime.now()
                dt_string_stop = now.strftime("%d/%m/%Y %H:%M:%S")
                print('\r\nSCAN NODE START ==> ', dt_string_start ,'\r\n')
                print('SCAN NODE STOP ==> ', dt_string_stop ,'\r\n')
                rospy.signal_shutdown('End of simulation')

            rate.sleep()

    except rospy.ROSInterruptException:
        now = datetime.now()
        dt_string_stop = now.strftime("%d/%m/%Y %H:%M:%S")
        print('\r\nSCAN NODE START ==> ', dt_string_start ,'\r\n')
        print('SCAN NODE STOP ==> ', dt_string_stop ,'\r\n')
        rospy.signal_shutdown('End of simulation')

        pass

