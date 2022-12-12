import rospy as ros
import std_msgs.msg
from nav_msgs.msg import OccupancyGrid
from rospy.numpy_msg import numpy_msg

class occupancy_grid:

    def data2matrix(data):
        print("Converting data to matrix")
        info = data.info
        robot_x = info.origin.position.x
        robot_y = info.origin.position.y
        d = data.data
        width = info.width
        height = info.height
        for i in range(width):
            for j in range(height):
                if d[i*width + j] != -1:
                    map_matrix[(i,j)] = d[i*width + j]
        map_matrix[(width/2 - int(robot_x),height/2 - int(robot_y))]= 200
        xmin=width
        xmax=0
        ymin=height
        ymax=0
        print(robot_x, robot_y)
        for (i,j) in map_matrix.keys():
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
                if (i,j) not in map_matrix:
                    print(" ", end = "")
                elif map_matrix[(i,j)] == 0:
                    print("-", end = "")
                elif map_matrix[(i,j)] == 100:
                    print("#", end = "")
                elif map_matrix[(i,j)] == 200:
                    print("@", end = "")
            print('') 

    def mannequins2grid(data):
        for i in range(len(data)):
            angle1 = data[i]
            angle2 = data[i+1] 

    def listener():
        print("Listening to map topic")
        ros.Subscriber("map", OccupancyGrid, occupancy_grid.data2matrix)
        ros.Subscriber("mannequin", numpy_msg, occupancy_grid.mannequins2grid)
        ros.spin()

    
ros.init_node('occupancy_grid', anonymous = False)
#rate = ros.Rate(10) 
map_matrix = {}
occupancy_grid.listener()