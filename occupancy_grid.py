import rospy as ros
import std_msgs.msg
from nav_msgs.msg import OccupancyGrid

class occupancy_grid:

    def data2matrix(data):
        print("Converting data to matrix")
        info = data.info
        robot_x = info.origin.position.x
        robot_y = info.origin.position.y
        d = data.data
        width = info.width
        height = info.height
        res = {} #[[0 for x in range(width)] for y in range(height)]
        for i in range(width):
            for j in range(height):
                if d[i*width + j] != -1:
                    res[(i,j)] = d[i*width + j]
        res[(width/2 - int(robot_x),height/2 - int(robot_y))]= 200
        xmin=width
        xmax=0
        ymin=height
        ymax=0
        print(robot_x, robot_y)
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
        ros.Subscriber("map", OccupancyGrid, occupancy_grid.data2matrix)
        ros.spin()

    
ros.init_node('occupancy_grid', anonymous = False)
#rate = ros.Rate(10)
occupancy_grid.listener()