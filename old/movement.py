import rospy as ros
import datetime
import std_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# This file depends on scan_node.py, which publishes a message to the topic 'crash_detection'
# This topic is Boolean, where True means that an object is near
# if an object is near, the robot stops
# For know it only makes the robot move forward   

class movement:
    def stop(data):
        #print(std_msgs.msg._Bool.Bool(True))
        if data == std_msgs.msg._Bool.Bool(True):
            print("Stopping")
            velocity_publisher = ros.Publisher('/cmd_vel', Twist, queue_size=10)
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            velocity_publisher.publish(msg)

    def listener():
        print("Listening to crash_detection topic")
        ros.Subscriber("crash_detection", std_msgs.msg.Bool, movement.stop)
        ros.spin()

    def move():
        velocity_publisher = ros.Publisher('/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
  
        vel_msg.linear.x = 0.1
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        
        

        now = ros.Time.now()
  
        while ros.Time.now() < now + ros.Duration.from_sec(1):
            velocity_publisher.publish(vel_msg)
            print("Moving forward", 0.1, "m/s")


ros.init_node('movement', anonymous = False)
#rate = ros.Rate(10)
movement.move()
movement.listener()

