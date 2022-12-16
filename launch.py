#Trying to launch all the launchers at once from the file 
#Works when running one (christmas_party.launch) but if SLAM is launched, it will cancel the previous
#I think this can be resolved with subprocesses 
#Before running, roscore must be running

import roslaunch
import rospkg
import rospy as ros
import sys
import time
import os

def launch():
    ros.init_node("launch")

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Obtaining the path of the launch files we want to initiate
    packager = rospkg.RosPack()
    
    try:
        gazebo_path = (
            f"{packager.get_path('christmas_party_simulation')}/launch/christmas_party.launch",
            "open_gazebo_gui:=true",
        )
    except:
        sys.exit("The specified environment couldn't be found")
    gazebo_launcher = roslaunch.parent.ROSLaunchParent(uuid, [gazebo_path])
    gazebo_launcher.start(auto_terminate=False)
    
    time.sleep(20)
    first_son = os.fork()
    second_son = os.fork()

    if first_son == 0:
        try:
    	    slam_path = (
		    f"{packager.get_path('turtlebot3_slam')}/launch/turtlebot3_slam.launch",
		    "open_rviz:=false",
			)
        except:
            sys.exit("The SLAM package couldn't be found")
            
        slam_launcher = roslaunch.parent.ROSLaunchParent(uuid, [slam_path])
        slam_launcher.start()
			
    if second_son == 0:
        try:
            navigation_path = f"{packager.get_path('turtlebot3_navigation')}/launch/turtlebot3_navigation.launch",
            "open_rviz:=false",
        except:
            sys.exit("The navigation package couldn't be found")
            
        navigation_launcher = roslaunch.parent.ROSLaunchParent(uuid, [navigation_path]) 
        navigation_launcher.start()
    
    #lam_launcher = roslaunch.child.ROSLaunchParent(uuid, [slam_path])
    #slam_launcher.start()

    while True:
        time.sleep(2)

    #navigation_launcher = roslaunch.parent.ROSLaunchParent(uuid, [navigation_path]) 
    #navigation_launcher.start()


if __name__ == "__main__":
    packager = rospkg.RosPack()
    #environments_path = f"{packager.get_path('turtlebot3_gazebo')}/launch"
    environments_path = f"{packager.get_path('christmas_party_simulation')}/launch"
    launch()
