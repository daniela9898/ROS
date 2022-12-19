import rospy as ros
import roslaunch
import time
import os
from subprocess import Popen, PIPE

def save_map():
    path = f"{os.path.expanduser('~')}/catkin_ws/src/christmas_party_simulation/maps/map"
    print(path)
    command = "rosrun map_server map_saver -f " + path
    print(command)
    p = Popen(command, shell=True, stdout=PIPE)



def remove_map():
    if os.path.exists(f"{os.path.expanduser('~')}/catkin_ws/src/christmas_party_simulation/maps/map.yaml"):
        os.remove(f"{os.path.expanduser('~')}/catkin_ws/src/christmas_party_simulation/maps/map.yaml")
    if os.path.exists(f"{os.path.expanduser('~')}/catkin_ws/src/christmas_party_simulation/maps/map.pgm"):
        os.remove(f"{os.path.expanduser('~')}/catkin_ws/src/christmas_party_simulation/maps/map.pgm")

def wait_for_map():
    while not os.path.exists(f"{os.path.expanduser('~')}/catkin_ws/src/christmas_party_simulation/maps/map.yaml") or not os.path.exists(f"{os.path.expanduser('~')}/catkin_ws/src/christmas_party_simulation/maps/map.pgm"):
        pass
    time.sleep(0.5)

def run_simulation():
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(['christmas_party_simulation', 'christmas_party.launch'])[0])]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    p = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    p.start()
    return p
    


def run_slam():
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(['turtlebot3_slam', 'turtlebot3_slam.launch'])[0])]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    p = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    p.start()
    return p

def run_navigation():
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(['turtlebot3_navigation', 'turtlebot3_navigation.launch'])[0], ['map_file:=$(env HOME)/catkin_ws/src/christmas_party_simulation/maps/slam.yaml'])]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    p = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    p.start()
    return p


def run_mannequin_detector():
    command = "python3 mannequin_detector_old.py"
    p = Popen(command, shell=True, stdout=PIPE)


def run_positions():
    command = "python3 positions.py"
    p = Popen(command, shell=True, stdout=PIPE)
    
def run_goals():
    command = "python3 mannequins_goal.py"
    p = Popen(command, shell=True, stdout=PIPE)
	
if __name__ == '__main__':
    ros.init_node("turtlebot3")

    remove_map()

    sim = run_simulation()
    time.sleep(10)
    slam = run_slam()

    time.sleep(5)

    map_save = save_map()

    wait_for_map()

    slam.shutdown()
    run_mannequin_detector()
    run_positions()


    nav = run_navigation()

    while True:
        pass
