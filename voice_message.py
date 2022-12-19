#!/usr/bin/env python

from rospy import Subscriber, Publisher
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

from typing import Union
import numpy as np
import rospy as ros
from rospy import Subscriber
from rospy_tutorials.msg import Floats
from std_msgs.msg import String
import matplotlib.pyplot as plt
import pygame

class produceVoiceMessage:
    QUEUE_SIZE = 2 #constant that is used to specify the size of the message queue for the ROS subscriber.
    
    def __init__(self, name: str = "voice_message"):  
        self.node_name: str = name
        # Declare additional ROS listener
        self.beacon_sub_name: str = "qr_codes"   #name of the ros topic
        self.beacon_sub: Union[None, Subscriber]   #Subscriber object to receive messages
        self.beacons: Union[None, np.ndarray] = None  #store the received qr_codes information
        
    def start_ros(self) -> None:
        ros.init_node(self.node_name, log_level=ros.INFO) #initialization of the node
        self.beacon_sub = ros.Subscriber(self.beacon_sub_name, String, callback=self.__beacon_ros_sub, queue_size=self.QUEUE_SIZE)    
                         
    def __beacon_ros_sub(self, msg):
        self.beacons = msg.data

    def run(self):
        if self.beacons is not None:
            if "FRONT" in self.beacons:
                soundhandle = SoundClient(blocking=True)
                soundhandle.say('This is your drink, Merry Christmas!!!')
                ros.sleep(1)
             else:
                ros.logwarn("No data received on /qr_codes topic yet")

def main():
    l = produceVoiceMessage()
    l.start_ros()
    while True:
    	l.run()


if __name__ == '__main__':
    main()
