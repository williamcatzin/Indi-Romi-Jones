#!/usr/bin/python3

import sys
import rospy
import numpy as np
import traceback
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose

from path_planner import PathPlanner

###############################SOCKET###################################
# Import socket module to read sensor data
# from romi robots via WIFI
import socket            
import struct

# Port on which you want to connect
_PORT = 8080
# IP Address
_IP_ADDRESS = '172.20.10.6'
# socket object
_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#########################################################################
    
def main(args):
    #connect to the server on local computer
    _SOCKET.connect((_IP_ADDRESS, _PORT))
    # Make sure that you've looked at and understand path_planner.py before starting
    rospy.init_node('path', anonymous=True)
    planner = PathPlanner(args[1])
    rospy.Subscriber('pose', Pose, planner.pose_callback, queue_size=10)
    rospy.Subscriber('filtered_scan', LaserScan, planner.laser_callback, queue_size=1)
    while not rospy.is_shutdown():
        if planner.direction == "Forward":
            direction = 'F'
            _SOCKET.send(direction.encode())
        elif planner.direction == "Right":
            direction = 'R'
            _SOCKET.send(direction.encode())
        elif planner.direction == "Stop":
            direction = 'S'
            _SOCKET.send(direction.encode())

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
