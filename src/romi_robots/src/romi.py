#!/usr/bin/env python3
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
from numpy.lib.function_base import angle
import rospy
import sys
from indi_romi import Romi
from romi_state import State

###############################SOCKET###################################
# Import socket module to read sensor data
# from romi robots via WIFI
import socket            
import struct

# Port on which you want to connect
_PORT = 8080
# IP Address
_IP_ADDRESS = '172.20.10.6' # William's IP
#_IP_ADDRESS = '192.168.43.78'
# socket object
_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#########################################################################

  

def main(args):
    # get romi name
    romi_name = args[1]
    #initialize ROS node 
    rospy.init_node(romi_name, anonymous=True)
    # instantiate romi object
    romi = Romi(romi_name)
    #connect to the server on local computer
    _SOCKET.connect((_IP_ADDRESS, _PORT))
   
   
    ranges = []
    avg_ranges = []
    angles = []
    count = 0
    left_tick_data = 0
    right_tick_data = 0
    # Start State Machine
    while not rospy.is_shutdown():
        # Receive Sensor Data from Romi
        if romi.state == State.Initialize:
            # Initial state of romi
            input("Press <Enter> to begin: ")
            msg = 'K'
            _SOCKET.sendall(msg.encode('utf-8'))
            romi.state = State.Receive
        elif romi.state == State.Receive:
            data0 = struct.unpack('f', _SOCKET.recv(4))[0]
            # Publish and broadcast when data0 is less than 0
            # print("Angle: \n", data2)
            # print("Range: \n", data1)
            #print(data0)
            if data0 < -1.0:
                data3 = struct.unpack('i', _SOCKET.recv(4))[0]
                data4 = struct.unpack('i', _SOCKET.recv(4))[0]
                data3 = struct.unpack('i', _SOCKET.recv(4))[0]
                data4 = struct.unpack('i', _SOCKET.recv(4))[0]
                #romi.state = State.Send
            elif data0 > 0:
                data1 = struct.unpack('f', _SOCKET.recv(4))[0] 
                data2 = struct.unpack('f', _SOCKET.recv(4))[0]
                data3 = struct.unpack('i', _SOCKET.recv(4))[0]
                data4 = struct.unpack('i', _SOCKET.recv(4))[0]
                print("RECEIVING...")
                ranges.append(data1)
                angles.append(data2)
                left_tick_data = data3
                right_tick_data = data4
                #print("Left Tick", left_tick_data)
                romi.state = State.Receive
            else:
                # publish laser and wheel encoder data
                data1 = struct.unpack('f', _SOCKET.recv(4))[0] 
                data2 = struct.unpack('f', _SOCKET.recv(4))[0]
                data3 = struct.unpack('f', _SOCKET.recv(4))[0]
                data4 = struct.unpack('f', _SOCKET.recv(4))[0]
                print("PUBLISHING...")
                # print("Avg Range1: \n", data1)
                # print("Avg Range2: \n", data2)
                # print("Avg Range3: \n", data3)
                # print("Avg Range4: \n", data4)
                romi.publish_sensor_data(ranges, left_tick_data, right_tick_data)
                romi.state = State.Receive
                ranges = []
        elif romi.state == State.Send:
            print("SENDING DIRECTION...")
            data0 = struct.unpack('f', _SOCKET.recv(4))[0]
            data1 = struct.unpack('f', _SOCKET.recv(4))[0] 
            data2 = struct.unpack('f', _SOCKET.recv(4))[0]
            data3 = struct.unpack('f', _SOCKET.recv(4))[0]
            data4 = struct.unpack('f', _SOCKET.recv(4))[0]

            print(data0, data1, data2, data3, data4)
            avg_ranges.extend([data1, data2, data3, data4])
            romi.path_finder(avg_ranges)
            direction = romi.direction
            print("Sending Direction: ", direction)
            _SOCKET.sendall(direction.encode('utf-8'))
            avg_ranges = []
            romi.state = State.Receive
        elif romi.state == State.Idle:
            continue




if __name__ == '__main__':
    try:
        # sys.argv contains romi info
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass


       
 
            
 

 
