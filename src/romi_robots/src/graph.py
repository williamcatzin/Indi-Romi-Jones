from numpy.testing._private.utils import rand
import rospy
import tf2_ros
import tf

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import ColorRGBA

import numpy as np

class Graph():

    def __init__(self):
        pass