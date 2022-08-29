# TODO determine a direction for indi romi to follow. To accomplish this, we'll
# need to use a variant/combination of A*, BFS/DFS, random walk algorithm and 
# forward odometry kinematics

# Naive Maze Algorithm

"""
Path Planner Class
"""

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class PathPlanner():
    """
    Path Planning Functionality for Baxter/Sawyer

    We make this a class rather than a script because it bundles up 
    all the code relating to planning in a nice way thus, we can
    easily use the code in different places. This is a staple of
    good object-oriented programming

    Fields:
    _robot: moveit_commander.RobotCommander; for interfacing with the robot
    _scene: moveit_commander.PlanningSceneInterface; the planning scene stores a representation of the environment
    _group: moveit_commander.MoveGroupCommander; the move group is moveit's primary planning class
    _planning_scene_publisher: ros publisher; publishes to the planning scene


    """
    def __init__(self, romi_name):
        self.name = romi_name

        self.x = 0
        self.y = 0
        self.direction = ""
        self.avg_ranges = []

    def pose_callback(self, msg):
        pose = msg
        self.x = pose.position.x
        self.y = pose.position.y
    
    def laser_callback(self, msg):
        for range in msg.ranges:
            self.avg_ranges.append(range)

    def direction_to_pose(self):
        """
        Generates a direction given a pose with respect to a fixed frame subject to lidar range contraints
        """
        self.direction = "Forward"

