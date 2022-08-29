################################################################################
#
# Node to wrap the romi class.
#
################################################################################

from re import X
import rospy
from rospy.rostime import Duration
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from romi_state import State
from std_msgs.msg import String

# YDLIDAR SPECIFICATIONS
ANGLE_MIN = -3.141593
ANGLE_MAX = 3.141593
ANGLE_INCREMENT = 0.012849
TIME_INCREMENT = 0.0
SCAN_TIME = 0.122000
RANGE_MIN = 0.120000
RANGE_MAX = 10.000000
INTENSITIES = []

# ROMI ROBOT PHYSICAL SPECIFICATIONS
WHEEL_TRACK = 0.149
WHEEL_RADIUS = 0.036
# 72 * pi/1000*(1/0.0006108)
TICKS_PER_ROTATION = 370

DIRECTIONS = ['F', 'R', 'L', 'B']

class Romi():
    def __init__(self, name):
        # romi's name
        self.name = name
        # romi's position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # romi's velcity
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        # romi's old wheel encoder data
        self.old_left_ticks = -1
        self.old_right_ticks = -1
        # romi's old time
        self.old_time = rospy.Time.now()
        # broadcasting and publishing objects
        self.laser_pub = rospy.Publisher("scan", LaserScan, queue_size=1)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        ##########################PATH PLANNING##################################
        #self.laser_sub = rospy.Subscriber("filtered_scan", LaserScan, self.path_finder, queue_size=1)
        # Direction of romi
        self.direction = ''
        # Initial state 
        self.state = State.Initialize


    def publish_sensor_data(self, ranges, left_tick_data, right_tick_data):
        current_time = rospy.Time.now()

        ############################Publish laser range data###############################
        scan = LaserScan()
        scan.header.stamp = current_time
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = ANGLE_MIN
        scan.angle_max = ANGLE_MAX
        scan.angle_increment = ANGLE_INCREMENT
        scan.time_increment = TIME_INCREMENT
        scan.scan_time = SCAN_TIME
        scan.range_min = RANGE_MIN
        scan.range_max = RANGE_MAX
        scan.ranges = []
        for r in ranges:
            scan.ranges.append(r)
        scan.intensities = [0] * len(ranges)
        self.laser_pub.publish(scan)


        ########################################ODOMETRY#####################################
        if self.old_left_ticks < 0:
            self.old_left_ticks = left_tick_data
        if self.old_right_ticks < 0:
            self.old_right_ticks = right_tick_data

        delta_left = self.get_forward_tick_delta(left_tick_data, self.old_left_ticks)
        # if the wheels ever turn in reverse, this code will not behave correctly
        # insead, the robot will appear to drive forward at high speed.

        delta_right = self.get_forward_tick_delta(right_tick_data, self.old_right_ticks)
        print("new, old (", right_tick_data, self.old_right_ticks, ")")
        print("Left Encoder: \n", delta_left)
        print("Right Encoder: \n", delta_right)
        # distance left wheel has traveled 
        left_wheel_distance_travled = (2 * np.pi * WHEEL_RADIUS * delta_left) / TICKS_PER_ROTATION #dl
        # distance right wheel has traveled
        right_wheel_distance_traveled = (2 * np.pi * WHEEL_RADIUS * delta_right) / TICKS_PER_ROTATION #dr
        # approx distance the center of the robot has traveled
        center_distance_traveled = (left_wheel_distance_travled + right_wheel_distance_traveled) / 2 # dc
        elapsed_time = (current_time - self.old_time).to_sec() #dt
        change_in_angle = (right_wheel_distance_traveled - left_wheel_distance_travled) / WHEEL_TRACK #dth

        # if the distance traveled is the same
        # then romi has moved forward
        if abs(left_wheel_distance_travled - right_wheel_distance_traveled) <= 0.5:
            dx = right_wheel_distance_traveled * np.cos(self.theta)
            dy = right_wheel_distance_traveled * np.sin(self.theta)
        else:
            radius = center_distance_traveled / change_in_angle
            # ICC: Instantaneous Center of Curvature
            iccX = self.x - radius * np.sin(self.theta)
            iccY = self.y + radius * np.cos(self.theta)
            dx = np.cos(change_in_angle) * (self.x - iccX) - np.sin(change_in_angle) * (self.y - iccY) + iccX - self.x
            dy = np.sin(change_in_angle) * (self.x - iccX) - np.cos(change_in_angle) * (self.y - iccY) + iccY - self.y

        self.x += dx
        self.y += dy
        self.theta = (self.theta + change_in_angle) % (2 * np.pi)

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        self.odom_broadcaster.sendTransform((self.x, self.y, 0.), odom_quat, current_time, "base_link", "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        if elapsed_time > 0.0:
            self.vx = dx/elapsed_time
            self.vy = dy/elapsed_time
            self.omega = change_in_angle/elapsed_time
        
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.omega))
        self.odom_pub.publish(odom)

        # Record current left and right ticks
        # and current time for next iteration
        self.old_left_ticks = left_tick_data
        self.old_right_ticks = right_tick_data
        self.old_time = current_time
            
    def get_forward_tick_delta(self, new_tick, old_tick):
        if new_tick < old_tick:
            return (1 << 17) - old_tick + new_tick 
        else:
            return new_tick - old_tick
    
    def path_finder(self, ranges):
        max_range = max(ranges[1], ranges[2])
        max_index = ranges.index(max_range)
        #print("MAX RANGE: ", DIRECTIONS[max_index])

        self.direction = DIRECTIONS[max_index]
            




