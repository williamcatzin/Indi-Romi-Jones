################################################################################
#
# OccupancyGrid2d class listens for LaserScans and builds an occupancy grid.
#
################################################################################

from numpy.testing._private.utils import rand
import rospy
import tf2_ros
import tf

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import ColorRGBA

import numpy as np

class OccupancyGrid2d(object):
    def __init__(self):
        self._intialized = False

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/grid_map_2d"

        # Load parameters.
        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Set up the map.
        self._map = np.zeros((self._x_num, self._y_num))

        # m = Marker()
        # m.header.stamp = rospy.Time.now()
        # m.header.frame_id = self._fixed_frame
        # m.ns = "map"
        # m.id = 0
        # m.type = Marker.CUBE_LIST
        # m.action = Marker.ADD
        # m.scale.x = self._x_res
        # m.scale.y = self._y_res
        # m.scale.z = 0.01
        # # Initial Pose
        # point = Point()
        # point.x = 0.0
        # point.y = 0.0
        # point.z = 0.0

        # quat = Quaternion()
        # quat.x = 0
        # quat.y = 0
        # quat.z = 0
        # quat.w = 0

        # pose = Pose()
        # pose.position = point
        # pose.orientation = quat
        # m.pose = pose
        # #m.lifetime = 0
        # m.frame_locked = True

        # for ii in range(self._x_num):
        #     for jj in range(self._y_num):
        #         p = Point(0.0, 0.0, 0.0)
        #         (p.x, p.y) = self.VoxelCenter(ii, jj)

        #         m.points.append(p)
        #         m.colors.append(self.Colormap(ii, jj))

        # self._vis_pub.publish(m)

        self._initialized = True
        return True

    def LoadParameters(self):
        # Random downsampling fraction, i.e. only keep this fraction of rays.
        if not rospy.has_param("~random_downsample"):
            return False
        self._random_downsample = rospy.get_param("~random_downsample")

        # Dimensions and bounds.
        # TODO! You'll need to set values for class variables called:
        # -- self._x_num
        # -- self._x_min
        # -- self._x_max
        # -- self._x_res # The resolution in x. Note: This isn't a ROS parameter. What will you do instead?
        # -- self._y_num
        # -- self._y_min
        # -- self._y_max
        # -- self._y_res # The resolution in y. Note: This isn't a R_sensor_topic
        # load x_num
        if not rospy.has_param("~x/num"):
            return False
        self._x_num = rospy.get_param("~x/num")

        # load x_min
        if not rospy.has_param("~x/min"):
            return False
        self._x_min = rospy.get_param("~x/min")
        
        # load x_max
        if not rospy.has_param("~x/max"):
            return False
        self._x_max = rospy.get_param("~x/max")

        # load y_num
        if not rospy.has_param("~y/num"):
            return False
        self._y_num = rospy.get_param("~y/num")

        # load y_min
        if not rospy.has_param("~y/min"):
            return False
        self._y_min = rospy.get_param("~y/min")

        # load y_max
        if not rospy.has_param("~y/max"):
            return False
        self._y_max = rospy.get_param("~y/max")

        # load x_res
        self._x_res = (self._x_max - self._x_min) / self._x_num

        # load y_res
        self._y_res = (self._y_max - self._y_min) / self._y_num

        # Update parameters.
        if not rospy.has_param("~update/occupied"):
            return False
        self._occupied_update = self.ProbabilityToLogOdds(
            rospy.get_param("~update/occupied"))

        if not rospy.has_param("~update/occupied_threshold"):
            return False
        self._occupied_threshold = self.ProbabilityToLogOdds(
            rospy.get_param("~update/occupied_threshold"))

        if not rospy.has_param("~update/free"):
            return False
        self._free_update = self.ProbabilityToLogOdds(
            rospy.get_param("~update/free"))

        if not rospy.has_param("~update/free_threshold"):
            return False
        self._free_threshold = self.ProbabilityToLogOdds(
            rospy.get_param("~update/free_threshold"))

        # Topics.
        # TODO! You'll need to set values for class variables called:

        if not rospy.has_param("~topics/sensor"):
            return False
        self._sensor_topic = rospy.get_param("~topics/sensor")

        
        if not rospy.has_param("~topics/vis"):
            return False
        self._vis_topic = rospy.get_param("~topics/vis")
       

        # Frames.
        # TODO! You'll need to set values for class variables called:
        if not rospy.has_param("~frames/fixed"):
            return False
        self._fixed_frame = rospy.get_param("~frames/fixed")

        if not rospy.has_param("~frames/sensor"):
            return False
        self._sensor_frame = rospy.get_param("~frames/sensor")
        

        return True

    def RegisterCallbacks(self):
        # Subscriber to scan from indi_romi.py
        self._sensor_sub = rospy.Subscriber(self._sensor_topic, LaserScan, self.SensorCallback, queue_size=1)

        # Publishes Marker objects for rviz map visualization
        self._vis_pub = rospy.Publisher(self._vis_topic, Marker, queue_size=10)

        # Publishes Pose data for path_planner.py
        self._pose_pub = rospy.Publisher("pose", Pose, queue_size=10)

        # Publishes Filtered scan for path planner
        self._filter_pub = rospy.Publisher("filtered_scan", LaserScan, queue_size=1)

        return True

    # Callback to process sensor measurements.
    def SensorCallback(self, msg):
        print("LaserScan Msg: ", msg)
        if not self._initialized:
            rospy.logerr("%s: Was not initialized.", self._name)
            return

        # Get our current pose from TF.
        try:
            pose = self._tf_buffer.lookup_transform(self._fixed_frame, self._sensor_frame, rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # Writes an error message to the ROS log but does not raise an exception
            rospy.logerr("%s: Could not extract pose from TF.", self._name)
            return

        # Extract x, y coordinates and heading (yaw) angle of the turtlebot, 
        # assuming that the turtlebot is on the ground plane.
        sensor_x = pose.transform.translation.x
        sensor_y = pose.transform.translation.y
        
        if abs(pose.transform.translation.z) > 0.05:
            rospy.logwarn("%s: Turtlebot is not on ground plane.", self._name)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [pose.transform.rotation.x, pose.transform.rotation.y,
             pose.transform.rotation.z, pose.transform.rotation.w])
        if abs(roll) > 0.1 or abs(pitch) > 0.1:
            rospy.logwarn("%s: Turtlebot roll/pitch is too large.", self._name)

        ranges = []
        # Loop over all ranges in the LaserScan.
        for idx, r in enumerate(msg.ranges):
            # Randomly throw out some rays to speed this up.
            if np.random.rand() > self._random_downsample:
                continue
            elif np.isnan(r):
                continue

            # Get angle of this ray in fixed frame.
            angle_sensor_frame = msg.angle_min + idx*msg.angle_increment

            angle_fixed_frame = angle_sensor_frame + yaw

            # Throw out this point if it is too close or too far away.
            if r > msg.range_max:
                rospy.logwarn("%s: Range %f > %f was too large.",
                              self._name, r, msg.range_max)
                continue
            if r < msg.range_min:
                rospy.logwarn("%s: Range %f < %f was too small.",
                              self._name, r, msg.range_min)
                continue
            
            ranges.append(r)
            # Walk along this ray from the scan point to the sensor.
            # Update log-odds at each voxel along the way.
            # Only update each voxel once. 
            # The occupancy grid is stored in self._map
            y_vals = np.linspace(sensor_y, sensor_y + r*np.sin(angle_fixed_frame), 100)
            x_vals = np.linspace(sensor_x, sensor_x + r*np.cos(angle_fixed_frame), 100)

            curr_voxel = ()
            for i in range(len(x_vals)):
                prev_voxel = curr_voxel
                curr_voxel = self.PointToVoxel(x_vals[i], y_vals[i])
                if (curr_voxel == prev_voxel):
                    continue
                if (curr_voxel == self.PointToVoxel(sensor_x + r*np.cos(angle_fixed_frame), sensor_y + r*np.sin(angle_fixed_frame))):
                    if (self._map[curr_voxel[0], curr_voxel[1]] + self._occupied_update > self._occupied_threshold):
                        continue
                    self._map[curr_voxel[0], curr_voxel[1]] += self._occupied_update
                else:
                    if (self._map[curr_voxel[0], curr_voxel[1]] - self._occupied_update < self._free_threshold):
                        continue
                    self._map[curr_voxel[0], curr_voxel[1]] += self._free_update
                
        # Visualize.
        self.Visualize()

        # Publish pose
        point = Point()
        point.x = sensor_x
        point.y = sensor_y
        point.z = 0.0

        quat = Quaternion()
        quat.x = 0
        quat.y = 0
        quat.z = 0
        quat.w = 0

        pose = Pose()
        pose.position = point
        pose.orientation = quat
        self._pose_pub.publish(pose)

        # avg_scan = LaserScan()
        # avg_scan.header.stamp = rospy.Time.now()
        # avg_scan.header.frame_id = 'laser_frame'
        # avg_scan.angle_min = 0
        # avg_scan.angle_max = 0
        # avg_scan.angle_increment = 0
        # avg_scan.time_increment = 0
        # avg_scan.scan_time = 0
        # avg_scan.range_min = 0
        # avg_scan.range_max = 0
        # avg_scan.ranges = ranges
        # avg_scan.intensities = []
        # self._filter_pub.publish(avg_scan)

    # Convert (x, y) coordinates in fixed frame to grid coordinates.
    def PointToVoxel(self, x, y):
        grid_x = int((x - self._x_min) / self._x_res)
        grid_y = int((y - self._y_min) / self._y_res)

        return (grid_x, grid_y)

    # Get the center point (x, y) corresponding to the given voxel.
    def VoxelCenter(self, ii, jj):
        center_x = self._x_min + (0.5 + ii) * self._x_res
        center_y = self._y_min + (0.5 + jj) * self._y_res

        return (center_x, center_y)

    # Convert between probabity and log-odds.
    def ProbabilityToLogOdds(self, p):
        return np.log(p / (1.0 - p))

    def LogOddsToProbability(self, l):
        return 1.0 / (1.0 + np.exp(-l))

    # Colormap to take log odds at a voxel to a RGBA color.
    def Colormap(self, ii, jj):
        p = self.LogOddsToProbability(self._map[ii, jj])

        c = ColorRGBA()
        c.r = p
        c.g = 0.1
        c.b = 1.0 - p
        c.a = 0.75
        return c

    # Visualize the map as a collection of flat cubes instead of
    # as a built-in OccupancyGrid message, since that gives us more
    # flexibility for things like color maps and stuff.
    # See http://wiki.ros.org/rviz/DisplayTypes/Marker for a brief tutorial.
    def Visualize(self):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self._fixed_frame
        m.ns = "map"
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = self._x_res
        m.scale.y = self._y_res
        m.scale.z = 0.01
        #m.lifetime = 0
        m.frame_locked = True

        for ii in range(self._x_num):
            for jj in range(self._y_num):
                p = Point(0.0, 0.0, 0.0)
                (p.x, p.y) = self.VoxelCenter(ii, jj)

                m.points.append(p)
                m.colors.append(self.Colormap(ii, jj))

        self._vis_pub.publish(m)
