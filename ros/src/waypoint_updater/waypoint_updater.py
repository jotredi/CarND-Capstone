#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5 # Max deceleration

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Subscriber for /traffic_waypoint
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Publisher for the final waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # base waypoints that the car should follow
        self.base_lane = None
        # current position
        self.pose = None
        # waypoint to stop at
        self.stopline_wp_idx = -1

        # for storing base waypoint as KDTree
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.waypoints_initialized = False
        self.pose_initialized = False

        self.loop()

    def loop(self):
        # reduced frecuency for being able to use camera in the simulator
        # (10 Hz instead of 50 Hz)
        rate = rospy.Rate(10) #Hz
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        """Identifies the closest waypoint index from current position"""

        # Current x,y position of the car
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self):
        """Publishes final waypoints for the car to follow"""
        if self.pose_initialized and self.waypoints_initialized:
            final_lane = self.generate_lane()
            self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        """Generates lane (slice of waypoints from base waypoints)
        that the car is going to follow with their corresponding
        velocities (taking traffic lights into account)

        Returns: final waypoints to publish
        """
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        # slice from base waypoints
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        # if no traffic light ahead
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            # use base waypoints as final waypoints
            lane.waypoints = base_waypoints
        else:
            # decelerate waypoints velocities for a smooth stop
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        """This function adjusts waypoint velocities for stopping at closest_idx

        Args:
            waypoints: waypoints to follow
            closest_idx: waypoint where the car should stop

        Returns:
            waypoint list with updated velocities
        """

        temp = [] # list to store new waypoints

        # Loop through base waypoints
        for i, wp in enumerate(waypoints):
    	    p = Waypoint()
    	    p.pose = wp.pose

            # take stop waypoint index 2 meters behind the line
            # (this considers car length)
    	    stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)

            # distance from current waypoint until stop waypoint
    	    dist = self.distance(waypoints, i, stop_idx)

            # Set waypoint velocity in terms of the distance
    	    vel = math.sqrt(2 * MAX_DECEL * dist)
    	    if vel < 1.:
                vel = 0.

            # consider speed limit
    	    p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
    	    temp.append(p)

        return temp

    def pose_cb(self, msg):
        """Gets current pose of the vehicle"""
        self.pose = msg
        self.pose_initialized = True

    def waypoints_cb(self, waypoints):
        """Stores base waypoints"""
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        self.waypoints_initialized = True

    def traffic_cb(self, msg):
        """Callback for /traffic_waypoint message"""
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
