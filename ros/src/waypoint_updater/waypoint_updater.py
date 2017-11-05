#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf

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

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Internal members
        self.waypoints = None
        self.nb_wp = 0
        self.pose = None
        self.last_idx = -1

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose
        
        if self.waypoints is None:
            return
        
        if self.last_idx < 0:
            # initial case
            self.initial_index_update()
        
        self.index_update()

    def waypoints_cb(self, waypoints):
        if self.waypoints is None:
            self.waypoints = waypoints.waypoints
            self.nb_wp = len(self.waypoints)
        else:
            rospy.logerr('We have got more than one base waypoint definition')

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
    
    def initial_index_update(self):
        rospy.logdebug("Initial index update")
        # find the closest WP (will manage the car direction afterwards, using the index_update function, called in the callback)
        min_dist = float("inf")
        min_idx = -1
        
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        
        #rospy.logerr("Initial called %s", self.pose)
        for idx, wp in enumerate(self.waypoints):
            d = dl(self.pose.position, wp.pose.pose.position)
            if d <= min_dist:
                min_dist = d
                self.last_idx = idx
        
        if self.last_idx == -1:
            rospy.logerr("Something has gone horribly wrong here, probably an empty WP initialisation?")
        else:
            rospy.loginfo("First waypoint index at initialisation: %s", self.last_idx)
            
    def index_update(self):
        # check if the wp is in front of car. If not, takes the next one
        
        # we're a car, 2D geometry should be good enough
        _,_,yaw = tf.transformations.euler_from_quaternion(
            [self.pose.orientation.x,
             self.pose.orientation.y,
             self.pose.orientation.z,
             self.pose.orientation.w])
        
        while self.last_idx < self.nb_wp:
            dx = self.waypoints[self.last_idx].pose.pose.position.x-self.pose.position.x
            dy = self.waypoints[self.last_idx].pose.pose.position.y-self.pose.position.y
            
            dx_in_car = math.cos(yaw)*dx + math.sin(yaw)*dy
            if dx > 0:
                break
            self.last_idx += 1
        rospy.logdebug("Current WP index: %s", self.last_idx)
        
        lane = Lane()
        lane.waypoints = self.waypoints[self.last_idx:self.last_idx+LOOKAHEAD_WPS]
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        
        self.final_waypoints_pub.publish(lane)
        


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
