#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import numpy as np
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

LOOKAHEAD_WPS = 120 # Number of waypoints we will publish. You can change this number [Original: 200, Owen: 30]
MAX_DECEL = 1.5    # try 0.5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        # TODO: Add other member variables you need below
        self.pose = None                 # of type PoseStamped
        self.base_lane = None       # of type Lane
        self.waypoints_2d = None         # N by [x, y] array of base_waypoint [x, y] coords 
        self.waypoint_tree = None        # KDTree of waypoints_2d
        self.num_base_waypoints = None   # len(waypoints_2d)
        self.prev_idx = -1               # track how often it changes
        self.prev_pose = None            # track how often pose updates
        self.stopline_wp_idx = -1
        self.count = 0
        self.start =True

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below (Dont Use for Now)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()
        
        #rospy.spin()
    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.num_base_waypoints:
                # Get closest waypoints
                closest_waypoint_idx = self.find_closest_waypoint_idx()
                #self.check_new_idx(closest_waypoint_idx)
                if self.start == True:
                    self.publish_waypoints(closest_waypoint_idx)
                    self.start = False
                    self.count += 1
                elif self.start == False and self.count == 2:        # done so you update waypoints every 3 cycles
                    self.publish_waypoints(closest_waypoint_idx)
                    self.count = 0
                self.count += 1
                #closest_waypoint_idx = self.find_closest_waypoint_idx()
                #self.check_new_idx(closest_waypoint_idx)
                #self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def check_new_idx(self,new_idx):
        if not (new_idx == self.prev_idx):
            rospy.logdebug("New Waypoint")
        self.prev_idx = new_idx

    def check_new_pose(self):
        if not self.prev_pose:
            self.prev_pose = self.pose
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        thresh = 0.01
        dist = dl(self.prev_pose.pose.position, self.pose.pose.position)
        if (dist > thresh):
            rospy.logdebug("New Pose; distance change: %s", dist)
        self.prev_pose = self.pose

    def publish_waypoints(self,closest_wp_idx):
        #waypoint_list = Lane()
        #waypoint_list.header = self.base_lane.header
        #waypoint_list.waypoints = self.base_lane.waypoints[closest_wp_idx:closest_wp_idx + LOOKAHEAD_WPS]
        #self.final_waypoints_pub.publish(waypoint_list)
        
        final_waypoints = self.generate_lane(closest_wp_idx)
        self.final_waypoints_pub.publish(final_waypoints)
        rospy.logdebug("Publishing Waypoints")
        rospy.logdebug("Closest IDX: %s", closest_wp_idx)

    def generate_lane(self, closest_idx):
        waypoint_list = Lane()

        #closest_idx = self.find_closest_waypoint_idx()
        furthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:furthest_idx]

        # if light is green at stopline (i.e. = -1) or stopline is too far away, dont decelerat
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= furthest_idx):
            waypoint_list.waypoints = base_waypoints
        # else, decelerate waypoints accordingly
        else:
            waypoint_list.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        
        return waypoint_list

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) #Two waypoints from line so front of car stop at line
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1:
                vel = 0
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp

    def find_closest_waypoint_idx(self):

        # Get Vehicle Coords
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        # Query for closest waypoint to car
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]

        # Use Hyperplane to determine if waypoint is ahead or behind
        # [1] Get Closest Coord and closest_idx-1 coord 
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        #[2] Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % self.num_base_waypoints # modular to wrap around if closest_idx is last wp
        return closest_idx    

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        #rospy.logdebug("Pulling Pose")
        #self.check_new_pose()

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_lane = waypoints
        if not self.waypoints_2d:
            rospy.logdebug("Starting waypoints_cb()")
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
            self.num_base_waypoints = len(self.waypoints_2d)
            rospy.logdebug("Number of base waypoints: %s", self.num_base_waypoints)
            rospy.logdebug("Leaving waypoints_cb()")

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        rospy.logdebug("Including Stop Light Waypoint: %s", self.stopline_wp_idx)

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
