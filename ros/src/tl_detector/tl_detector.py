#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import numpy as np
import tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3
GAP = 10
USE_CLASSIFIER = False 

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.waypoints_tree = None
        self.num_base_waypoints = 0
        self.camera_image = None
        self.lights = []
        self.image_count = 0

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.last_img_cb_time = rospy.get_time()
        rospy.logdebug("init last_img_cb_time %d ",self.last_img_cb_time)


        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.is_site = self.config['is_site']

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.is_site)
        self.listener = tf.TransformListener()

        rospy.spin()

    def print_image_process_timestamp(self):
        self.last_img_cb_time = rospy.get_time()
        rospy.logdebug("last_img_cb_time %d ",self.last_img_cb_time)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            rospy.logdebug("Starting waypoints_cb()")
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
            self.num_base_waypoints = len(self.waypoints_2d)
            rospy.logdebug("Number of base waypoints: %s", self.num_base_waypoints)
            rospy.logdebug("Leaving waypoints_cb()")

    def traffic_cb(self, msg):
        if self.image_count % GAP != 0:
            return
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        
        # Only process 1 in every 3 images. 
        self.image_count += 1
        if self.image_count % GAP != 0:
            return

        self.print_image_process_timestamp()
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        x = pose.position.x
        y = pose.position.y
        return self.get_closest_waypoint_coord(x, y)

    def get_closest_waypoint_coord(self,x,y):
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

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if USE_CLASSIFIER:
            if(not self.has_image):
                self.prev_light_loc = None
                return TrafficLight.UNKNOWN 

            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

            #Get classification
            return self.light_classifier.get_classification(cv_image)
        else:
            return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        closest_line_idx = None # Index of waypoint closest to the traffic line of closest_light

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        if(self.pose and self.waypoint_tree):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose)
            #rospy.logdebug("tl_detector - latest car_position_wp  %d", car_position_wp)

            max_diff = 100 #self.num_base_waypoints # how many waypoints you look ahead, lower for performance. 

            #TODO find the closest visible traffic light (if one exists)
            for i, cur_light in enumerate(self.lights):
                # Get stop line waypoint indicies
                line = stop_line_positions[i]
                line_wp_idx = self.get_closest_waypoint_coord(line[0],line[1]) 
                # Determine distance (in waypoints) between car and stoplight
                dist = line_wp_idx - car_wp_idx
                # If line is within max distance, try and find closer stop light.
                if dist >=0 and dist < max_diff:
                    max_diff = dist
                    closest_light = cur_light
                    closest_line_idx = line_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            rospy.loginfo('Found traffic light ' + str(state))
            return closest_line_idx, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
