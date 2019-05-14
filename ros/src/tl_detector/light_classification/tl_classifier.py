from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
from PIL import Image
import rospy
import os
import cv2

class TLClassifier(object):
    def __init__(self, is_site):
        #TODO load classifier

        if is_site:
            self.SSD_GRAPH_FILE_PATH = rospy.get_param('~/site_model_path', "not found")
        else:
            self.SSD_GRAPH_FILE_PATH = rospy.get_param('~/sim_model_path', "not found")

        rospy.logdebug('#### is_site %s? SSD_GRAPH_FILE_PATH read = %s',is_site, self.SSD_GRAPH_FILE_PATH)


        self.detection_graph = self.load_graph(self.SSD_GRAPH_FILE_PATH)
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')

        # The classification of the object (integer id).
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
