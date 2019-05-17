from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
from PIL import Image
import rospy
import os
import cv2
import datetime


USING_CLASSIFIER = True
SIMPLE = False

class TLClassifier(object):
    def __init__(self, is_site):
        #TODO load classifier

        if USING_CLASSIFIER: 
            if is_site:
                self.SSD_GRAPH_FILE_PATH = rospy.get_param('~/site_model_path', "not found")
            else:
                self.SSD_GRAPH_FILE_PATH = rospy.get_param('~/sim_model_path', "not found")

            rospy.logdebug('#### is_site %s? SSD_GRAPH_FILE_PATH read = %s',is_site, self.SSD_GRAPH_FILE_PATH)


            #self.image_tensor = None
            #self.detection_boxes = None
            #self.detection_scores = None
            #self.detection_classes = None
            #self.num_detections = None

            self.detection_graph = self.load_graph(self.SSD_GRAPH_FILE_PATH)
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')

            # The classification of the object (integer id).
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')

            #self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

            # Start session once
            self.sess = tf.Session(graph=self.detection_graph)
        else:
            pass

    def filter_boxes(self,min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                idxs.append(i)

        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    def load_graph(self,graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            
            #self.image_tensor = graph.get_tensor_by_name('image_tensor:0')
            #self.detection_boxes = graph.get_tensor_by_name('detection_boxes:0')
            #self.detection_scores = graph.get_tensor_by_name('detection_scores:0')
            #self.detection_classes = graph.get_tensor_by_name('detection_classes:0')
            #self.num_detections = graph.get_tensor_by_name('num_detections:0')

        # graph_file_1='models/ssd_sim/frozen_inference_graph.pb'
        # with open(graph_file_1, 'rb') as f:
        #     serialized = f.read()
        #     detector_graph_def = tf.GraphDef()
        #     detector_graph_def.ParseFromString(serialized)
        #     tf.import_graph_def(detector_graph_def, name='detector')
        return graph



    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)
        #image_np = np.expand_dims(image, axis=0)

        # Actual detection.
        start = datetime.datetime.now()
        rospy.loginfo('Starting Classification')
        (boxes, scores, classes) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes],
            feed_dict={self.image_tensor: image_np})

        end = datetime.datetime.now()
        timeDiff = end-start
        rospy.loginfo('Total Classification Time: %s', timeDiff)

        # Remove unnecessary dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        if SIMPLE == True:
            print('Scores: ', scores[0:10])
            print('Classes: ', classes[0:10])

            threshold = 0.5

            if scores[1] > threshold:
                if classes[1] == 1:
                    print('GREEN')
                    return TrafficLight.GREEN
                elif classes[1] == 2:
                    print('RED')
                    return TrafficLight.RED
                elif classes[1] == 3:
                    print('YELLOW')
                    return TrafficLight.YELLOW
            return TrafficLight.RED
        else: 

            #print('Scores: ', scores[0])
            #print('Classes: ', classes[0])

            confidence_cutoff = 0.5 #before 0.8
            # Filter boxes with a confidence score less than `confidence_cutoff`
            boxes, scores, classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes)
            #
            # # The current box coordinates are normalized to a range between 0 and 1.
            # # This converts the coordinates actual location on the image.
            # width, height = image.size
            # box_coords = to_image_coords(boxes, height, width)
            #
            # # Each class with be represented by a differently colored box
            # draw_boxes(image, box_coords, classes)
            #
            # plt.figure(figsize=(12, 8))
            # plt.imshow(image)
            # Changing Default output to RED as greenlight detection is very accurate. 
            # Yellow lights are detected as green; thus, if the light is not green or yellow, it must be red. 



            if len(classes) == 0:
                return TrafficLight.RED

            switcher = {
                1: TrafficLight.GREEN,
                2: TrafficLight.RED,
                3: TrafficLight.YELLOW,
                }

            class_result = switcher.get(classes[0], TrafficLight.RED)
            rospy.logdebug('####classification class = %d', class_result)
            return class_result
