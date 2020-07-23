from styx_msgs.msg import TrafficLight
from tl_detection.detector import traffic_light_detector

import rospy
import numpy as np
import tensorflow as tf
import cv2
import os

class TLClassifier(object):
    def __init__(self, detection_path, classifier_path):
        # Load tl detector
        self.tld = traffic_light_detector(detection_path)

        # Load tl classifier
        self.sess = tf.Session()

        saver = tf.train.import_meta_graph(os.path.join(classifier_path, 'model.meta'))
        saver.restore(self.sess, os.path.join(classifier_path, 'model'))

        self.graph = tf.get_default_graph()
        self.input = self.graph.get_tensor_by_name("input:0")
        self.logits = self.graph.get_tensor_by_name("logits:0")
        self.prediction = tf.argmax(self.logits, 1)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Detect bounding boxes
        box_coords, _ = self.tld.predict(image)

        # tl detections in one image
        detections = []

        # For each bounding box
        for i in range(len(box_coords)):
            # Get coordinates
            bot, left, top, right = box_coords[i, ...]

            # Crop image
            cropped_image = image[int(bot):int(top), int(left):int(right)]

            # Resize image to dimensions accepted by classifier (24x12)
            resized = cv2.resize(cropped_image, (12,24), interpolation = cv2.INTER_AREA)

            # Normalize image
            norm = (resized - 128.0)/128.0

            detections.append(norm)

        # Get classification
        if len(detections):
            predictions = self.sess.run(self.logits, feed_dict={self.input: np.array(detections)})

            rospy.loginfo(predictions)

        # Return true if traffic lights detected
        if len(box_coords):
            return True
        else:
            return False

        #TODO implement light color prediction
        #return TrafficLight.UNKNOWN
