#!/usr/bin/env python

"""Fusion semantic cloud from ZED and Robosense-16

@author DLonng
@date 2020-07-30

https://zh-google-styleguide.readthedocs.io/en/latest/google-python-styleguide/python_style_rules/#
"""

from __future__ import division
from __future__ import print_function

import rospy
import os
import sys
import PIL

import numpy as np
import message_filters
import cv2

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from cv_bridge import CvBridge, CvBridgeError

from semantic_cloud_generator import PointType, SemanticCloudGenerator


class SemanticCloud:
    """ Fusion semantic cloud from ZED and Robosense-16

    Take in an RGB image and Robosense - 16 PointCloud, Use CNN to do semantic segmantation
    Out put a cloud point with semantic color registered
    """

    def __init__(self):
        """ Init semantic cloud

        init params, init CNN, init ROS

        Args:
            xxx: xxx

        Returns:
            xxx
        """

        # Init cloud type
        point_type = rospy.get_param('/semantic_generator/point_type')

        if 0 == point_type:
            self.point_type = PointType.COLOR
            print('Generatr color point cloud.')
        elif 1 == point_type:
            self.point_type = PointType.SEMANTIC_MAX
            print('Generatr semantic point cloud [max fusion].')
        elif 2 == point_type:
            self.point_type = PointType.SEMANTIC_BAYESIAN
            print('Generatr semantic point cloud [bayesian fusion].')
        else:
            print('Invalid point type.')
            return

        # Need read from yaml file.
        self.img_width, self.img_height = rospy.get_param('/camera/width'), rospy.get_param('/camera/height')

        # Init CNN
        print('Init CNN')

        # Init semantic cloud
        # Bayesian: semantic_colors, confidences
        #      Max: semantic_color, confidence
        if self.point_type is PointType.SEMANTIC_BAYESIAN:
            self.bayes_semantic_imgs = np.zeros((3, self.img_height, self.img_width, 3), dtype = np.uint8)
            self.bayes_confidences = np.zeros((3, self.img_height, self.img_width), dtype = np.float32)

        # Init ROS
        print('Init ROS')

        self.bridge = CvBridge()

        # Get camera info
        calibration_fs = cv2.FileStorage(rospy.get_param('/semantic_generator/calibration_file'), cv2.FILE_STORAGE_READ)

        self.camera_mat = calibration_fs.getNode('CameraMat').mat()
        self.fx = self.camera_mat[0][0]
        self.fy = self.camera_mat[1][1]
        self.cx = self.camera_mat[0][2]
        self.cy = self.camera_mat[1][2]
        print(self.camera_mat)

        self.distcoeff_mat = calibration_fs.getNode('DistCoeff').mat()
        self.extrinsic_mat = calibration_fs.getNode('CameraExtrinsicMat').mat()

        calibration_fs.release()

        # Image subscriber
        self.image_sub = message_filters.Subscriber(rospy.get_param('/semantic_generator/image_raw_topic'), Image, queue_size=1)
        # Point Cloud subscriber
        self.cloud_sub = message_filters.Subscriber(rospy.get_param('/semantic_generator/cloud_raw_topic'), PointCloud2, queue_size=1)

        # Semantic img publisher
        self.sem_img_pub = rospy.Publisher(rospy.get_param('/semantic_generator/semantic_img_topic'), Image, queue_size=1)
        # Semantic cloud publisher
        self.sem_cloud_pub = rospy.Publisher(rospy.get_param('/semantic_generator/semantic_cloud_topic'), PointCloud2, queue_size=1)

        # Need to study!
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.cloud_sub], queue_size=1, slop=0.3)
        
        # go!
        self.ts.registerCallback(self.image_cloud_callback)

        self.sem_cloud_generator = SemanticCloudGenerator()

        print('Init OK')

    def image_cloud_callback(self, image_raw, cloud_raw):
        """ Produce semantic cloud with semantic img and raw cloud

        Args:
            image_raw: raw zed left image (sensor_msgs.Image)
            cloud_raw: raw robosense-16 lidar point cloud (sensor_msgs.PointCloud2)

        """
        cv_img = self.bridge.imgmsg_to_cv2(image_raw, "bgr8")

        if self.point_type is PointType.COLOR:
            print('No use color point cloud')
        elif self.point_type is PointType.SEMANTIC_MAX:
            print('produce max semantic cloud')
            
            max_semantic_img, max_confidence = []#self.predict_max(cv_img)

            # produce max semantic cloud
            semantic_cloud = self.sem_cloud_generator.generate_cloud_semantic_max(cv_img,
                                                                                  cloud_raw, 
                                                                                  extrinsic_mat, 
                                                                                  max_semantic_img, 
                                                                                  max_confidence, 
                                                                                  image_raw.header.stamp)

        else:
            print('produce bayes semantic cloud')
            
            # produce bayes_semantic_imgs and bayes_confidences
            self.predict_bayesian(cv_img)

            # produce bayes semantic cloud
            semantic_cloud = self.sem_cloud_generator.generate_cloud_semantic_bayesian(cv_img, 
                                                                                       cloud_raw, 
                                                                                       self.bayes_semantic_imgs,
                                                                                       self.bayes_confidences,
                                                                                       image_raw.header.stamp)

        # Publihser semantic img
        if self.sem_cloud_pub.get_num_connections() > 0:
            if self.point_type is PointType.SEMANTIC_MAX:
                #semantic_img_msg = self.bridge.cv2_to_imgmsg(max_semantic_img, encoding="bgr8")
                print('Todo...')
            else:
                #semantic_img_msg = self.bridge.cv2_to_imgmsg(self.bayes_semantic_imgs[0], encoding="bgr8")
                print('Todo...')
            #self.sem_img_pub.publish(semantic_img_msg)

        self.sem_cloud_pub.publish(semantic_cloud)

    def predict(self, cv_img):
        """
        """

    def predict_max(self, cv_img):
        """
        """

    def predict_bayesian(self, cv_img):
        """
        """


    def cv2PIL(self, img, colorChange):
        image = PIL.Image.fromarray(cv2.cvtColor(img, colorChange))
        return image


    def PIL2cv(self, img, colorChange):
        image = cv2.cvtColor(np.asarray(img), colorChange)
        return image


if __name__ == '__main__':
    rospy.init_node('semantic_cloud')

    seg_cnn = SemanticCloud()

    print("Hello World!")

    rospy.spin()
