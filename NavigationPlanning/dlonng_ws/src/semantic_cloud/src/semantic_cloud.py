#!/usr/bin/env python

""" Fusion semantic cloud from ZED and Robosense-16

@author DLonng
@date 2020-07-30
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
import torch
import torch.nn.functional as F
import utils as ptutil

from torchvision import transforms
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from model.lednet import LEDNet

from semantic_cloud_generator import PointType, SemanticCloudGenerator


class SemanticCloud:
    """ Fusion semantic cloud from ZED and Robosense-16

    Take in an RGB image and Robosense - 16 PointCloud, Use CNN to do semantic segmantation
    Out put a cloud point with semantic color registered
    """

    def __init__(self, device, pretrained):
        """ Init semantic cloud

        init params, init CNN, init ROS

        Args:
            xxx: xxx

        Returns:
            xxx
        """

        point_type = rospy.get_param('/semantic_generator/point_type')

        # Select semantic point type
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
        self.init_cnn(device, pretrained)

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
        self.distcoeff_mat = calibration_fs.getNode('DistCoeff').mat()
        self.extrinsic_mat = calibration_fs.getNode('CameraExtrinsicMat').mat()
        self.extrinsic_mat_inv = np.linalg.inv(self.extrinsic_mat)

        calibration_fs.release()
        
        self.image_sub = rospy.Subscriber(rospy.get_param('/semantic_generator/image_raw_topic'), Image, self.image_callback, queue_size=1)
        self.cloud_sub = rospy.Subscriber(rospy.get_param('/semantic_generator/cloud_raw_topic'), PointCloud2, self.cloud_callback, queue_size=1)
        
        '''
        self.image_sub = message_filters.Subscriber(rospy.get_param('/semantic_generator/image_raw_topic'), Image, queue_size=1)
        self.cloud_sub = message_filters.Subscriber(rospy.get_param('/semantic_generator/cloud_raw_topic'), PointCloud2, queue_size=1)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.cloud_sub], queue_size=5, slop=0.3)
        self.ts.registerCallback(self.image_cloud_callback)
        '''
        
        self.sem_img_pub = rospy.Publisher(rospy.get_param('/semantic_generator/semantic_img_topic'), Image, queue_size=1)
        self.sem_cloud_pub = rospy.Publisher(rospy.get_param('/semantic_generator/semantic_cloud_topic'), PointCloud2, queue_size=1)

        self.sem_cloud_generator = SemanticCloudGenerator(self.img_width, self.img_height, self.point_type)

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
            # Or  to predict semantic img in image_callback ?
            max_semantic_img, max_confidence = self.predict_max(cv_img)
         
            # using in test!
            #max_semantic_img = cv_img
            #max_confidence = np.ones(self.img_width * self.img_height)

            # produce max semantic cloud
            semantic_cloud = self.sem_cloud_generator.generate_cloud_semantic_max(cv_img,
                                                                                  cloud_raw, 
                                                                                  self.camera_mat, 
                                                                                  self.extrinsic_mat_inv, 
                                                                                  max_semantic_img, 
                                                                                  max_confidence)

        else:
            print('produce bayes semantic cloud')
            
            # produce bayes_semantic_imgs and bayes_confidences
            self.predict_bayesian(cv_img)

            # produce bayes semantic cloud
            semantic_cloud = self.sem_cloud_generator.generate_cloud_semantic_bayesian(cv_img, 
                                                                                       cloud_raw, 
                                                                                       self.camera_mat, 
                                                                                       self.extrinsic_mat_inv, 
                                                                                       self.bayes_semantic_imgs,
                                                                                       self.bayes_confidences)

        # Publihser semantic img
        if self.sem_cloud_pub.get_num_connections() > 0:
            if self.point_type is PointType.SEMANTIC_MAX:
                semantic_img_msg = self.bridge.cv2_to_imgmsg(max_semantic_img, encoding="bgr8")
            else:
                semantic_img_msg = self.bridge.cv2_to_imgmsg(self.bayes_semantic_imgs[0], encoding="bgr8")

            self.sem_img_pub.publish(semantic_img_msg)

        self.sem_cloud_pub.publish(semantic_cloud)


    def image_callback(self, image_raw):
        self.cv_img = self.bridge.imgmsg_to_cv2(image_raw, "bgr8")
        self.max_semantic_img, self.max_confidence = self.predict_max(self.cv_img)

    def cloud_callback(self, cloud_raw):
        if self.point_type is PointType.COLOR:
            print('No use color point cloud')
        elif self.point_type is PointType.SEMANTIC_MAX:
            print('produce max semantic cloud')

            # produce max semantic cloud
            semantic_cloud = self.sem_cloud_generator.generate_cloud_semantic_max(self.cv_img,
                                                                                  cloud_raw, 
                                                                                  self.camera_mat, 
                                                                                  self.extrinsic_mat_inv, 
                                                                                  self.max_semantic_img, 
                                                                                  self.max_confidence)

        else:
            print('produce bayes semantic cloud')
            
            # produce bayes_semantic_imgs and bayes_confidences
            self.predict_bayesian(self.cv_img)

            # produce bayes semantic cloud
            semantic_cloud = self.sem_cloud_generator.generate_cloud_semantic_bayesian(self.cv_img, 
                                                                                       cloud_raw, 
                                                                                       self.bayes_semantic_imgs,
                                                                                       self.bayes_confidences,
                                                                                       cloud_raw.header.stamp)
            print('produce bayes semantic cloud ok!')

        # Publihser semantic img
        if self.sem_cloud_pub.get_num_connections() > 0:
            if self.point_type is PointType.SEMANTIC_MAX:
                semantic_img_msg = self.bridge.cv2_to_imgmsg(self.max_semantic_img, encoding="bgr8")
            else:
                semantic_img_msg = self.bridge.cv2_to_imgmsg(self.bayes_semantic_imgs[0], encoding="bgr8")

            self.sem_img_pub.publish(semantic_img_msg)

        self.sem_cloud_pub.publish(semantic_cloud)


    def init_cnn(self, device, pretrained):
        """
            No test!
        """
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

        self.device = device
        self.model = LEDNet(6).to(device)
        self.model.load_state_dict(torch.load(pretrained))
        self.model.eval()


    def predict_max(self, cv_img):
        """
            No test!
        Args:
            cv_img: self.bridge.imgmsg_to_cv2(image_raw, "bgr8")

        Returns:
            max_semantic_img
            max_confidence
        """
        pilImg = self.cv2PIL(cv_img, cv2.COLOR_BGR2RGB)
        img = self.transform(pilImg).unsqueeze(0).to(self.device)
	
        with torch.no_grad():
            output = self.model(img)
            confidence = F.softmax(output,1).max(1)
            confidence = confidence[0]
            confidence = confidence.squeeze(0).cpu().numpy()
            confidence = confidence.reshape(-1)
        
        predict = torch.argmax(output, 1).squeeze(0).cpu().numpy()
        mask = ptutil.get_color_pallete(predict, 'citys')
        mask.save(os.path.join('/home/zmx/catkin_ws/src/semantic_cloud/', 'output.png'))
        mask = cv2.imread(os.path.join('/home/zmx/catkin_ws/src/semantic_cloud/', 'output.png'))
        #return max_semantic_img, max_confidence
        return mask, confidence

    def predict_bayesian(self, cv_img):
        """
            No test!

        Args:
            cv_img: self.bridge.imgmsg_to_cv2(image_raw, "bgr8")
        """
        pilImg = self.cv2PIL(cv_img, cv2.COLOR_BGR2RGB)
        img = self.transform(pilImg).unsqueeze(0).to(self.device)

        with torch.no_grad():
            output = self.model(img)
            output = F.softmax(output,1)
            confidence, pred_labal = torch.topk(input = output, k=3, dim=1, largest=True,sorted=True)
        
        pred_labal = pred_labal.squeeze(0).cpu().numpy()
        confidence = confidence[0].squeeze(0).cpu().numpy()
        
        #pre_confidence = confidence[0].reshape(-1)
        #pre_confidence1 = confidence[1].reshape(-1)
        #pre_confidence2 = confidence[2].reshape(-1)

        self.bayes_confidences[0] = confidence[0].reshape(-1)
        self.bayes_confidences[1] = confidence[1].reshape(-1)
        self.bayes_confidences[2] = confidence[2].reshape(-1)

        #mask = ptutil.get_color_pallete(pred_labal[0], 'citys')
        #mask2 = ptutil.get_color_pallete(pred_labal[1], 'citys')
        #mask3 = ptutil.get_color_pallete(pred_labal[2], 'citys')
        self.bayes_semantic_imgs[0] = ptutil.get_color_pallete(pred_labal[0], 'citys')
        self.bayes_semantic_imgs[1] = ptutil.get_color_pallete(pred_labal[1], 'citys')
        self.bayes_semantic_imgs[2] = ptutil.get_color_pallete(pred_labal[2], 'citys')

        # don't need return bayes_semantic_imgs and bayes_confidences


    def cv2PIL(self, img, colorChange):
        image = PIL.Image.fromarray(cv2.cvtColor(img, colorChange))
        return image


    def PIL2cv(self, img, colorChange):
        image = cv2.cvtColor(np.asarray(img), colorChange)
        return image



if __name__ == '__main__':
    rospy.init_node('semantic_cloud')

    device = torch.device('cuda')
    
    pretrained = '/home/zmx/catkin_ws/src/semantic_cloud/LEDNet_model_best_6.pth'

    seg_cnn = SemanticCloud(device, pretrained)

    print("Hello World!")

    rospy.spin()

