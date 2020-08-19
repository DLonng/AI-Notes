from __future__ import division
from __future__ import print_function

import rospy
import numpy as np
import types


from sensor_msgs.msg import PointCloud2, PointField

import sensor_msgs.point_cloud2 as pc2

from enum import Enum


class PointType(Enum):
    COLOR = 0
    SEMANTIC_MAX = 1
    SEMANTIC_BAYESIAN = 2


class SemanticCloudGenerator:
    """
    """

    def __init__(self, width, height, point_type):
        """ Init semantic cloud
        
        Max semantic cloud:
             [x, y, z, 0, bgr0, semantics, confidence, 0]
             
        Bayes semantic cloud:
            [x, y, z, 0, bgr0, 0, 0, 0, color0, color1, color2, 0, confidence0, confidence1, confidence2, 0]

        Args:
            width:
            height:
            frame_id:
            point_type:
        """
        self.img_width = width
        self.img_height = height
        self.point_type = point_type

        self.bgr0_vect = np.zeros([self.img_width * self.img_height, 4], dtype = '<u1')
        self.semantic_color_vect = np.zeros([self.img_width * self.img_height, 4], dtype = '<u1')
        
        # Create semantic cloud
        self.semantic_cloud = PointCloud2()
        self.semantic_cloud.height = 1
        self.semantic_cloud.width = self.img_width * self.img_width
        self.semantic_cloud.is_bigendian = False
       
        if self.point_type is PointType.SEMANTIC_MAX:
            self.semantic_cloud.point_step = 8 * 4
        elif self.point_type is PointType.SEMANTIC_BAYESIAN:
            self.semantic_cloud.point_step = 16 * 4

        self.semantic_cloud.row_step = self.semantic_cloud.point_step * self.semantic_cloud.width * self.semantic_cloud.height
        self.semantic_cloud.is_dense = False


        # Append: x y z 0 bgr0
        self.semantic_cloud.fields.append(PointField(
                             name = "x",
                             offset = 0,
                             datatype = PointField.FLOAT32, count = 1))
        self.semantic_cloud.fields.append(PointField(
                             name = "y",
                             offset = 4,
                             datatype = PointField.FLOAT32, count = 1))
        self.semantic_cloud.fields.append(PointField(
                             name = "z",
                             offset = 8,
                             datatype = PointField.FLOAT32, count = 1))
        self.semantic_cloud.fields.append(PointField(
                             name = "rgb",
                             offset = 16,
                             datatype = PointField.FLOAT32, count = 1))                  

        # Max semantic cloud
        # [x, y, z, 0, bgr0, semantics, confidence, 0]
        if self.point_type is PointType.SEMANTIC_MAX:
            self.semantic_cloud.fields.append(PointField(
                            name = "semantic_color",
                            offset = 20,
                            datatype = PointField.FLOAT32, count = 1))
            self.semantic_cloud.fields.append(PointField(
                            name = "confidence",
                            offset = 24,
                            datatype = PointField.FLOAT32, count = 1))
        # Bayes semantic cloud
        # [x, y, z, 0, bgr0, 0, 0, 0, color0, color1, color2, 0, confidence0, confidence1, confidence2, 0]        
        elif self.point_type is PointType.SEMANTIC_BAYESIAN:
        
            self.semantic_cloud.fields.append(PointField(
                                name = "semantic_color1",
                                offset = 32,
                                datatype = PointField.FLOAT32, count = 1))
            self.semantic_cloud.fields.append(PointField(
                                name = "semantic_color2",
                                offset = 36,
                                datatype = PointField.FLOAT32, count = 1))
            self.semantic_cloud.fields.append(PointField(
                                name = "semantic_color3",
                                offset = 40,
                                datatype = PointField.FLOAT32, count = 1))
            self.semantic_cloud.fields.append(PointField(
                                name = "confidence1",
                                offset = 48,
                                datatype = PointField.FLOAT32, count = 1))
            self.semantic_cloud.fields.append(PointField(
                                name = "confidence2",
                                offset = 52,
                                datatype = PointField.FLOAT32, count = 1))
            self.semantic_cloud.fields.append(PointField(
                                name = "confidence3",
                                offset = 56,
                                datatype = PointField.FLOAT32, count = 1))
            

    def generate_cloud_semantic_max(self, cv_img, cloud_raw, camera_mat, extrinsic_mat, max_semantic_color, max_confidence):
        """
            Need to clear cloud_raw.data
        """

        cloud_size = cloud_raw.width * cloud_raw.height
        self.semantic_cloud.width = cloud_size

        # Semantic data store semantic_cloud.data
        semantic_data = np.ones([cloud_size, 8], dtype = '<f4')
        raw_point = np.mat(np.zeros((4, 1)))
        cam_point = np.mat(np.zeros((4, 1)))

        # Get RGB
        self.bgr0_vect[:, 0:1] = cv_img[:, :, 0].reshape(-1, 1)
        self.bgr0_vect[:, 1:2] = cv_img[:, :, 1].reshape(-1, 1)
        self.bgr0_vect[:, 2:3] = cv_img[:, :, 2].reshape(-1, 1)

        # Get semantic color
        self.semantic_color_vect[:, 0:1] = max_semantic_color[:, :, 0].reshape(-1, 1)
        self.semantic_color_vect[:, 1:2] = max_semantic_color[:, :, 1].reshape(-1, 1)
        self.semantic_color_vect[:, 2:3] = max_semantic_color[:, :, 2].reshape(-1, 1)

        fx = camera_mat[0, 0]
        fy = camera_mat[1, 1]
        cx = camera_mat[0, 2]
        cy = camera_mat[1, 2]
        
        i = 0
        for point in pc2.read_points(cloud_raw, skip_nans = True):

            raw_point[0, 0] = point[0]
            raw_point[1, 0] = point[1]
            raw_point[2, 0] = point[2]
            raw_point[3, 0] = 1

            cam_point = extrinsic_mat * raw_point

            x = cam_point[0, 0]
            y = cam_point[1, 0]
            z = cam_point[2, 0]

            col = int(x * fx / z + cx)
            row = int(y * fy / z + cy)

            if (row >= 0) and (row < self.img_width) and (col >= 0) and (col < self.img_width) and (z > 0):
                # x, y, z
                semantic_data[i][0] = point[0]
                semantic_data[i][1] = point[1]
                semantic_data[i][2] = point[2]
                # 0
                semantic_data[i][3] = 0
                # bgr0
                semantic_data[i][4] = self.bgr0_vect[row * self.img_width + col].view('<f4')
                # semantic color
                semantic_data[i][5] = self.semantic_color_vect[row * self.img_width + col].view('<f4')
                # confidence
                semantic_data[i][6] = max_confidence[row * self.img_width + col]
                # 0
                semantic_data[i][7] = 0

                i = i + 1

        self.semantic_cloud.data = np.getbuffer(semantic_data.ravel())[:]
        self.semantic_cloud.header = cloud_raw.header
        
        return self.semantic_cloud

    def generate_cloud_semantic_bayesian(self, cv_img, cloud_raw, camera_mat, extrinsic_mat, bayes_semantic_colors, bayes_confidences):
        """
        """
