from __future__ import division
from __future__ import print_function

import rospy
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from enum import Enum


class PointType(Enum):
    COLOR = 0
    SEMANTIC_MAX = 1
    SEMANTIC_BAYESIAN = 2


class SemanticCloudGenerator:
    """
    """

    def __init__(self, width, height, frame_id, point_type):
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

        if self.point_type is PointType.SEMANTIC_BAYESIAN:
            self.ros_data = np.ones([width * height, 16], dtype = '<f4') 
        else:
            self.ros_data = np.ones([width * height, 8], dtype = '<f4')
        
        # Create semantic cloud
        self.semantic_cloud = PointCloud2()
        # Need cloud_raw.header.frame_id
        self.semantic_cloud.header.frame_id = frame_id
        #self.semantic_cloud.height = 1
        #self.semantic_cloud.width = width * height

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

        self.semantic_cloud.is_bigendian = False

        if self.point_type is PointType.SEMANTIC_BAYESIAN:
            self.semantic_cloud.point_step = 16 * 4 # In bytes
        else:
            self.semantic_cloud.point_step = 8 * 4 # In bytes
        
        #self.semantic_cloud.row_step = self.semantic_cloud.point_step * self.semantic_cloud.width * self.semantic_cloud.height
        self.semantic_cloud.is_dense = False


    def generate_cloud_semantic_max(self, bgr_img, cloud_raw, camera_mat, extrinsic_mat, max_semantic_color, max_confidence, stamp):
        """
        """
        # init semantic_data
        # for cloud size
            # get row and col
            # judge image range
            # add xyz
            # add rgb
            # add max semantic color
            # add max confidence
        # 不用每次都新建一个语义点云类型，重复使用，记得清空之前的数据
        self.semantic_cloud.points.clear()
        
        # Init with new cloud_raw
        cloud_size = cloud_raw.points.size()
        self.semantic_cloud.height = 1
        self.semantic_cloud.width = cloud_size
        self.semantic_cloud.row_step = self.semantic_cloud.point_step * self.semantic_cloud.width * self.semantic_cloud.height

        # Semantic data store semantic_cloud.data
        semantic_data = np.ones([cloud_size, 8], dtype = '<f4')
        raw_point = np.mat(np.zeros((4, 1)))
        cam_point = np.mat(np.zeros((4, 1)))

        fx = camera_mat[0, 0]
        fy = camera_mat[1, 1]
        cx = camera_mat[0, 2]
        cy = camera_mat[1, 2]

        for i in range(cloud_size):
            raw_point[0, 0] = cloud_raw.points[i].x
            raw_point[1, 0] = cloud_raw.points[i].y
            raw_point[2, 0] = cloud_raw.points[i].z
            raw_point[3, 0] = 1

            cam_point = extrinsic_mat * raw_point

            x = cam_point[0, 0]
            y = cam_point[1, 0]
            z = cam_point[2, 0]

            col = int(x * fx / z + cx)
            row = int(y * fy / z + cy)

            if (row >= 0) and (row < self.img_width) and (col >= 0) and (col < self.img_width) and (z > 0):
                '''
                '''



            
            



        self.semantic_cloud.data = np.getbuffer(semantic_data.ravel())[:]
        self.semantic_cloud.header.stamp = stamp;
        return self.semantic_cloud 
        


    def generate_cloud_semantic_bayesian(self, bgr_img, cloud_raw, bayes_semantic_colors, bayes_confidences, stamp):
        """
        """