from __future__ import division
from __future__ import print_function

import rospy
import numpy as np

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
        
        # Create semantic cloud
        self.semantic_cloud = PointCloud2()

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


    def generate_cloud_semantic_max(self, cv_img, cloud_raw, camera_mat, extrinsic_mat, max_semantic_color, max_confidence, stamp):
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
        #self.semantic_cloud.data

        cloud_xyz = self.pointcloud2_to_xyz_array(cloud_raw)
        
    
        cloud_size = cloud_raw.width * cloud_raw.height

        # Init with new cloud_raw
        self.semantic_cloud.header.frame_id = cloud_raw.header.frame_id
        self.semantic_cloud.height = 1
        self.semantic_cloud.width = cloud_size
        self.semantic_cloud.row_step = self.semantic_cloud.point_step * self.semantic_cloud.width * self.semantic_cloud.height
        # In Bytes
        self.semantic_cloud.point_step = 8 * 4
        self.semantic_cloud.is_bigendian = False
        self.semantic_cloud.is_dense = False

        # Semantic data store semantic_cloud.data
        semantic_data = np.ones([cloud_size, 8], dtype = '<f4')
        raw_point = np.mat(np.zeros((4, 1)))
        cam_point = np.mat(np.zeros((4, 1)))

        fx = camera_mat[0, 0]
        fy = camera_mat[1, 1]
        cx = camera_mat[0, 2]
        cy = camera_mat[1, 2]

        for i in range(cloud_size):
            raw_point[0, 0] = cloud_xyz[i, 0]
            raw_point[1, 0] = cloud_xyz[i, 1]
            raw_point[2, 0] = cloud_xyz[i, 2]
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
                # x, y, z
                semantic_data[i][0] = cloud_xyz[i, 0]
                semantic_data[i][1] = cloud_xyz[i, 1]
                semantic_data[i][2] = cloud_xyz[i, 2]
                # 0
                semantic_data[i][3] = 0
                # bgr0
                semantic_data[i][4] = float((cv_img[row][col][0] << 24) | (cv_img[row][col][1] << 16) | (cv_img[row][col][0] << 8) | 0)
                # semantic color
                semantic_data[i][5] = float((max_semantic_color[row][col][0] << 24) | (max_semantic_color[row][col][1] << 16) | (max_semantic_color[row][col][0] << 8) | 0)
                # confidence
                semantic_data[i][6] = max_confidence[row * self.img_width + col]
                # 0
                semantic_data[i][7] = 0

        self.semantic_cloud.data = np.getbuffer(semantic_data.ravel())[:]
        self.semantic_cloud.header.stamp = stamp;
        return self.semantic_cloud

    def generate_cloud_semantic_bayesian(self, bgr_img, cloud_raw, bayes_semantic_colors, bayes_confidences, stamp):
        """
        """

    def pointcloud2_to_array(self, cloud_msg):
        ''' 
        Converts a rospy PointCloud2 message to a numpy recordarray 
        
        Assumes all fields 32 bit floats, and there is no padding.
        '''
        dtype_list = [(f.name, np.float32) for f in cloud_msg.fields]
        cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

    def get_xyz_points(self, cloud_array, remove_nans=True):
        '''
        Pulls out x, y, and z columns from the cloud recordarray, and returns a 3xN matrix.
        '''
        # remove crap points
        if remove_nans:
            mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
            cloud_array = cloud_array[mask]
        
        # pull out x, y, and z values
        points = np.zeros(list(cloud_array.shape) + [3], dtype=np.float)
        points[...,0] = cloud_array['x']
        points[...,1] = cloud_array['y']
        points[...,2] = cloud_array['z']

        return points

    def pointcloud2_to_xyz_array(self, cloud_msg, remove_nans=True):
        return self.get_xyz_points(self.pointcloud2_to_array(cloud_msg), remove_nans=remove_nans)

        

"""
from matplotlib import pyplot as plt
    img = cv2.imread('/home/dlonng/Pictures/house-cat.jpg')

    print(img[1][1][0])
    print(img[1][1][1])
    print(img[1][1][2])

    r = img[1][1][0]
    g = img[1][1][1]
    b = img[1][1][2]

    f4_rgb = (r << 24 | g << 16 | b << 8)
    print(f4_rgb)

    #semantic_data[i][4] = float((cv_img[row][col][0] << 24) | (cv_img[row][col][1] << 16) | (cv_img[row][col][0] << 8) | 0)

    
    plt.imshow(img)
    plt.show()
    
    print(img.shape)
"""