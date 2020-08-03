from __future__ import division
from __future__ import print_function

import rospy
from enum import Enum


class PointType(Enum):
    COLOR = 0
    SEMANTIC_MAX = 1
    SEMANTIC_BAYESIAN = 2


class SemanticCloudGenerator:
    """
    """

    def __init__(self):
        """
        """


    def generate_cloud_semantic_max(self, bgr_img, depth_img, semantic_color, confidence, stamp):
        """
        """


    def generate_cloud_semantic_bayesian(self, bgr_img, depth_img, semantic_color, confidence, stamp):
        """
        """