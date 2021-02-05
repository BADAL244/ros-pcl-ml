#!/usr/bin/env python

import numpy as np 

import pickle
import rospy 
from imagecontrol.features import compute_color_histograms
from imagecontrol.features import compute_normal_histograms

from imagecontrol.pcl_helper import *

import pcl

from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, PointField
from imagecontrol.srv import GetNormals
from imagecontrol.srv import GetCloud




if __name__ == "__main__":
    rospy.init_node('clustering' , anonymous=True)


    

