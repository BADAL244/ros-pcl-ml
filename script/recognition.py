#!/usr/bin/env python

import numpy as np
import pcl
import pickle
import sklearn
from sklearn.preprocessing import LabelEncoder

from imagecontrol.srv import GetNormals
from imagecontrol.features import compute_color_histograms
from imagecontrol.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from imagecontrol.marker_tools import *
from imagecontrol.msg import DetectedObjectsArray
from imagecontrol.msg import DetectedObject
from imagecontrol.pcl_helper import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('get_normals',
                                          GetNormals)
    return get_normals_prox(cloud).cluster

def pcl_callback(pcl_msg):


    # Convert ROS msg to PCL data (XYZRGB)
    ros_cloud = ros_to_pcl(pcl_msg)

    print(ros_cloud)
        
 


    # Extract histogram features (similar to capture_features.py)
    histogram_bins = 64
    chists = compute_color_histograms(pcl_msg,
                                        nbins=histogram_bins,
                                        using_hsv=True)
    normals = get_normals(pcl_msg)
    nhists = compute_normal_histograms(normals,
                                        nbins=histogram_bins)
    feature = np.concatenate((chists, nhists))

    # Make the prediction, retrieve the label for the result and add it
    #   to detected_objects_labels list
    prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
    label = encoder.inverse_transform(prediction)[0]
    print(label)
   

    # Publish a label into RViz
    

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('recognition', anonymous=True)

    # Create Subscriber to receive the published data coming from the
    #   pcl_callback() function that will be processing the point clouds
    pcl_sub = rospy.Subscriber('/cluster', pc2.PointCloud2,
                               pcl_callback, queue_size=1)

    # Create Publishers
    object_markers_pub = rospy.Publisher('/object_markers', Marker,
                                         queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects',
                                           DetectedObjectsArray,
                                           queue_size=1)

    # Load model from disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
