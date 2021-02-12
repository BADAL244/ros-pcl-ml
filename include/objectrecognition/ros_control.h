#ifndef IMAGE_CONTROL_H
#define IMAGE_CONTROL_H

#include "ros/ros.h"

#include <iostream>
#include <unordered_set>
#include <pcl/common/common.h>
#include "ros_pointprocess.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "objectrecognition/BoundingBox3d.h"
#include "objectrecognition/BoundingBoxes3d.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/features/normal_3d.h>
#include "objectrecognition/GetNormals.h"
#include "objectrecognition/GetCloud.h"
#include "objectrecognition/vectorpointcloud.h"
using namespace std;

class Controller
{
    public:

        explicit Controller(ros::NodeHandle nh);

        void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg );

        //marker creation for visualization of bounding boxes in rviz 
        //@param BoundingBoxes3d

        void display_marker(objectrecognition::BoundingBoxes3d& boxes); 


        bool getNormalsReq(objectrecognition::GetNormals::Request &req, objectrecognition::GetNormals::Response &rsp);

        void callbackNorm(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    private:
        ros::NodeHandle m_nh;

        ros::Subscriber m_PontCloud_callback;
        float filterRes = 0.01;                                       
        Eigen::Vector4f minPoint = Eigen::Vector4f(0 , -.4 , -1, 1);

        Eigen::Vector4f maxPoint = Eigen::Vector4f(2, .4 , 1 ,  1);
        int maxIter = 500;
        float clusterTolerance = 1;
        int minClusterSize = 700;
        int maxClusterSize = 2000;
        ros::Publisher pub1;
        ros::Publisher pub2;
        ros::Publisher pub4;
        const std::string point_cloud = "/zed2/zed_node/point_cloud/cloud_registered";
        //Marker publisher MarkerArray
        ros::Publisher markers_pub_;
        ros::Publisher cloudvector;
        ros::ServiceServer get_normals_srv_;


};

#endif