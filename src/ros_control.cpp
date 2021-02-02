#include "../include/objectrecognition/ros_control.h"

#include <stdint.h>

#include <stdlib.h>
#include <math.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "ros_pointcloud.cpp"

Controller::Controller(ros::NodeHandle nh):m_nh(nh){

    m_PontCloud_callback = m_nh.subscribe(point_cloud , 1 , &Controller::callback , this);


}
void Controller::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    //ros point cloud msgs to pcl point cloud conversion 
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    //pcl point cloud filter process using ProcessPointClouds class instance creation and calling their function for filetering 

    ProcessPointClouds<pcl::PointXYZRGB>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZRGB>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtercloud = pointProcessorI->FilterCloud(temp_cloud , filterRes , minPoint , maxPoint);
    ProcessPointClouds<pcl::PointXYZRGB>* pointProcessor = new ProcessPointClouds<pcl::PointXYZRGB>();

    //fileted point cloud data segmented using a plane equation for separate Plane and NOn Plane POint cloud planes 
    std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmentCloud2 = pointProcessor->SegmentPlane(filtercloud, 100, 0.01);

    //segmanted plane and non plane point cloud has been conveted to ROSMsg for visualization and subscription 

    auto cloud1 = pointProcessorI->conversion(segmentCloud2.first);
    auto cloud2 = pointProcessorI->conversion(segmentCloud2.second);

    //Non Plane ground removal point cloud has been clustered using Point processor clustering function
    //@param clusterTolerance 
    //@param minClusterSize
    //@param maxClusterSize
    
    int clusterId = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud2.second, clusterTolerance, minClusterSize, maxClusterSize);
    // Bounding Boxes around the cluster point cloud
    
    objectrecognition::BoundingBoxes3d boxes;


    for(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        
        Box box = pointProcessorI->BoundingBox(cluster);
        objectrecognition::BoundingBox3d bb_box; 

        bb_box.xmin = box.x_min;
        bb_box.xmax = box.x_max;
        bb_box.ymin = box.y_min;
        bb_box.ymax = box.y_max;
        bb_box.zmin = box.z_min;
        bb_box.zmax = box.z_max;
        boxes.bounding_boxes.push_back(bb_box);
    
        ++clusterId;
    }
}