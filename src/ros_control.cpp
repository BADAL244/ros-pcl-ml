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
    markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("Non_plane", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2> ("Plane_surface", 1);
    get_normals_srv_ = nh.advertiseService("get_normals", &Controller::getNormalsReq, this);
    cloudvector = nh.advertise<objectrecognition::vecpointcloud>("merged_clouds_topic", 1);

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
        auto cloud3 = pointProcessorI->conversion(cluster);
        objectrecognition::vectorpointcloud vector_cloud;
        bb_box.xmin = box.x_min;
        bb_box.xmax = box.x_max;
        bb_box.ymin = box.y_min;
        bb_box.ymax = box.y_max;
        bb_box.zmin = box.z_min;
        bb_box.zmax = box.z_max;
        boxes.bounding_boxes.push_back(bb_box);
        vector_cloud.pointclouds.push_back(cloud3);
        cloudvector.publish(vector_cloud);
        ++clusterId;
    }


    pub1.publish(cloud1);

    pub2.publish(cloud2);
}


//Marker creation function for Bounding Boxes visualization in rviz
void Controller::display_marker(objectrecognition::BoundingBoxes3d& boxes){
    visualization_msgs::MarkerArray msg;

  int counter_id = 0;
  for (auto bb : boxes.bounding_boxes) {
    visualization_msgs::Marker bbx_marker;

    bbx_marker.header.frame_id = "zed2_camera_center";
    bbx_marker.header.stamp = boxes.header.stamp;
    bbx_marker.ns = "box";
    bbx_marker.id = counter_id++;
    bbx_marker.type = visualization_msgs::Marker::CUBE;
    bbx_marker.action = visualization_msgs::Marker::ADD;
    bbx_marker.frame_locked = false;
    bbx_marker.pose.position.x = (bb.xmax + bb.xmin) / 2.0;
    bbx_marker.pose.position.y = (bb.ymax + bb.ymin) / 2.0;
    bbx_marker.pose.position.z = (bb.zmax + bb.zmin) / 2.0;
    bbx_marker.pose.orientation.x = 0.0;
    bbx_marker.pose.orientation.y = 0.0;
    bbx_marker.pose.orientation.z = 0.0;
    bbx_marker.pose.orientation.w = 1.0;
    bbx_marker.scale.x = (bb.xmax - bb.xmin);
    bbx_marker.scale.y = (bb.ymax - bb.ymin);
    bbx_marker.scale.z = (bb.zmax - bb.zmin);
    bbx_marker.color.b = 0;
    bbx_marker.color.g = bb.probability * 255.0;
    bbx_marker.color.r = (1.0 - bb.probability) * 255.0;
    bbx_marker.color.a = 0.4;
    
    bbx_marker.text = bb.object_name;

    msg.markers.push_back(bbx_marker);
  }

 markers_pub_.publish(msg);
}


bool Controller::getNormalsReq(objectrecognition::GetNormals::Request &req, objectrecognition::GetNormals::Response &rsp)
  {
    rsp.cluster = req.cluster;

    pcl::PointCloud<pcl::PointXYZ> *p_cloud = new pcl::PointCloud<pcl::PointXYZ>();
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > sp_pcl_cloud(p_cloud);
    pcl::fromROSMsg(req.cluster, *p_cloud);

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (sp_pcl_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.01);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Compute the features
    ne.compute(*cloud_normals);

    pcl::toROSMsg(*cloud_normals, rsp.cluster);


    return true;
  }