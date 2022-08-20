#include "../include/objectrecognition/ros_control.h"

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "ros_pointcloud.cpp"
typedef  pcl::PointXYZ PointType;

KdTree* tree;
std::ofstream fw;
const std::string filename{"/home/wasp/catkin_ws/src/ros-pcl-ml/text_file/data_collected.txt"};


Controller::Controller(ros::NodeHandle nh):m_nh(nh),pr_nh("~"){

    m_PontCloud_callback = m_nh.subscribe(point_cloud , 1 , &Controller::callback , this);
    marker_callback_sub = m_nh.subscribe(marker_topic , 1 , &Controller::markers_callback , this);
    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_topic", 1);
    


    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("Non_plane", 1);
    pr_nh.getParam("xmin", _xmin);
    pr_nh.getParam("xmax", _xmax);
    pr_nh.getParam("ymin", _ymin);
    pr_nh.getParam("ymax", _ymax);
    pr_nh.getParam("zmin", _zmin);
    pr_nh.getParam("zmax", _zmax);
    
    pr_nh.getParam("orientation_x", _orientation_x);
    pr_nh.getParam("orientation_y", _orientation_y);
    pr_nh.getParam("orientation_z", _orientation_z);
    pr_nh.getParam("orientation_w", _orientation_w);

    pr_nh.getParam("resolution", _resolution);
    pr_nh.getParam("height", _height);
    pr_nh.getParam("width", _width);

    fw.open (filename);



}
void Controller::callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg){
    received_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*pointcloud_msg.get(), *received_cloud_ptr.get());
    cropped_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    if(new_set.size() >=6 ){

        for(auto ele:new_set){
            if(ele.new_id.id == 3){
                cout << ele.Pt.x << "id 3 content" << endl;
                x_max=  ele.Pt.x+.3;
                x_min = ele.Pt.x-.3;
                y_min = ele.Pt.y;
            }else if(ele.new_id.id == 7){
                cout << ele.Pt.y << "id 7 content" << endl;
                y_max = ele.Pt.y;
            }else if(ele.new_id.id == 4){
                cout << ele.Pt.z << "id 4 content" << endl;

                z_min = ele.Pt.z;
            }else if(ele.new_id.id == 9){
                cout << ele.Pt.z << "id 9 content" << endl;

                z_max = ele.Pt.z; 
            }
        }
    

    }

    
    //croppingCloud(received_cloud_ptr, cropped_cloud_ptr,0.0 , 2.0 , -.5 , .50 , -.50 ,.5 );

    croppingCloud(received_cloud_ptr, cropped_cloud_ptr,0.0 , x_max , y_min , y_max , z_min , z_max);
    pcl::toROSMsg(*cropped_cloud_ptr.get(),cropped_cloud_msg );


    cropped_cloud_msg.header.stamp = pointcloud_msg->header.stamp;
    cropped_cloud_msg.header.frame_id = pointcloud_msg->header.frame_id;
    pub1.publish(cropped_cloud_msg);

    project_to_costmap(cropped_cloud_ptr);


}


void Controller::markers_callback(const ar_track_alvar_msgs::AlvarMarkers& marker_msgs){
        new_markers = marker_msgs; 
        // cout << new_marker.id << endl;

        for (auto new_marker : new_markers.markers) {
            cout << new_marker.id << endl;
            if(new_marker.id == 0 || new_marker.id == 3){
                pcl::PointXYZ new_point;
                new_point.x = new_marker.pose.pose.position.x;
                new_point.y = new_marker.pose.pose.position.y;
                new_point.z = new_marker.pose.pose.position.z;
                container.Pt = new_point;
                container.new_id.id = new_marker.id;
                new_set.insert(container);

            }else if(new_marker.id == 4 || new_marker.id == 7){
                pcl::PointXYZ  new_point;
                new_point.x = new_marker.pose.pose.position.x;
                new_point.y = new_marker.pose.pose.position.y;
                new_point.z = new_marker.pose.pose.position.z;
                container.Pt = new_point;
                container.new_id.id= new_marker.id;
                new_set.insert(container);
                
            }else if(new_marker.id == 12 || new_marker.id == 9){
                pcl::PointXYZ  new_point;
                new_point.x = new_marker.pose.pose.position.x;
                new_point.y = new_marker.pose.pose.position.y;
                new_point.z = new_marker.pose.pose.position.z;
                container.Pt = new_point;
                container.new_id.id = new_marker.id;
                new_set.insert(container);
                
            }else if(new_marker.id == 5 || new_marker.id == 8){
                pcl::PointXYZ new_point;
                new_point.x = new_marker.pose.pose.position.x;
                new_point.y = new_marker.pose.pose.position.y;
                new_point.z = new_marker.pose.pose.position.z;
                container.Pt = new_point;
                container.new_id.id= new_marker.id;
                new_set.insert(container);

            }

        }  

}


template
<typename PointT>
void Controller::croppingCloud(boost::shared_ptr<pcl::PointCloud<PointT> > input_cloud_ptr, boost::shared_ptr<pcl::PointCloud<PointT> > output_cloud_ptr,
double x_min, double x_max,double  y_min,double  y_max,double  z_min,double z_max)
{
    boost::shared_ptr<pcl::PointCloud<PointT> > boundingbox_ptr (new pcl::PointCloud<PointT> );



    PointT point;

     //x_min,y_min,z_min
    point.x = x_min;
    point.y = y_min;
    point.z = z_min;

    boundingbox_ptr->push_back(point);


    //x_min,y_min,z_max
    point.x = x_min;
    point.y = y_min;
    point.z = z_max;

    boundingbox_ptr->push_back(point);

    //x_min,y_max,z_min
    point.x = x_min;
    point.y = y_max;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_min,y_max,z_max
    point.x = x_min;
    point.y = y_min;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_max,y_min,z_min
    point.x = x_max;
    point.y = y_min;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_max,y_min,z_max
    point.x = x_max;
    point.y = y_min;
    point.z = z_max;

    boundingbox_ptr->push_back(point);

    //x_max,y_max,z_min
    point.x = x_max;
    point.y = y_max;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_max,y_max,z_max
    point.x = x_max;
    point.y = y_max;
    point.z = z_max;

    boundingbox_ptr->push_back(point);


    pcl::ConvexHull<PointT> hull;
    std::vector<pcl::Vertices> polygons;

    boost::shared_ptr<pcl::PointCloud<PointT> > surface_hull;

    try
    {
        hull.setInputCloud(boundingbox_ptr);
        hull.setDimension(3);

        surface_hull.reset(new pcl::PointCloud<PointT>);
        hull.reconstruct(*surface_hull, polygons);

    }catch( std::exception e)
    {
        return;
    }



    try
    {
        pcl::CropHull<PointT> bb_filter;

        bb_filter.setDim(3);
        bb_filter.setNegative(false);
        bb_filter.setInputCloud(input_cloud_ptr);
        bb_filter.setHullIndices(polygons);
        bb_filter.setHullCloud(surface_hull);
        bb_filter.filter(*output_cloud_ptr.get());

    }
    catch( std::exception e )
    {
        return;
    }


}

template
<typename PointT>
bool Controller::project_to_costmap(boost::shared_ptr<pcl::PointCloud<PointT> > input_cloud_ptr){
    pcl::PointXYZ bb_min, bb_max;
    pcl::getMinMax3D(*input_cloud_ptr, bb_min, bb_max);
    cout << "have you reached here0" <<endl; 
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(input_cloud_ptr);
    cout << "have you reached here1" <<endl; 
     

    std::vector<int> indices;
    std::vector<float> distances;

    int K = 10;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    for (auto& point : *input_cloud_ptr)
    {
        cout << "have you reached here2" <<endl; 
      // remove points that are outside the boundind box, by setting their z to a huge value
      if (point.x > bb_min.x and point.x < bb_max.x)
        point.x = 0;
      else
        cout << "have you reached here3" <<endl; 
        continue;
    }
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = "zed_camera_center";
    grid.info.resolution = resolution;
    cout << "have you reached here4" <<endl; 
    if (bb_min.y < bb_max.z){
      grid.info.height = ((int)((bb_max.y - bb_min.y) / resolution)) + 1;
      cout << "have you reached here5" <<endl; 
      cout << grid.info.height << "width" << endl;
    }
    else  
      grid.info.height = 1;
    if (bb_min.z < bb_max.z){
      grid.info.width = ((int)((bb_max.z - bb_min.z) / resolution)) + 1;
      cout << grid.info.width << "height" <<endl;
      cout << "have you reached here6" <<endl; }
    else  
      grid.info.width = 1;
    int size_data = input_cloud_ptr->points.size();
    std::vector<std::vector<float>> total_data(size_data);
    for (int i = 0 ; i < size_data ; i++) {
        total_data[i].resize(2, 0.0);
    }
    //total_data(input_cloud_ptr->points.size() , std::vector<float>(2, 0));
    total_data = search_cloud(input_cloud_ptr);
    KdTree* tree = new KdTree;
    for (int i=0; i<total_data.size(); i++)
    tree->insert(total_data[i],i);




    grid.info.origin.position.y = bb_min.y;
    grid.info.origin.position.x = bb_min.x;
    grid.info.origin.position.z = bb_min.z;
    grid.info.origin.orientation.w = -0.7071068;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.7071068;
    grid.info.origin.orientation.z = 0.0;
    size_t grid_size = grid.info.width * grid.info.height;
    grid.data = std::vector<int8_t>(grid_size, 0);
    int maxn = 0;
    int index = 0;

    for (int index_y = 0; index_y < grid.info.height; index_y++)
    {
      for (int index_x = 0; index_x < grid.info.width; index_x++)
      {
        unsigned int i = index_x + (grid.info.height - index_y - 1) * grid.info.width;
        double y = bb_min.y + index_y * resolution;
        double z = bb_min.z + index_x * resolution;
        PointType point(0, y, z);
        //int neighbours =  kdtree.radiusSearch(point, resolution, indices, distances);
        cout << "have you reached here7" <<endl; 
        
        std::vector<int> nearby = tree->search({y,z},.1);
        cout << y << ",,,,," << z << ",,,,," << nearby.size() << ",,,,," ;
        if(fw.is_open() && allowed){
            fw << y << "," << z << "," << nearby.size() << std::endl;
        }

        if (nearby.size() > 1000)
          grid.data[i] = 0;
        else
          grid.data[i] = 100;
        //index++;

      }
    }
fw.close();
allowed = false;
cout << "have you reached here12" <<endl; 
grid_pub.publish(grid);

return true;
}


template
<typename PointT>
std::vector<std::vector<float>> Controller::search_cloud(boost::shared_ptr<pcl::PointCloud<PointT> > input_cloud_ptr){

   
    int size_data = input_cloud_ptr->points.size();
    std::vector<std::vector<float>> points(size_data);
    for (int i = 0 ; i < size_data ; i++) {
        points[i].resize(2, 0.0);
    }

    cout << "is it came here ??" << endl; 

    for(size_t i=0;i<input_cloud_ptr->points.size();i++){
        //point.x = input_cloud_ptr.points[i].x;
        points[i][0] = input_cloud_ptr->points[i].y;
        points[i][1] = input_cloud_ptr->points[i].z;

    }
    return points;
}

