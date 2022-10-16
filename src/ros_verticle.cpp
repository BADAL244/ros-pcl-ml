#include "../include/objectrecognition/ros_control.h"

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

typedef  pcl::PointXYZ PointType;

KdTree* tree;
std::ofstream fw;
const std::string filename{"/home/wasp/catkin_ws/src/ros-pcl-ml/text_file/data_collected.txt"};


Controller::Controller(ros::NodeHandle nh):m_nh(nh),pr_nh("~"){

    m_PontCloud_callback = m_nh.subscribe(point_cloud , 1 , &Controller::callback , this);
    marker_callback_sub = m_nh.subscribe(marker_topic , 1 , &Controller::markers_callback , this);
    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_topic", 1);
    
    server_.reset(new ReconfigureServer(dr_mutex_));  
    dynamic_reconfigure::Server<objectrecognition::TuningConfig>::CallbackType cbt =
            boost::bind(&Controller::drCallback, this, boost::placeholders::_1, boost::placeholders::_2);
    server_->setCallback(cbt);
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
    pr_nh.getParam("thereshold", _thereshold);
    pr_nh.getParam("radius", _radius);

    pr_nh.getParam("ID1", _id_one);
    pr_nh.getParam("ID2", _id_two);



    fw.open (filename);



}
void Controller::callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg){
    received_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*pointcloud_msg.get(), *received_cloud_ptr.get());
    cropped_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    if(new_set.size() >=2 ){

        for(auto ele:new_set){
            if(ele.new_id.id == _id_one){
                cout << ele.Pt.x  << "id"<< _id_one <<  "content" << endl;
                x_min=  ele.Pt.x * config_.scale_x;
                x_max = x_min + 1.6;
                y_min = ele.Pt.y;
                z_min = ele.Pt.z + config_.scale_z_min;
                z_max = ele.Pt.z + config_.scale_z_max;
            }else if(ele.new_id.id == _id_two){
                cout << ele.Pt.y << "id"<< _id_two <<  "content" << endl;
                y_max = ele.Pt.y;
            }
        }
    

    }

    if(config_.enable){
    cout << "have you been here" << endl;
    if(_xmin <= _xmax && _ymin<=_ymax && _zmin <= _zmax){
    new_cropping_cloud(received_cloud_ptr, cropped_cloud_ptr, _xmin , _xmax , _ymin , _ymax , _zmin , _zmax);
    }

    }else{
    new_cropping_cloud(received_cloud_ptr, cropped_cloud_ptr, x_min , x_max , y_min , y_max , z_min , z_max);
    }


    pcl::PointXYZ cc_min, cc_max;
    if(cropped_cloud_ptr->points.size() > 0){
    pcl::getMinMax3D(*cropped_cloud_ptr, cc_min, cc_max);
    if(cc_min.x <= cc_max.x && cc_min.y<=cc_max.y && cc_min.z <= cc_max.z){
            pcl::toROSMsg(*cropped_cloud_ptr.get(),cropped_cloud_msg );
            cropped_cloud_msg.header.stamp = pointcloud_msg->header.stamp;
            cropped_cloud_msg.header.frame_id = pointcloud_msg->header.frame_id;
            pub1.publish(cropped_cloud_msg);
        }
    }
    if(config_.enable){
        cout << "yes bro" << endl;
    }

    project_to_costmap(cropped_cloud_ptr);


}


void Controller::markers_callback(const ar_track_alvar_msgs::AlvarMarkers& marker_msgs){
        new_markers = marker_msgs; 
        // cout << new_marker.id << endl;

        for (auto new_marker : new_markers.markers) {
            cout << new_marker.id << endl;
            if(new_marker.id == _id_one){
                pcl::PointXYZ new_point;
                new_point.x = new_marker.pose.pose.position.x;
                new_point.y = new_marker.pose.pose.position.y;
                new_point.z = new_marker.pose.pose.position.z;
                container.Pt = new_point;
                container.new_id.id = new_marker.id;
                new_set.insert(container);

            }else if(new_marker.id == _id_two ){
                pcl::PointXYZ  new_point;
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

     
    for (auto& point : *input_cloud_ptr)
    {
      if (point.x > bb_min.x and point.x < bb_max.x)
        point.x = 0;
      else
        cout << "have you reached here3" <<endl; 
        continue;
    }


    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = "zed_camera_center";
    if(config_.map_enable){
        grid.info.resolution = config_.resolution;
    }else{
        grid.info.resolution = _resolution;
    }
    



    if (bb_min.y < bb_max.z){
      grid.info.height = ((int)((bb_max.y - bb_min.y) / grid.info.resolution)) + 1;
      cout << "have you reached here5" <<endl; 
      cout << grid.info.height << "width" << endl;
    }
    else  
      grid.info.height = 1;


    if (bb_min.z < bb_max.z){
      grid.info.width = ((int)((bb_max.z - bb_min.z) / grid.info.resolution)) + 1;
      cout << grid.info.width << "height" <<endl;
      cout << "have you reached here6" <<endl; }
    else  
      grid.info.width = 1;
    int size_data = input_cloud_ptr->points.size();


    std::vector<std::vector<float>> total_data(size_data);
    for (int i = 0 ; i < size_data ; i++) {
        total_data[i].resize(2, 0.0);
    }
 
    total_data = search_cloud(input_cloud_ptr);
    KdTree* tree = new KdTree;
    for (int i=0; i<total_data.size(); i++)
    tree->insert(total_data[i],i);




    grid.info.origin.position.y = bb_min.y;
    grid.info.origin.position.x = bb_min.x;
    grid.info.origin.position.z = bb_min.z;

    if(config_.quaternion_enable){
    grid.info.origin.orientation.w = config_.orientation_w_w;
    grid.info.origin.orientation.x = config_.orientation_x_x;
    grid.info.origin.orientation.y = config_.orientation_y_y;
    grid.info.origin.orientation.z = config_.orientation_z_z; 
    }else{
    grid.info.origin.orientation.w = _orientation_w;
    grid.info.origin.orientation.x = _orientation_x;
    grid.info.origin.orientation.y = _orientation_y;
    grid.info.origin.orientation.z = _orientation_z;
       
    }

    size_t grid_size = grid.info.width * grid.info.height;
    grid.data = std::vector<int8_t>(grid_size, 0);


    for (int index_y = 0; index_y < grid.info.height; index_y++)
    {
      for (int index_x = 0; index_x < grid.info.width; index_x++)
      {
        unsigned int i = index_x + (grid.info.height - index_y - 1) * grid.info.width;
        double y = bb_min.y + index_y * grid.info.resolution;
        double z = bb_min.z + index_x * grid.info.resolution;

        if(config_.map_enable){
            nearby = tree->search({y,z}, config_.radius);
        }else{
            nearby = tree->search({y,z}, _radius);
        }
        
        if (nearby.size() > _thereshold)
          grid.data[i] = 0;
        else
          grid.data[i] = 100;
      }
    }

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



    for(size_t i=0;i<input_cloud_ptr->points.size();i++){
        //point.x = input_cloud_ptr.points[i].x;
        points[i][0] = input_cloud_ptr->points[i].y;
        points[i][1] = input_cloud_ptr->points[i].z;

    }
    return points;
}

template
<typename PointT>
void Controller::new_cropping_cloud(boost::shared_ptr<pcl::PointCloud<PointT> > input_cloud_ptr, boost::shared_ptr<pcl::PointCloud<PointT> > crop_box_cloud_ptr,
double x_min, double x_max,double  y_min,double  y_max,double  z_min,double z_max)
{

    Eigen::Vector4f minPoint = Eigen::Vector4f(x_min, y_min , z_min, 1);
    Eigen::Vector4f maxPoint = Eigen::Vector4f(x_max, y_max , z_max, 1);

    pcl::CropBox<PointT> region(true);

    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(input_cloud_ptr);
    region.filter(*crop_box_cloud_ptr.get());

}


void Controller::drCallback(objectrecognition::TuningConfig& config,
        uint32_t level)
{
        config_ = config;
        std::cout << config_.scale_x << std::endl;
        std::cout << config_.orientation_x_x << std::endl;
        std::cout << _orientation_x << std::endl;
}
