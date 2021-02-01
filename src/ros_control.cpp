#include "../include/objectrecognition/ros_control.h"

#include <stdint.h>

#include <stdlib.h>
#include <math.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>


Controller::Controller(ros::NodeHandle nh):m_nh(nh){

    m_PontCloud_callback = m_nh.subscribe(point_cloud , 1 , &Controller::callback , this);


}
void Controller::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

}