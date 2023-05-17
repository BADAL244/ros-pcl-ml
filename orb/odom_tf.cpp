/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    explicit ImageGrabber(ORB_SLAM3::System* pSLAM, ros::NodeHandle nh_):mpSLAM(pSLAM),nh1(nh_){
        // cv::Mat Twc = Tcw.rowRange(0,3).col(3); // Translation
        // cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3); // Rotation
        // cv::Mat R = (cv::Mat_ <float>(3,3) << 1., 0., 0., 0., 0., -1., 0., 1., 0.);

        // Rwc = Rwc * R;
        // // Twc = Twc * R;

        // vector<float> q = ORB_SLAM3::Converter::toQuaternion(/*Tcw.rowRange(0,3).colRange(0,3)*/Rwc);

        // tf_cam.setOrigin(tf::Vector3(0, 0, 0));
        // tf_cam.setRotation(tf::Quaternion(1, 0, 0, 0));

        // tfBc.sendTransform(tf::StampedTransform(tf_cam, ros::Time::now(), "/odom","/base_link"));
        
        
        odom_pub = nh1.advertise<nav_msgs::Odometry>("/odom", 1);

        
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    tf::Quaternion hamiltonProduct(tf::Quaternion a, tf::Quaternion b);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    ros::Publisher odom_pub ;
    ros::NodeHandle nh1;

    tf::TransformBroadcaster tfBc;
    tf::Transform tf_cam;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true, 0, "LOADED SEQUENCE", file_name+".osa");
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);
    ros::NodeHandle nh;
    ImageGrabber igb(&SLAM ,nh);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    
    
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    cv::Mat pose = cv::Mat::eye(4,4,CV_32F);
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        pose = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        pose = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    if (!pose.empty() && nh1.ok())
    {
        tf::Matrix3x3 tf3d;
        tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
                pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
                pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));

        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);
        double aux = tfqt[0];
            tfqt[0]=-tfqt[2];
            tfqt[2]=tfqt[1];
            tfqt[1]=aux;



        //Translation for camera
        tf::Vector3 origin;
        origin.setValue(pose.at<float>(0,3),pose.at<float>(1,3),pose.at<float>(2,3));
        //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
        const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                                0, 0, 1,
                                                -1, 0, 0);

        tf::Vector3 translationForCamera = origin * rotation270degXZ;

        //Hamilton (Translation for world)
        tf::Quaternion quaternionForHamilton(tfqt[3], tfqt[0], tfqt[1], tfqt[2]);
        tf::Quaternion secondQuaternionForHamilton(tfqt[3], -tfqt[0], -tfqt[1], -tfqt[2]);
        tf::Quaternion translationHamilton(0, translationForCamera[0], translationForCamera[1], translationForCamera[2]);

        tf::Quaternion translationStepQuat;
        translationStepQuat = hamiltonProduct(hamiltonProduct(quaternionForHamilton, translationHamilton), secondQuaternionForHamilton);

        tf::Vector3 translation(translationStepQuat[1], translationStepQuat[2], translationStepQuat[3]);

        //Scaling
        // if(m_numScales > 0) {
        //     translation = m_scale * translation;
        // }
        //Set world
        // m_currentQ = tfqt;
        // m_currentT = translation;
        // translation = translation - m_worldT;
        // tfqt = tfqt * m_worldQ.inverse();

        //Creates transform and populates it with translation and quaternion
        tf::Transform transformCurrent;
        transformCurrent.setOrigin(translation);
        transformCurrent.setRotation(tfqt);
        //Publishes transform
        static tf::TransformBroadcaster br;
        br.sendTransform(tf::StampedTransform(transformCurrent, ros::Time::now(), "/odom", "/base_link"));

    } 

}

tf::Quaternion ImageGrabber::hamiltonProduct(tf::Quaternion a, tf::Quaternion b) {

	tf::Quaternion c;

		c[0] = (a[0]*b[0]) - (a[1]*b[1]) - (a[2]*b[2]) - (a[3]*b[3]);
		c[1] = (a[0]*b[1]) + (a[1]*b[0]) + (a[2]*b[3]) - (a[3]*b[2]);
		c[2] = (a[0]*b[2]) - (a[1]*b[3]) + (a[2]*b[0]) + (a[3]*b[1]);
		c[3] = (a[0]*b[3]) + (a[1]*b[2]) - (a[2]*b[1]) + (a[3]*b[0]);

	return c;
}


