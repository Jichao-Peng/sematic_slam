//
// Created by leo on 18-11-13.
//

#ifndef PROJECT_SEMATICSLAM_H
#define PROJECT_SEMATICSLAM_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <YOLO_V3/src/Detecting.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>




#include "ORB_SLAM2/include/System.h"

using namespace std;

class SematicSLAM
{
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
public:
    SematicSLAM(string image_topic, string depth_topic);

    void Run();
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

private:
    ros::NodeHandle node;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Synchronizer<sync_pol> synchronizer;
    ros::Publisher cloud_pub;

    ORB_SLAM2::System* mpSLAM;
};


#endif //PROJECT_SEMATICSLAM_H
