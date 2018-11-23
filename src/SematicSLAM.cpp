//
// Created by leo on 18-11-13.
//

#include "SematicSLAM.h"


SematicSLAM::SematicSLAM(string image_topic, string depth_topic):
        image_sub(node, "/camera/rgb/image_raw", 1),
        depth_sub(node, "/camera/depth_registered/sw_registered/image_rect_raw", 1),
        synchronizer(sync_pol(10),image_sub,depth_sub)
{
    string argv1 = "/home/leo/Desktop/sematic_slam_project/src/sematic_slam/ORB_SLAM2/Vocabulary/ORBvoc.txt";
    string argv2 = "/home/leo/Desktop/sematic_slam_project/src/sematic_slam/ORB_SLAM2/config/ZR300.yaml";
    mpSLAM = new ORB_SLAM2::System(argv1,argv2,ORB_SLAM2::System::RGBD,false);//最后一位代表是否开启画图

    cloud_pub = node.advertise<sensor_msgs::PointCloud2>("global_map",1);
    synchronizer.registerCallback(boost::bind(&SematicSLAM::GrabRGBD,this,_1,_2));
}

void SematicSLAM::Run()
{
    while (ros::ok())
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGBA>);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_copy(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*global_map, *global_map_copy);

        sensor_msgs::PointCloud2 global_map_output;
        pcl::toROSMsg(*global_map_copy, global_map_output);
        global_map_output.header.frame_id = "camera_depth_frame";
        global_map_output.header.stamp = ros::Time::now();
        cloud_pub.publish(global_map_output);

        ros::Rate rate(100);
        rate.sleep();
        ros::spinOnce();
    }

    // Stop all threads
    mpSLAM->Shutdown();
    // Save camera trajectory
    mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void SematicSLAM::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_orb_slam2");
    ros::start();

    SematicSLAM SematicSLAMSystem("/camera/rgb/image_raw","/camera/depth_registered/sw_registered/image_rect_raw");
    SematicSLAMSystem.Run();

    cv::waitKey(0);

    return 0;
}