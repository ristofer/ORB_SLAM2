/**
* Author: Tim Resink
* Email: timresink@gmail.com


TODOs:
- Add settings to settings file
    -     cameraTopic
    -     tfTopic



*/

// STD
#include <iostream>
//#include <string>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <uchile_srvs/Onoff.h>
#include <std_msgs/Int8.h>

// TF
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// cv bridge
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h> 

//Eigen
#include <Eigen/Geometry>

// ORB-SLAM
#include "System.h"
#include "MapPoint.h"


class SubscribeHandler{
public:
	// constructors
    SubscribeHandler(const string &strVocFile,
                     const string &strSettingsFile, ros::NodeHandle *pNodeHandler,
                     tf::TransformListener *pTFlistener, tf::TransformBroadcaster *pTFbroadcaster);

    // transform world and camera with grabbed image
    tf::StampedTransform T_o_c;
    // transform between base and camera with image
    tf::StampedTransform T_b_c;
    cv::Mat cvT_o_c;
    cv_bridge::CvImageConstPtr cv_ptr;
    cv::Mat Twc;

    // methods
	void Shutdown();

private:
    // methods
    void GrabImage(const sensor_msgs::ImageConstPtr &msg);
    void Publish_Orientation(cv::Mat Tcw, tf::StampedTransform T_w_c);
    void Publish_Tracking_State(int state);
    void GetCurrentROSAllPointCloud(sensor_msgs::PointCloud2 &all_point_cloud, sensor_msgs::PointCloud2 &ref_point_cloud, cv::Mat matriz);
    void PointCloudPub(cv::Mat matriz);
    void InitPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    bool Active(uchile_srvs::Onoff::Request &req, uchile_srvs::Onoff::Response &res);

    cv::Mat tfToMat(const tf::StampedTransform& tfT);
    cv::Mat tfToMat(const tf::Transform& tfT);
    Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    std::vector<float> toQuaternion(const Eigen::Matrix<double, 3, 3> &M);
    std::vector<float> Normalize(std::vector<float> vect);
    g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    Eigen::Matrix4f toEigMat(const g2o::SE3Quat &SE3);

    // ROS
    ros::Subscriber subImage;
    ros::Subscriber m_initPoseSub_;
    ros::Publisher maqui_orientation;
    ros::Publisher tracking_state;

    ros::ServiceServer active_server;

    ros::Publisher AllPointCloud_pub_;
    ros::Publisher RefPointCloud_pub_; 

    ros::NodeHandle* mpNodeHandler;
    tf::TransformListener* mpTFlistener;
    tf::TransformBroadcaster* mpTFbroadcaster;

    // ORB SLAM pointer
    ORB_SLAM2::System* mpSLAM;

    //topics
    std::string cameraTopic;
    std::string tfTopic;
    std::string cameraFrameTopic;
    std::string odomFrameTopic;
    std::string broadCastTopic;
    std::string baseFrameTopic;
    std::string cameraFrameNameToPublish;
    std::string worldFrameNameToPublish;

    sensor_msgs::PointCloud2 allMapPoints;
    sensor_msgs::PointCloud2 referenceMapPoints;

    Eigen::Matrix4f offset_;

    // flags
    bool mbReferenceWorldFrame;
    bool _is_on;

    int useBaseFrame;
};
