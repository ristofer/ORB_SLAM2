/**
* This file is part of the odometry integration of pepper in SLAM
* Author: Tim Resink
* Email: timresink@gmail.com
*/

#include"SubscribeHandler.hpp"
using namespace Eigen;



SubscribeHandler::SubscribeHandler(const string &strVocFile,
                                   const string &strSettingsFile, ros::NodeHandle* pNodeHandler,
                                   tf::TransformListener* pTFlistener, tf::TransformBroadcaster* pTFbroadcaster):
mpNodeHandler(pNodeHandler),
mpTFlistener(pTFlistener),
mpTFbroadcaster(pTFbroadcaster),
mbReferenceWorldFrame(false)
{
      cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
      cameraTopic = (std::string) fsSettings["Topic.Camera"];
      cameraFrameTopic = (std::string) fsSettings["Topic.CameraFrame"];
      odomFrameTopic = (std::string) fsSettings["Topic.OdomFrame"];
      tfTopic =(std::string) fsSettings["Topic.TF"];
      int queueSize = (int) fsSettings["Topic.QueueSize"];
      baseFrameTopic = (std::string) fsSettings["Topic.BaseFrame"];
      useBaseFrame = (int) fsSettings["Initializer.baseFrame"];
      cameraFrameNameToPublish = (std::string) fsSettings["Topic.CameraFrameNameToPublish"];
      worldFrameNameToPublish = (std::string) fsSettings["Topic.WorldFrameNameToPublish"];

      _is_on = false;

      mpSLAM = new ORB_SLAM2::System(strVocFile, strSettingsFile, ORB_SLAM2::System::MONOCULAR,true,true);

      if(useBaseFrame)
          broadCastTopic = baseFrameTopic + "_ORB";
      else
          broadCastTopic = cameraFrameNameToPublish;

      subImage = mpNodeHandler->subscribe(cameraTopic, 1, &SubscribeHandler::GrabImage, this);
      maqui_orientation = mpNodeHandler->advertise<geometry_msgs::PoseStamped>("/maqui/odom_ORB", queueSize);
      tracking_state = mpNodeHandler->advertise<std_msgs::Int8>("/orb_slam_tracking_state", queueSize);


      AllPointCloud_pub_ = mpNodeHandler->advertise<sensor_msgs::PointCloud2>("/orb_slam/point_cloud_all", queueSize);
      RefPointCloud_pub_ = mpNodeHandler->advertise<sensor_msgs::PointCloud2>("/orb_slam/point_cloud_ref", queueSize);

      m_initPoseSub_ = mpNodeHandler->subscribe("/maqui/nav/initialpose",1,&SubscribeHandler::InitPoseReceived,this);

      active_server = mpNodeHandler->advertiseService("/maqui/orb/active", &SubscribeHandler::Active,this);

      offset_.setIdentity();


      //PointCloudPub();

      // Initialize ORB system
   // argument 4 boolean is user viewer
}



void SubscribeHandler::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    try
    {
        mpTFlistener->waitForTransform(odomFrameTopic, cameraFrameTopic, ros::Time(0), ros::Duration(0.0001));
        mpTFlistener->lookupTransform(odomFrameTopic, cameraFrameTopic,ros::Time(0), T_o_c);
    }
    catch(tf::TransformException& e)
    {
        ROS_WARN("TF exception while grabbing camera transform \n %s", e.what());
    }

    cvT_o_c = tfToMat(T_o_c);
    mpSLAM->SetOdomPose(cvT_o_c);
    Twc = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

    if(!Twc.empty())
    {
        SubscribeHandler::Publish_Orientation(Twc.clone(), T_o_c);
        PointCloudPub(Twc.clone());
    }

    int TrackingState = mpSLAM->GetTrackingState();
    SubscribeHandler::Publish_Tracking_State(TrackingState);

}



void SubscribeHandler::Publish_Orientation(cv::Mat Tcw, tf::StampedTransform T_o_c)
{


    Eigen::Matrix<double, 3, 3> Tcw_eig = SubscribeHandler::toMatrix3d(Tcw.clone());
    std::vector<float> q = SubscribeHandler::toQuaternion(Tcw_eig);

    // TF fill broadcast message
    tf::Transform TForientation;

    TForientation.setOrigin(tf::Vector3(Tcw.at<float>(0,3), Tcw.at<float>(1,3),Tcw.at<float>(2,3)));

    tf::Quaternion quatTF;
    quatTF.setX(q[0]);
    quatTF.setY(q[1]);
    quatTF.setZ(q[2]);
    quatTF.setW(q[3]);
    TForientation.setRotation(quatTF);

    // publish geometry message
    geometry_msgs::PoseStamped orientation_msg;

    orientation_msg.header.frame_id = "CameraTop_optical_frame";
    orientation_msg.header.stamp.sec = T_o_c.stamp_.sec;
    orientation_msg.pose.position.x = Tcw.at<float>(0,3);
    orientation_msg.pose.position.y = Tcw.at<float>(1,3);
    orientation_msg.pose.position.z = Tcw.at<float>(2,3);
    orientation_msg.pose.orientation.x = q[0];
    orientation_msg.pose.orientation.y = q[1];
    orientation_msg.pose.orientation.z = q[2];
    orientation_msg.pose.orientation.w = q[3];



    mpTFbroadcaster->sendTransform(tf::StampedTransform(TForientation, T_o_c.stamp_, worldFrameNameToPublish
            , broadCastTopic));

    maqui_orientation.publish(orientation_msg);

}

void SubscribeHandler::Publish_Tracking_State(int state)
{
    std_msgs::Int8 StateMsg;
    StateMsg.data = state;
    tracking_state.publish(StateMsg);
    return;
}
cv::Mat SubscribeHandler::tfToMat(const tf::StampedTransform& tfT)
{
    cv::Mat cvT = cv::Mat::eye(4, 4, CV_32F);

    cvT.at<float>(0,3) = tfT.getOrigin().x();
    cvT.at<float>(1,3) = tfT.getOrigin().y();
    cvT.at<float>(2,3) = tfT.getOrigin().z();
    cvT.at<float>(3,3) = 1.0;

    cvT.at<float>(0,0) = tfT.getBasis().getColumn(0).x();
    cvT.at<float>(1,0) = tfT.getBasis().getColumn(0).y();
    cvT.at<float>(2,0) = tfT.getBasis().getColumn(0).z();
    cvT.at<float>(0,1) = tfT.getBasis().getColumn(1).x();
    cvT.at<float>(1,1) = tfT.getBasis().getColumn(1).y();
    cvT.at<float>(2,1) = tfT.getBasis().getColumn(1).z();
    cvT.at<float>(0,2) = tfT.getBasis().getColumn(2).x();
    cvT.at<float>(1,2) = tfT.getBasis().getColumn(2).y();
    cvT.at<float>(2,2) = tfT.getBasis().getColumn(2).z();

    return cvT;
}

cv::Mat SubscribeHandler::tfToMat(const tf::Transform& tfT)
{
    cv::Mat cvT = cv::Mat::eye(4, 4, CV_32F);

    cvT.at<float>(0,3) = tfT.getOrigin().x();
    cvT.at<float>(1,3) = tfT.getOrigin().y();
    cvT.at<float>(2,3) = tfT.getOrigin().z();
    cvT.at<float>(3,3) = 1.0;

    cvT.at<float>(0,0) = tfT.getBasis().getColumn(0).x();
    cvT.at<float>(1,0) = tfT.getBasis().getColumn(0).y();
    cvT.at<float>(2,0) = tfT.getBasis().getColumn(0).z();
    cvT.at<float>(0,1) = tfT.getBasis().getColumn(1).x();
    cvT.at<float>(1,1) = tfT.getBasis().getColumn(1).y();
    cvT.at<float>(2,1) = tfT.getBasis().getColumn(1).z();
    cvT.at<float>(0,2) = tfT.getBasis().getColumn(2).x();
    cvT.at<float>(1,2) = tfT.getBasis().getColumn(2).y();
    cvT.at<float>(2,2) = tfT.getBasis().getColumn(2).z();

    return cvT;
}

g2o::SE3Quat SubscribeHandler::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

cv::Mat SubscribeHandler::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=eigMat(i,j);

    return cvMat.clone();
}
Eigen::Matrix4f SubscribeHandler::toEigMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    Eigen::Matrix4f matrix_eig;
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            matrix_eig(i,j)=eigMat(i,j);

    return matrix_eig;
}

Eigen::Matrix<double,3,3> SubscribeHandler::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;
    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

std::vector<float> SubscribeHandler::toQuaternion(const Eigen::Matrix<double, 3, 3> &M)
{
    Eigen::Quaterniond q(M);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    std::vector<float> quat = SubscribeHandler::Normalize(v);
    return quat;
//      return v;
}


std::vector<float> SubscribeHandler::Normalize(std::vector<float> vect)
{
    float sum = 0;
    // normalize
    for (unsigned int i = 0; i == vect.size(); i++)
    {
        sum+= vect[i]*vect[i];
    }

    float scalar = 1/std::sqrt(sum);

    for (unsigned int j = 0; j == vect.size(); j++)
    {
        vect[j] *= scalar;
    }
    return vect;
}

void SubscribeHandler::Shutdown(){

	std::cout << "ROS shutdown" << std::endl;
	ros::shutdown();
    mpSLAM->Shutdown();
}



void SubscribeHandler::GetCurrentROSAllPointCloud( sensor_msgs::PointCloud2 &all_point_cloud, sensor_msgs::PointCloud2 &ref_point_cloud, cv::Mat matriz)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_all( new pcl::PointCloud<pcl::PointXYZRGBA> );  
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ref( new pcl::PointCloud<pcl::PointXYZRGBA> );     
    
    const vector<ORB_SLAM2::MapPoint*> &vpMPs = mpSLAM->GetAllMapPoints();
    const vector<ORB_SLAM2::MapPoint*> &vpRefMPs = mpSLAM->GetReferenceMapPoints();

    g2o::SE3Quat O_wm_wo = mpSLAM->GetInitialPose();
    Eigen::Matrix4f orb_world_pre = toEigMat(O_wm_wo);
    g2o::SE3Quat test = toSE3Quat(matriz.clone());
    Eigen::Matrix4f world_to_camera = toEigMat(test.inverse());

    
    set<ORB_SLAM2::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;
    
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        pcl::PointXYZRGBA p1;
        //g2o::SE3Quat pos_g2o = toSE3Quat(pos_cv.clone());
        //cv::Mat pos_inv = toCvMat(pos_g2o  * O_wm_wo.inverse());
        //cv::Mat pos = pos_inv.inv();
    Eigen::Vector4f p1_temp, p1_temp_t, p1_temp_pre;
    p1_temp(0) = pos.at<float>(0);
    p1_temp(1) = pos.at<float>(1);
    p1_temp(2) = pos.at<float>(2);
    p1_temp(3) = 1; 
    p1_temp_pre = orb_world_pre * p1_temp;
    p1_temp_t = world_to_camera * p1_temp_pre;    
    p1.x = p1_temp_t(0);
    p1.y = p1_temp_t(1);
    p1.z = p1_temp_t(2);
    p1.b = 0;
    p1.g = 0;
    p1.r = 255;
    p1.a = 255;
    cloud_all->points.push_back( p1 );
    }
    pcl::PCLPointCloud2 pcl_pc1;
    pcl::toPCLPointCloud2(*cloud_all, pcl_pc1);    // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc1, all_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    all_point_cloud.header.frame_id = "CameraTop_optical_frame";  
    all_point_cloud.header.stamp = ros::Time::now();   
  
    for(set<ORB_SLAM2::MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        pcl::PointXYZRGBA p2;
    Eigen::Vector4f p2_temp, p2_temp_t;
    p2_temp(0) = pos.at<float>(0);
    p2_temp(1) = pos.at<float>(1);
    p2_temp(2) = pos.at<float>(2);
    p2_temp(3) = 1;
    p2_temp_t = world_to_camera * orb_world_pre * p2_temp;    
    p2.x = p2_temp_t(0);
    p2.y = p2_temp_t(1);
    p2.z = p2_temp_t(2);
    p2.b = 0;
    p2.g = 255;
    p2.r = 0;
    p2.a = 255;
    cloud_ref->points.push_back( p2 );
    }
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud_ref, pcl_pc2); // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc2, ref_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    ref_point_cloud.header.frame_id = "CameraTop_optical_frame";
    ref_point_cloud.header.stamp = ros::Time::now();   

}

void SubscribeHandler::PointCloudPub(cv::Mat matriz)
{

          
    GetCurrentROSAllPointCloud(allMapPoints, referenceMapPoints, matriz.clone());
    AllPointCloud_pub_.publish(allMapPoints);
    RefPointCloud_pub_.publish(referenceMapPoints);
    
}


void SubscribeHandler::InitPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);

    if (msg->header.frame_id != "map"){
        ROS_WARN("Frame ID of \"initialpose\" (%s) is different from the global frame map", msg->header.frame_id.c_str());
    }

    // set offset so that current pose is set to "initialpose"
    tf::StampedTransform baseInMap;
    try{
        // just get the latest
        mpTFlistener->lookupTransform("base_footprint", "map", ros::Time(0), baseInMap);
    } catch(tf::TransformException){
        ROS_WARN("Failed to lookup transform!");
        return;
    }

    tf::Transform delta = pose * baseInMap;
    cv::Mat delta_cv = tfToMat(delta);
    offset_ = toEigMat(toSE3Quat(delta_cv));

}

 bool SubscribeHandler::Active(uchile_srvs::Onoff::Request  &req, uchile_srvs::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
            subImage = mpNodeHandler->subscribe(cameraTopic, 1, &SubscribeHandler::GrabImage, this);
            _is_on = true;
          
            ROS_INFO_STREAM("Turning on "+cameraTopic+" . . . OK");
        } else ROS_DEBUG_STREAM("Already turned on");
    }
    else{
        if (_is_on) {
              subImage.shutdown();
              _is_on = false;
              ROS_INFO_STREAM(" Turning off "+cameraTopic+" . . . OK");
        } else  ROS_DEBUG_STREAM("Already turned off"); 
    }
    return true;
}