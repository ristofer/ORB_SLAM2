/**
* This file is part of the odometry integration of pepper in SLAM
* Author: Tim Resink
* Email: timresink@gmail.com
*/
#include <Eigen/StdVector>
#include"SubscribeHandler.hpp"

using namespace Eigen;



SubscribeHandler::SubscribeHandler(const string &strVocFile,
                                   const string &strSettingsFile):
mbReferenceWorldFrame(false)
{
      cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);

      mpSLAM = new ORB_SLAM2::System(strVocFile, strSettingsFile, ORB_SLAM2::System::MONOCULAR,true,true);

      // Initialize ORB system
   // argument 4 boolean is user viewer
}





void SubscribeHandler::Shutdown(){

	std::cout << "ROS shutdown" << std::endl;
    mpSLAM->Shutdown();
}



