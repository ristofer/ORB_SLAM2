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

// cv bridge
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

//Eigen
#include <Eigen/Geometry>

// ORB-SLAM
#include "System.h"


class SubscribeHandler{
public:
	// constructors
    SubscribeHandler(const string &strVocFile,
                     const string &strSettingsFile);

    // methods
	void Shutdown();

private:
    // methods


    // ORB SLAM pointer
    ORB_SLAM2::System* mpSLAM;

    // flags
    bool mbReferenceWorldFrame;

    int useBaseFrame;
};
