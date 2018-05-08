/**
* This file is part of the odometry integration of pepper in SLAM
* Author: Tim Resink
* Email: timresink@gmail.com
*/

// ROS
#include <Eigen/StdVector>

// Eigen
#include <Eigen/Geometry>

// own files
#include "SubscribeHandler.hpp"

// g2o
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"




int main(int argc, char * argv[]){
	// initialize ROS. Allows for name remapping (something:/= "somethingelse")
	// in the command line.
    // Listener is the name of the node written here

    SubscribeHandler maquiHandler(argv[1], argv[2]);

    if(argc != 3)
    {
         cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
         maquiHandler.Shutdown();
        return 1;
    }



	maquiHandler.Shutdown();

	return 0;
};

