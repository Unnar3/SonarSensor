#include <ros/ros.h>
#include <fstream>
#include <string> 
#include <iostream> 
#include "VehicleState/VehicleState.h"
#include "SensorData/DVLData.h"
#include "SensorData/ISData.h"
#include "SensorData/MTiData.h"
#include "KalmanFilter/DeadReckoning.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <unistd.h>
#include <Eigen/Dense>

#define NODE_NAME       "SonarSensor_test"

using namespace cv;
using namespace Eigen;


void MyLine( Mat img, Point start, Point end )
    {
      int thickness = 1;
      int lineType = 8;
      line( img, 
        start,
        end,
        Scalar( 255, 255, 255 ),
        thickness,
        lineType );
    };

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    std::string data_path = "/home/unnar/catkin_ws/src/SonarSensor/sonardata/_040825_1735_IS.log";
    
    // Read sonar data;
    // ReadISLOG islog;
    // ReadISLOG::isData is_data = islog.readISLOG(data_path);

    dvlData dvldata;
    dvldata.readLogFile("/home/unnar/catkin_ws/src/SonarSensor/sonardata/_040825_1735_DVL.log");
    std::vector<dvlData::positionEstimate> pos = dvldata.getPosition();
    
    MTiData mtid;
    std::vector<MTiData::mtid> vmtid;
    vmtid = mtid.readMTiLOG("/home/unnar/catkin_ws/src/SonarSensor/sonardata/_040825_1735_MTi.log");


    VectorXd init(8);
    init << 0,0,0,0,1,1,0,0;
    double logtime = 0.0;
    double dT = 0.01;

    DeadReckoning drec(init, 0.01, logtime);
    drec.predictState(logtime + dT);
    drec.predictState(logtime + 2*dT);
    drec.predictState(logtime + 3*dT);

    VectorXd m(4);
    m << 10, 10, 0.05 , 0.1;
    drec.updateStateDVL(m, 0.01);

    VectorXd t(1);
    t << 1.14;
    drec.updateStateMTi(t, 0.01);

    return 0;
}