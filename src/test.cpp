#include <ros/ros.h>
#include <fstream>
#include <string> 
#include <iostream> 
#include "VehicleState/VehicleState.h"
#include "SensorData/DVLData.h"
#include "SensorData/ISData.h"
#include "SensorData/MTiData.h"
#include "KalmanFilter/DeadReckoning.h"
#include "utils/utils.h"

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
using namespace utils;


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
    // std::vector<dvlData::positionEstimate> pos = dvldata.getPosition();
    std::vector<dvlData::dvld> dvlmes = dvldata.getMeasurements();
    
    MTiData mtid;
    std::vector<MTiData::mtid> vmtid;
    vmtid = mtid.readMTiLOG("/home/unnar/catkin_ws/src/SonarSensor/sonardata/_040825_1735_MTi.log");

    

    int isi = 0, dvli = 0, mtii=0;
    // int issize = is_data.logtime.size();
    int dvlsize = dvlmes.size();
    int mtisize = vmtid.size();

    // double logtime = 0;
    double logtime = std::min(dvlmes.at(0).logtime, vmtid.at(0).logtime);
    VectorXd init(8);
    init << 0,0,0,vmtid.at(0).Yaw,0,0,0,0;
    DeadReckoning drec(init, logtime);
    VectorXd dvlm(4);
    VectorXd mtimes(1);
    double dT = 0.09;

    double x = 0.0, y = 0.0;
    cv::namedWindow( "Display window", WINDOW_NORMAL );
    Mat img(2000, 2000, CV_8UC3, Scalar::all(0));
    Point2d prev(1000.0,1000.0);
    Point2d curr(0.0,0.0);
    VectorXd updState;

    std::ofstream myfile;
    myfile.open ("/home/unnar/example.txt");

    while(dvli < dvlsize || mtii < mtisize){
        updState = drec.returnState();
        // std::cout << "x: " << updState(0) << "y: " << updState(1) << "theta: " << updState(3) << std::endl;
        myfile << updState(0) << "," << updState(1) << "," << updState(3) << "\n";
        if(dvli < dvlsize && dvlmes.at(dvli).logtime <= logtime){
            dvlm(0) = dvlmes.at(dvli).vby*cos(M_PI/6) + dvlmes.at(dvli).vbx*cos(M_PI/3);
            dvlm(1) = dvlmes.at(dvli).vby*sin(M_PI/6) + dvlmes.at(dvli).vbx*sin(M_PI/3);
            dvlm(2) = dvlmes.at(dvli).vbz;
            dvlm(3) = dvlmes.at(dvli).yaw;;
            drec.updateStateDVL(dvlm, dvlmes.at(dvli).logtime);
            updState = drec.returnState();

            curr.x = 1000 + updState(0)/1000;
            curr.y = 1000 + updState(1)/1000;
            cv::line(img, prev, curr, Scalar(0,50,255));
            // myfile << curr.x << "," << curr.y << "\n";
            prev=curr;
            dvli++;
            continue;
            // logtime = dvlmes.at(dvli).logtime;
        }
        if (mtii < mtisize && vmtid.at(mtii).logtime <= logtime){
            mtimes(0) = vmtid.at(mtii).Yaw;
            drec.updateStateMTi(mtimes, vmtid.at(mtii).logtime);
            updState = drec.returnState();
            curr.x = 1000 + updState(0)/1000;
            curr.y = 1000 + updState(1)/1000;
            cv::line(img, prev, curr, Scalar(255,255,255));
            // myfile << curr.x << "," << curr.y << "\n";
            prev=curr;
            mtii++;
            continue;
            // logtime = vmtid.at(mtii).logtime;
        }

        logtime += dT;
        drec.predictState(logtime);
        updState = drec.returnState();
        curr.x = 1000 + updState(0)/1000;
        curr.y = 1000 + updState(1)/1000;
        cv::line(img, prev, curr, Scalar(255,0,255));
        // myfile << curr.x << "," << curr.y << "\n";
        prev=curr;
    }

    myfile.close();
    // drec.predictState(logtime + dT);
    // drec.predictState(logtime + 2*dT);
    // drec.predictState(logtime + 3*dT);

    // VectorXd m(4);
    // m << 10, 10, 0.05 , 0.1;
    // drec.updateStateDVL(m, 0.01);

    // VectorXd t(1);
    // t << 1.14;
    // drec.updateStateMTi(t, 0.01);
     
     
    // for(auto i=0; i < 5; ++i){
    //     logtime += dT;
    //     drec.predictState(logtime); 
    //     std::cout << "predict" << std::endl;
    //     std::cout << drec.returnState() << std::endl;
    //     updState = drec.returnState();
    //     curr.x = 1000 + updState(0)*1000;
    //     curr.y = 1000 + updState(1)*1000;
    //     cv::line(img, prev, curr, Scalar(255,255,255));
    //     prev=curr;
    // }
    // dvlm << 1,0,0,0;
    // logtime += dT;
    // drec.updateStateDVL(dvlm,logtime);
    // std::cout << "update DVL" << std::endl;
    // std::cout << drec.returnState() << std::endl;
    // updState = drec.returnState();
    // curr.x = 1000 + updState(0)*1000;
    // curr.y = 1000 + updState(1)*1000;
    // cv::line(img, prev, curr, Scalar(0,255,255),2);
    // prev=curr;

    // mtimes << M_PI/6;
    // logtime += dT;
    // drec.updateStateMTi(mtimes,logtime);
    // std::cout << "update MTi" << std::endl;
    // std::cout << drec.returnState() << std::endl;
    // updState = drec.returnState();
    // curr.x = 1000 + updState(0)*1000;
    // curr.y = 1000 + updState(1)*1000;
    // cv::line(img, prev, curr, Scalar(255,0,255),2);
    // prev=curr;

    cv::imshow( "Display window", img ); 
    cv::waitKey(0);


    return 0;
}