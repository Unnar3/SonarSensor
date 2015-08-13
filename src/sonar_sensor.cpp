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

#define NODE_NAME       "s8_object_detection_node"

using namespace cv;



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
    ReadISLOG islog;
    ReadISLOG::isData is_data = islog.readISLOG(data_path);

    dvlData dvldata;
    dvldata.readLogFile("/home/unnar/catkin_ws/src/SonarSensor/sonardata/_040825_1735_DVL.log");
    std::vector<dvlData::positionEstimate> pos = dvldata.getPosition();
    
    // loop through both datasets 
    int dvlsize = pos.size();
    int issize = is_data.logtime.size();
    int dvlcounter = 0;
    int iscounter = 0;
    double dvllogtime = 0.0;
    double islogtime = 0.0;
    bool initialized = false;
    bool isnewer = true;

    namedWindow( "Display window",  WINDOW_NORMAL );
    int imagesize = 5000;
    cv::Mat image(imagesize, imagesize, CV_32FC1,0.0f);

    double angle,x,y;
    int numberrays = 20;
    double anglediff = std::abs(is_data.transducerangle.at(0) - is_data.transducerangle.at(1))/numberrays;

    while(dvlcounter < dvlsize || iscounter < issize){
        // Only start after dvl measurements have been recieved
        // get rid of the first measurements.
        if(!initialized){
            // First measurement, initialize first values.
            if(dvlcounter == 0 && iscounter == 0){
                islogtime  = is_data.logtime.at(iscounter);
                dvllogtime = pos.at(dvlcounter).logtime;
                dvlcounter++;
            } 
            // We have initialized, now step up sonar data 
            // until they are at the same time.
            else { 
                islogtime  = is_data.logtime.at(iscounter);
            }
            iscounter++;
            
            // If sonar data newer than dvl data we can start.
            if(islogtime > dvllogtime){
                initialized = true;
            }
            continue;
        }


        // Add one sonar ray to the image
        angle = is_data.transducerangle.at(iscounter);
        for ( size_t j = 0; j < is_data.bins.at(iscounter).size(); j++){
            for( int k = -numberrays/2 ; k < numberrays/2; k++){
                y = cos(angle-M_PI/6 + pos.at(dvlcounter).yaw + (float)k * anglediff) * j;
                x = sin(angle-M_PI/6 + pos.at(dvlcounter).yaw + (float)k * anglediff) * j;

                if(image.at<float>(imagesize/2 - (int)(pos.at(dvlcounter).y/0.1) + y,imagesize/2 + (int)(pos.at(dvlcounter).x/0.1) + x) < is_data.bins.at(iscounter).at(j)/100.0){
                    image.at<float>(imagesize/2 - (int)(pos.at(dvlcounter).y/0.1) + y,imagesize/2 + (int)(pos.at(dvlcounter).x/0.1) + x) = is_data.bins.at(iscounter).at(j)/100.0;
                }
            }
        }
        // check if we can add next ray or if new position has arrived
        iscounter++;
        if (is_data.logtime.at(iscounter) > pos.at(dvlcounter+1).logtime){
            dvlcounter++;
        }

        if(iscounter % 200 == 0){ 
            cv::Mat falseColorsMap;
            applyColorMap(image, falseColorsMap, cv::COLORMAP_AUTUMN);
            cv::imshow( "Display window", falseColorsMap );
            cv::waitKey(0);
        }
    }


    // cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    // cv::Mat image(1000, 1000, CV_32FC1,0.0f);
    // cv::Mat falseColorsMap;
    // double angle = 0.0;
    // double numberrays = 20;
    // double anglediff = std::abs(is_data.transducerangle.at(0) - is_data.transducerangle.at(1))/numberrays;


    // VehicleState state;

    // int x = 0, y = 0;
    // int circle_count = 0;
    // for (size_t i = 0; i < is_data.transducerangle.size(); ++i){
    //     angle = is_data.transducerangle.at(i);
    //     for ( size_t j = 0; j < is_data.bins.at(i).size(); j++){
    //         for( int k = -numberrays/2 ; k < numberrays/2; k++){
    //             y = cos(angle + (float)k * anglediff) * j;
    //             x = -sin(angle + (float)k * anglediff) * j;
    //             image.at<float>(499.0 + y,499.0 + x) = is_data.bins.at(i).at(j)/100.0;
    //         }
    //     }

    //     circle_count++; // Count the placement in the circle.
    //     if (angle == 0){
    //         std::cout << "circle count: " << circle_count << std::endl;
    //         circle_count = 0;

    //         applyColorMap(image, falseColorsMap, cv::COLORMAP_AUTUMN);



    //         cv::imshow( "Display window", falseColorsMap ); 
    //         cv::waitKey(0);

    //         // std::cout << image << std::endl;
    //     }
    // }

    /// Windows names
    // char path_window[] = "Drawing: Path";

    // /// Create black empty images
    // Mat path_image = Mat::zeros( 1000, 1000, CV_8UC3 );

    // double shrink = 100.00;
    // for ( auto i = 1; i < pos.size(); ++i ){ 
    // // std::cout << "x: " << pos.at(i).x << std::endl;
    // // std::cout << "y: " << pos.at(i).y << std::endl;
    //     MyLine( path_image, Point( pos.at(i-1).x/shrink+ 500, pos.at(i-1).y/shrink + 800), Point( pos.at(i).x/shrink+ 500, pos.at(i).y/shrink+ 800 ) );
    // }
    // cv::imshow( "Display window", path_image ); 
    // cv::waitKey(0);

    return 0;
}