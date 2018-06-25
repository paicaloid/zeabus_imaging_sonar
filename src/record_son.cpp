#include <stdlib.h>
#include <bvt_sdk.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"

int main(int argc, char** argv){

    /// Setup Path ///
	std::string rootPath = "/home/paicaloid/bvtsdk/";
	std::string dataPath = rootPath + "data/";
	std::string fileName = "17_Jul_17_14.son";
	std::string fullPath = dataPath + fileName;

    printf("SDK Ready!!!\n") ;
	BVTSDK::Sonar sonar;
    printf("Create BVTSDK:Sonar : Success\n") ;

    sonar.Open("NET" , "192.168.1.45");
	printf("Sonar is Connect\n") ;

    BVTSDK::Head head = sonar.GetHead(0);
    printf("Create BVTSDK:Head : Success\n") ;

    float start_range = 1 ;
	float stop_range = 20 ;
	head.SetRange(start_range, stop_range) ;
    printf("Sonar SetRange from %f to %f : Success\n", start_range, stop_range) ;

    BVTSDK::Sonar file;
    printf("Create BVTSDK:Sonar for record : Success\n") ;

    file.CreateFile(fileName, sonar, "") ;
    printf("Create file to record : Success\n") ;

    BVTSDK::Head file_head = file.GetHead(0) ;
    printf("Create BVTSDK:Head for record : Success\n") ;

    int ping_num = 0 ;
    int max_ping = 150 ;

    BVTSDK::Ping ping = head.GetPing(0);
    printf("Ready for record\n") ;
    for(ping_num = 0; ping_num < max_ping; ping_num++)
    {
        ping = head.GetPing(-1); //DECIDE ping_num < 0 : save _____ ping_num >= 0 : open

        file_head.PutPing(ping) ; //write ping to file

        printf("Getting ping %d \n", ping_num);
    }



}