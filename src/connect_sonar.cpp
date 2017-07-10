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

int pos = 0;
<<<<<<< HEAD
int intensity = 0;
double int2double = 0;

int thres = 0 ;
double maxval = 255 ;
int thres_type = CV_THRESH_BINARY ;

float start_range = 1 ;
float stop_range = 20 ;
=======
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f

BVTSDK::ImageGenerator img;
BVTSDK::ColorMapper map;
cv::Mat color_img ;
<<<<<<< HEAD
cv::Mat binary_img ;
=======
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f

void SpeedChange( int , void*){
	img.SetSoundSpeedOverride(pos);
}

<<<<<<< HEAD
void ThresholdChange( int , void*){
	threshold(binary_img, binary_img,thres, maxval, cv::THRESH_BINARY );
}

=======
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
int main(int argc, char** argv){
    /// Setup Path ///
	std::string rootPath = "/home/paicaloid/bvtsdk/";
	//std::string dataPath = rootPath + "data/";
	std::string mapperPath = rootPath + "colormaps/bone.cmap";
	std::string fileName = "test_001.son";
	std::string storePath = "/home/paicaloid/catkin_ws/src/cpp_sonar/data" ;
	std::string fullPath = storePath + fileName;

    pos = 1500;
<<<<<<< HEAD
	intensity = 30;
	thres = 70 ;

	cv::namedWindow("RawImage" , cv::WINDOW_NORMAL);
	cv::namedWindow("BinaryThresholding" , cv::WINDOW_NORMAL);
	cv::createTrackbar("Sound Speed" , "RawImage" , &pos , 2000 , SpeedChange);
	cv::createTrackbar("Threshold" , "BinaryThresholding" , &thres , 255 , ThresholdChange);

	printf("SDK Ready!!!\n") ;
=======

    cv::namedWindow("RawImage" , cv::WINDOW_NORMAL);
	cv::createTrackbar("Sound Speed" , "RawImage" , &pos , 2000 , SpeedChange);

	printf("Before Connect\n") ;
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f

    BVTSDK::Sonar sonar;
    sonar.Open("NET" , "192.168.1.45");

<<<<<<< HEAD
	printf("Sonar is Connect\n") ;

	//int head_count ;
	//head_count = sonar.GetHeadCount() ;
	//printf("Sonar has %d HeadCount\n",head_count) ;

    BVTSDK::Head head = sonar.GetHead(0);
	head.SetRange(start_range, stop_range) ;

	BVTSDK::Ping ping = head.GetPing(0);

	
    img.SetHead(head);
=======
	printf("After Connect\n") ;

	int head_count ;
	head_count = sonar.GetHeadCount() ;
	printf("Sonar has %d HeadCount\n",head_count) ;

    BVTSDK::Head head = sonar.GetHead(0);
	float start_range = 1 ;
	float stop_range = 20 ;
	head.SetRange(start_range, stop_range) ;

	// Create file to save into .son
	printf("Create file for store .son\n") ;
	BVTSDK::Sonar sonfile;
	sonfile.CreateFile(fileName, sonar, "" ) ;

	printf("-----\n") ;
	BVTSDK::Head out_head = NULL ;
	sonfile.GetHead(0) ;

	BVTSDK::Ping ping = head.GetPing(0);
	
    img.SetHead(head);
    map.Load(mapperPath);
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f

    int p = 1;
	int k = 0;

	printf("Initial Publisher for /imaging_sonar\n") ;
    ros::init(argc, argv, "connect_sonar_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
<<<<<<< HEAD
	image_transport::Publisher pub = it.advertise("/connect_image_sonar", 1);

	ros::Rate loop_rate(5);
=======
	image_transport::Publisher pub = it.advertise("connect_image_sonar", 1);
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f

    while (nh.ok()) {
		
        map.Load(mapperPath);
<<<<<<< HEAD

=======
		printf("++++++\n") ;
		out_head.PutPing(ping) ;
		printf("Get Image\n") ;
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
        ping = head.GetPing(-1);
		BVTSDK::MagImage mag = img.GetImageXY(ping);
		BVTSDK::ColorImage cimg = map.MapImage(mag);

        int height = cimg.GetHeight();
		int width = cimg.GetWidth();

		cv::Mat color_img(height , width , CV_8UC4 , cimg.GetBits());
<<<<<<< HEAD
		cv::Mat binary_img(height , width , CV_8UC4 , cimg.GetBits());
        
		// Pause by pressing 'P'
=======
        // Pause by pressing 'P'
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
        while(!p){
	 		k = cv::waitKey(30);
	    	if (k == 112){
	    		p = !p;
	    	}
		}

<<<<<<< HEAD
		cvtColor(color_img, color_img, cv::COLOR_RGB2GRAY);
		cvtColor(binary_img, binary_img, cv::COLOR_RGB2GRAY);
		threshold(binary_img, binary_img,thres, maxval, cv::THRESH_BINARY );

        cv::imshow("RawImage", color_img);
		cv::imshow("BinaryThresholding", binary_img) ;

=======
        cv::imshow("RawImage", color_img);
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
        k = cv::waitKey(30);
    	if( k == 27 )
    		break;
    	else if (k == 112){
    		p = !p;
    	}
<<<<<<< HEAD
		
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", color_img).toImageMsg();
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
=======
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f

		

    }
}