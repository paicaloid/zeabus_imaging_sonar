<<<<<<< HEAD
=======

>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
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
int intensity = 0;
double int2double = 0;

int thres = 0 ;
double maxval = 255 ;
int thres_type = CV_THRESH_BINARY ;

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

void ThresholdChange( int , void*){
<<<<<<< HEAD
	threshold(binary_img, binary_img,thres, maxval, cv::THRESH_BINARY );
=======
	threshold(color_img, color_img,thres, maxval, cv::THRESH_BINARY );
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
}

int main(int argc, char** argv){

	/// Setup Path ///
	std::string rootPath = "/home/paicaloid/bvtsdk/";
	std::string dataPath = rootPath + "data/";
	std::string mapperPath = rootPath + "colormaps/bone.cmap";
<<<<<<< HEAD
	std::string fileName = "10_Jul_17_bouy3.son";
	std::string fullPath = dataPath + fileName;

	pos = 1530;
=======
	std::string fileName = "Mar_01_2017_115453.son";
	std::string fullPath = dataPath + fileName;

	pos = 1500;
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
	intensity = 30;
	thres = 70 ;

	cv::namedWindow("RawImage" , cv::WINDOW_NORMAL);
<<<<<<< HEAD
	cv::namedWindow("BinaryThresholding" , cv::WINDOW_NORMAL);
	cv::createTrackbar("Sound Speed" , "RawImage" , &pos , 2000 , SpeedChange);
	cv::createTrackbar("Threshold" , "BinaryThresholding" , &thres , 255 , ThresholdChange);
=======
	//cv::namedWindow("BinaryThresholding" , cv::WINDOW_NORMAL);
	cv::createTrackbar("Sound Speed" , "RawImage" , &pos , 2000 , SpeedChange);
	//cv::createTrackbar("Threshold" , "BinaryThresholding" , &thres , 255 , ThresholdChange);
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
	
	printf("SDK Ready!!!\n") ;
	BVTSDK::Sonar sonar;
	sonar.Open("FILE" , fullPath);
<<<<<<< HEAD
	//sonar.Open("NET" , "192.168.1.45");

	BVTSDK::Head head = sonar.GetHead(0);
	float start_range = 1 ;
	float stop_range = 20 ;
	head.SetRange(start_range, stop_range) ;

=======

	BVTSDK::Head head = sonar.GetHead(0);
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
	BVTSDK::Ping ping = head.GetPing(0);
	
	int i = head.GetPingCount();

	img.SetHead(head);
	//BVTSDK::ColorMapper map;
<<<<<<< HEAD
	map.Load(mapperPath);

	map.SetAutoMode(1);

	if(map.GetAutoMode())
	{
		printf("AutoMode : Enable\n");
	}
	else
	{
		printf("AutoMode : Disable\n");
	}

	int top = map.GetTopThreshold() ;
	int bottom = map.GetBottomThreshold() ;

	printf("Top : %d\n", top) ;
	printf("Bottom : %d\n", bottom) ;

	//int top = 15000 ;
	//int bottom = 10000 ;
	
	//map.SetThresholds(top, bottom) ;
=======
	//map.Load(mapperPath);
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f

	int p = 1;
	int k = 0;

	printf("Initial Publisher for /imaging_sonar\n") ;
	printf("Pause video by pressing : P\n") ;
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/imaging_sonar", 1);
	

	ros::Rate loop_rate(5);

	while (nh.ok()) {
<<<<<<< HEAD
		//map.Load(mapperPath);
=======
		map.Load(mapperPath);
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f

		float Gamma = 0.5 ;
		map.SetGamma(Gamma) ;

		ping = head.GetPing(-1);
		BVTSDK::MagImage mag = img.GetImageXY(ping);
		BVTSDK::ColorImage cimg = map.MapImage(mag);

<<<<<<< HEAD
		float ClipThres = 1.0 ;
		img.SetClippingThreshold(ClipThres) ;



=======
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
		//Gamma = map.GetGamma() ;
		//printf("Colormap Gamma : %f\n",Gamma) ;
		
		int height = cimg.GetHeight();
		int width = cimg.GetWidth();

		//printf("Height : %d, Width : %d\n",height, width) ;

		cv::Mat color_img(height , width , CV_8UC4 , cimg.GetBits());
<<<<<<< HEAD
		cv::Mat binary_img(height , width , CV_8UC4 , cimg.GetBits());
=======
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
		// Pause by pressing 'P'
	 	while(!p){
	 		k = cv::waitKey(30);
	    	if (k == 112){
<<<<<<< HEAD
				printf("ClipThres : %f\n", ClipThres) ;
=======
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
	    		p = !p;
	    	}
		}
		//double thres = 70 ;
		//double maxval = 255 ;
		//int thres_type = CV_THRESH_BINARY ;
		cvtColor(color_img, color_img, cv::COLOR_RGB2GRAY);
<<<<<<< HEAD
		cvtColor(binary_img, binary_img, cv::COLOR_RGB2GRAY);
		threshold(binary_img, binary_img,thres, maxval, cv::THRESH_BINARY );
=======
		//threshold(color_img, color_img,thres, maxval, cv::THRESH_BINARY );
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
		
		
		
		cv::imshow("RawImage", color_img);
<<<<<<< HEAD
		cv::imshow("BinaryThresholding", binary_img) ;
=======
>>>>>>> 9722cbba9574fc4cfa099b995af8822aff401e6f
        k = cv::waitKey(30);
    	if( k == 27 )
    		break;
    	else if (k == 112){
    		p = !p;
    	}
    	i--;

		//printf("Sending Image\n") ;
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", color_img).toImageMsg();
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
}

