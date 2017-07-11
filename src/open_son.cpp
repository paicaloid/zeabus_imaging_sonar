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

int thres = 0 ;
double maxval = 255 ;
int thres_type = CV_THRESH_BINARY ;

int interpolation = CV_INTER_LINEAR ;

BVTSDK::ImageGenerator img;
BVTSDK::ColorMapper map;
cv::Mat color_img ;
cv::Mat binary_img ;

void SpeedChange( int , void*){
	img.SetSoundSpeedOverride(pos);
}

void ThresholdChange( int , void*){
	threshold(binary_img, binary_img,thres, maxval, cv::THRESH_BINARY );
}

int main(int argc, char** argv){

	/// Setup Path ///
	std::string rootPath = "/home/paicaloid/bvtsdk/";
	std::string dataPath = rootPath + "data/";
	std::string mapperPath = rootPath + "colormaps/bone.cmap";
	std::string fileName = "10_Jul_17_bouy3.son";
	std::string fullPath = dataPath + fileName;

	pos = 1530;
	intensity = 30;
	thres = 70 ;

	cv::namedWindow("RawImage" , cv::WINDOW_NORMAL);
	cv::namedWindow("BinaryThresholding" , cv::WINDOW_NORMAL);
	cv::createTrackbar("Sound Speed" , "RawImage" , &pos , 2000 , SpeedChange);
	cv::createTrackbar("Threshold" , "BinaryThresholding" , &thres , 255 , ThresholdChange);
	
	printf("SDK Ready!!!\n") ;
	BVTSDK::Sonar sonar;
	sonar.Open("FILE" , fullPath);

	BVTSDK::Head head = sonar.GetHead(0);
	float start_range = 1 ;
	float stop_range = 20 ;
	head.SetRange(start_range, stop_range) ;

	BVTSDK::Ping ping = head.GetPing(0);
	
	int PingCount = head.GetPingCount();

	printf("Sonar Ping = %d\n",PingCount);

	img.SetHead(head);
	//BVTSDK::ColorMapper map;
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

	int p = 1;
	int k = 0;
	int j = 0 ;

	printf("Initial Publisher for /imaging_sonar\n") ;
	printf("Pause video by pressing : P\n") ;
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/imaging_sonar", 1);
	

	ros::Rate loop_rate(5);

	while (nh.ok()) {
		//map.Load(mapperPath);

		float Gamma = 0.5 ;
		map.SetGamma(Gamma) ;

		ping = head.GetPing(-1);
		BVTSDK::MagImage mag = img.GetImageXY(ping);
		BVTSDK::ColorImage cimg = map.MapImage(mag);

		float ClipThres = 1.0 ;
		img.SetClippingThreshold(ClipThres) ;



		//Gamma = map.GetGamma() ;
		//printf("Colormap Gamma : %f\n",Gamma) ;
		
		int height = cimg.GetHeight();
		int width = cimg.GetWidth();

		//printf("Height : %d, Width : %d\n",height, width) ;

		cv::Mat color_img(height , width , CV_8UC4 , cimg.GetBits());
		cv::Mat binary_img(height , width , CV_8UC4 , cimg.GetBits());
		// Pause by pressing 'P'
	 	while(!p){
	 		k = cv::waitKey(30);
	    	if (k == 112){
				printf("ClipThres : %f\n", ClipThres) ;
	    		p = !p;
	    	}
		}
		//double thres = 70 ;
		//double maxval = 255 ;
		//int thres_type = CV_THRESH_BINARY ;
		cvtColor(color_img, color_img, cv::COLOR_RGB2GRAY);
		cvtColor(binary_img, binary_img, cv::COLOR_RGB2GRAY);
		threshold(binary_img, binary_img,thres, maxval, cv::THRESH_BINARY );
		
		
		cv::imshow("RawImage", color_img);
		cv::imshow("BinaryThresholding", binary_img) ;
        k = cv::waitKey(30);
    	if( k == 27 )
    		break;
    	else if (k == 112){
    		p = !p;
    	}

		int row = color_img.rows ;
		int col = color_img.cols ;
		//printf("Row : %d, Column : %d\n",row, col) ;

		//resize image
		cv::resize(color_img, color_img, cv::Size(), 0.5, 0.5, interpolation) ;

		int roww = color_img.rows ;
		int coll = color_img.cols ;
		//printf("Row : %d, Column : %d\n",roww, coll) ;

		j++ ;
		if(j == PingCount)
		{
			printf("\n\n\n\n\n\n ### Finish ### \n\n\n\n\n\n") ;
			j = 0 ;
		}

		//printf("Sending Image\n") ;
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", color_img).toImageMsg();
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

		

	}
}

