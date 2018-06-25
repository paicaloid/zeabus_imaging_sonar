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
float ggg = 0 ;
int set_gamma = 50 ;
int top = 0 ;
int bottom = 0 ;

float start_range = 1 ;
float stop_range = 20 ;

int thres = 0 ;
double maxval = 255 ;
int thres_type = CV_THRESH_BINARY ;

int interpolation = CV_INTER_LINEAR ;

BVTSDK::Sonar sonar;
BVTSDK::Head head = NULL ;
BVTSDK::Ping ping = NULL ;
BVTSDK::ImageGenerator img;
BVTSDK::ColorMapper map;
cv::Mat color_img ;
cv::Mat binary_img ;

void Set_Gamma( int , void* ){
	float int2float = set_gamma/100.0 ;
	map.SetGamma(int2float) ;
}

void SetMapperThreshold( int , void*){
	if(top > bottom){
		map.SetThresholds(top, bottom);
	}
}

void SpeedChange( int , void*){
	img.SetSoundSpeedOverride(pos);
}

void ThresholdChange( int , void*){
	threshold(binary_img, binary_img,thres, maxval, cv::THRESH_BINARY );
}

void Sonar_status(){
	std::string model = sonar.GetSonarModelName() ;
	int fileSize = sonar.GetFileSize() ;
	// int timeIndex = sonar.GetTimeIndexedPingCount() ;	// useless
	// float temp = sonar.GetTemperature() ;				// coredump
	// int NavData = sonar.GetNavDataCount() ; 				// Navdata = 0, need find more information about NavData

	printf("Sonar model : %s\n", model.c_str()) ;
	printf("File size : %d bytes\n", fileSize) ;
	// printf("Time index : %d\n", timeIndex) ;
	// printf("Temperature : %f\n", temp) ;
	// printf("NavData count : %d\n", NavData) ;

	if(sonar.SupportsMulticast()){
		printf("SUPPORTTTTTTT\n") ;
	}

	float gain = head.GetGainAdjustment() ;
	int freq = head.GetCenterFreq() ;
	int PingCount = head.GetPingCount();
	printf("Gain : %f dB\n", gain) ;
	printf("Freq : %d Hz\n", freq) ;
	printf("Number of Ping = %d\n",PingCount);

	double timeStamp = ping.GetTimestamp() ;
	int timeZone = ping.GetTimeZoneOffset() ;
	printf("timeStamp : %d\n", timeZone) ;

	if(ping.HasNavData()){
		printf("NAVDATA\n") ;
	}

}

int main(int argc, char** argv){

	/// Setup Path ///
	std::string rootPath = "/home/paicaloid/bvtsdk/";
	std::string dataPath = rootPath + "data/";
	std::string mapperPath = rootPath + "colormaps/bone.cmap";
	std::string fileName = "Apr_27_2017_12842_6.son";
	std::string fullPath = dataPath + fileName;

	pos = 1530;
	intensity = 30;
	thres = 70 ;

	cv::namedWindow("RawImage" , cv::WINDOW_NORMAL);
	//cv::namedWindow("BinaryThresholding" , cv::WINDOW_NORMAL);
	cv::namedWindow("RThetaImage", cv::WINDOW_NORMAL );
	cv::createTrackbar("Sound Speed" , "RawImage" , &pos , 2000 , SpeedChange);
	//cv::createTrackbar("Threshold" , "BinaryThresholding" , &thres , 255 , ThresholdChange);
	cv::createTrackbar("Sound Speed" , "RThetaImage" , &pos , 2000 , SpeedChange);

	sonar.Open("FILE" , fullPath);
	head = sonar.GetHead(0);
	head.SetRange(start_range, stop_range) ;
	ping = head.GetPing(0);
	Sonar_status() ;

	img.SetHead(head);
	float resolution = img.GetRangeResolution() ;
	printf("Resolution : %f meters\n", resolution*100000) ;
	float xx = img.GetClippingThreshold();
	float yy = img.GetNoiseThreshold() ;
	printf("===============> %f %f\n", xx, yy) ;
	// printf("===============> %f &f\n", xx, yy) ;
	
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

	top = map.GetTopThreshold() ;
	bottom = map.GetBottomThreshold() ;
	ggg = map.GetGamma() ;

	printf("Gamma : %f\n", ggg) ;
	printf("Top : %d\n", top) ;
	printf("Bottom : %d\n", bottom) ;

	// int top = 15000 ;
	// int bottom = 10000 ;
	
	// map.SetThresholds(top, bottom) ;

	cv::createTrackbar("Top Thres" , "RawImage" , &top , 32700 , SetMapperThreshold);
	cv::createTrackbar("Bottom Thres" , "RawImage" , &bottom , 32700 , SetMapperThreshold);
	cv::createTrackbar("Gamma" , "RawImage" , &set_gamma , 100 , Set_Gamma);

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

	int pai = 0 ;

	while (nh.ok()) {
		//map.Load(mapperPath);

		// float Gamma = 0.5 ;
		// map.SetGamma(Gamma) ;

		

		ping = head.GetPing(-1);
		BVTSDK::MagImage mag = img.GetImageXY(ping);
		BVTSDK::MagImage rth = img.GetImageRTheta(ping);
		BVTSDK::ColorImage cimg = map.MapImage(mag);
		BVTSDK::ColorImage cimgrth = map.MapImage(rth) ;

		// float ClipThres = 1.0 ;
		// img.SetClippingThreshold(ClipThres) ;
		
		// pai++ ;
		// if(pai == 10 ){
		// 	pai = 0 ;
		// 	clip = clip + 100 ;
		// 	head.SetClippingThreshold(clip) ;
		// 	printf("Clipping : %d\n", head.GetClippingThreshold()) ;
		// }

		//Gamma = map.GetGamma() ;
		//printf("Colormap Gamma : %f\n",Gamma) ;
		
		int height = cimg.GetHeight();
		int width = cimg.GetWidth();

		int height_rth = cimgrth.GetHeight();
		int width_rth = cimgrth.GetWidth();

		//printf("Height : %d, Width : %d\n",height, width) ;
		
		// unsigned int *bits = cimg.GetBits() ;
		// printf("%u\n", bits) ;
		
		cv::Mat color_img(height , width , CV_8UC4 , cimg.GetBits());
		cv::Mat binary_img(height , width , CV_8UC4 , cimg.GetBits());
		cv::Mat color_rth(height_rth , width_rth , CV_8UC4 , cimgrth.GetBits());

		// Pause by pressing 'P'
	 	while(!p){
	 		k = cv::waitKey(30);
	    	if (k == 112){
				// printf("ClipThres : %f\n", ClipThres) ;
	    		p = !p;
	    	}
		}
		//double thres = 70 ;
		//double maxval = 255 ;
		//int thres_type = CV_THRESH_BINARY ;
		cvtColor(color_img, color_img, cv::COLOR_RGB2GRAY);
		cvtColor(color_rth, color_rth, cv::COLOR_RGB2GRAY);
		//cvtColor(binary_img, binary_img, cv::COLOR_RGB2GRAY);
		//threshold(binary_img, binary_img,thres, maxval, cv::THRESH_BINARY );
		
		
		cv::imshow("RawImage", color_img);
		//cv::imshow("BinaryThresholding", binary_img) ;
		cv::imshow("RThetaImage", color_rth) ;
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
		cv::resize(color_rth, color_rth, cv::Size(), 0.5, 0.5, interpolation) ;
		int roww = color_img.rows ;
		int coll = color_img.cols ;
		//printf("Row : %d, Column : %d\n",roww, coll) ;

		// j++ ;
		// if(j == PingCount)
		// {
		// 	printf("\n\n\n\n\n\n ### Finish ### \n\n\n\n\n\n") ;
		// 	j = 0 ;
		// }

		//printf("Sending Image\n") ;
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", color_rth).toImageMsg();
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

		

	}
}