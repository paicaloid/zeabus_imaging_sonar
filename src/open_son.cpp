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

double maxval = 255 ;

int thres_type = CV_THRESH_BINARY ;
int interpolation = CV_INTER_LINEAR ;

BVTSDK::Sonar sonar;
BVTSDK::Head head = NULL ;
BVTSDK::Ping ping = NULL ;
BVTSDK::ImageGenerator img;
BVTSDK::ColorMapper map;
BVTSDK::ColorMapper automap;
// cv::Mat color_img ;
// cv::Mat auto_img ;

/// Changing Gamma in ColorMapper ///
void Set_Gamma( int , void* ){
	float int2float = set_gamma/100.0 ;
	map.SetGamma(int2float) ;
}

/// Changing Threshold in ColorMapper ///
void SetMapperThreshold( int , void*){
	if(top > bottom){
		map.SetThresholds(top, bottom);
	}
}

/// Changing Sound speed in ImageGenerator ///
void SpeedChange( int , void*){
	img.SetSoundSpeedOverride(pos);
}

/// Show Sonar information ///
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
	std::string rootPath = "/home/paicaloid/bvtsdk/";			// SDK path
	std::string dataPath = rootPath + "data/";					// .son path
	std::string mapperPath = rootPath + "colormaps/bone.cmap";	// Colormapper path
	std::string fileName = "Apr_27_2017_12842_6.son";			// filename of .son
	std::string fullPath = dataPath + fileName;

	/// Create Windowname ///
	cv::namedWindow("RawImage" , cv::WINDOW_NORMAL);
	cv::namedWindow("RThetaImage", cv::WINDOW_NORMAL );
	cv::namedWindow("RawImage_Auto", cv::WINDOW_NORMAL );
	
	// Open .son file ///
	sonar.Open("FILE" , fullPath);				// Open .son file
	head = sonar.GetHead(0);					// Connect head to sonar, 0 = single-head sonar
	head.SetRange(start_range, stop_range) ;	// Setup range 
	ping = head.GetPing(0);						// Connnect Ping to head with specific ping (Ping #0 or first ping)
	img.SetHead(head);							// Create ImageGenerator
	map.Load(mapperPath);						// Load Colormapper
	map.SetAutoMode(0);							// Disable Auto Parameter, 0 = disable
	automap.Load(mapperPath) ;					// Load Colormapper
	automap.SetAutoMode(1) ;					// Enable Auto Parameter, 1 = enable 
	Sonar_status() ;

	/// Create Trackbar ///
	pos = 1530;
	top = map.GetTopThreshold() ;
	bottom = map.GetBottomThreshold() ;
	ggg = map.GetGamma() ;
	/// RawImage Trackerbar ///
	cv::createTrackbar("Sound Speed" , "RawImage" , &pos , 2000 , SpeedChange);
	cv::createTrackbar("Top Thres" , "RawImage" , &top , 32700 , SetMapperThreshold);
	cv::createTrackbar("Bottom Thres" , "RawImage" , &bottom , 32700 , SetMapperThreshold);
	cv::createTrackbar("Gamma" , "RawImage" , &set_gamma , 100 , Set_Gamma);
	/// Soundspeed Trackbar ///
	cv::createTrackbar("Sound Speed" , "RThetaImage" , &pos , 2000 , SpeedChange);
	cv::createTrackbar("Sound Speed" , "RawImage_Auto" , &pos , 2000 , SpeedChange);
	

	int p = 1;
	int k = 0;
	int j = 0 ;

	/// ROS Initial ///
	printf("Initial Publisher for /imaging_sonar\n") ;
	printf("Pause video by pressing : P\n") ;
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/imaging_sonar", 1);
	ros::Rate loop_rate(5);


	int pai = 0 ;

	while (nh.ok()) {

		ping = head.GetPing(-1);  	// Getping less than 0 is get next ping

		/// Genarate XY ColorImage ///
		BVTSDK::MagImage mag = img.GetImageXY(ping);			// Get XY image from ping
		BVTSDK::ColorImage cimg = map.MapImage(mag);			// Covert to ColorImage with Colormapping
		BVTSDK::ColorImage cimg_auto = automap.MapImage(mag);	// Covert to ColorImage with Colormapping

		/// Generate RTheta ColorImage ///
		BVTSDK::MagImage rth = img.GetImageRTheta(ping);		// Get RTheta image from ping
		BVTSDK::ColorImage cimgrth = map.MapImage(rth);			// Covert to ColorImage with Colormapping
		
		/// Find XY Image resolution ///
		int height = cimg.GetHeight();
		int width = cimg.GetWidth();
		printf("XY Image resolution : %d x %d\n", width, height) ;

		/// Find RTheta Image resolution ///
		int height_rth = cimgrth.GetHeight();
		int width_rth = cimgrth.GetWidth();
		printf("RTheta Image resolution : %d x %d\n", width_rth, height_rth) ;
		
		/// Convert to Matrix format ///
		/// GetBits() will return pointer of the entrie image ///
		cv::Mat color_img(height , width , CV_8UC4 , cimg.GetBits());
		cv::Mat auto_img(height , width , CV_8UC4 , cimg_auto.GetBits());
		cv::Mat color_rth(height_rth , width_rth , CV_8UC4 , cimgrth.GetBits());

		// Pause by pressing 'P' ///
	 	while(!p){
	 		k = cv::waitKey(30);
	    	if (k == 112){
	    		p = !p;
	    	}
		}
		
		/// Chgange Image to grayscale ///
		cvtColor(color_img, color_img, cv::COLOR_RGB2GRAY);
		cvtColor(auto_img, auto_img, cv::COLOR_RGB2GRAY);
		cvtColor(color_rth, color_rth, cv::COLOR_RGB2GRAY);

		/// Show Image ///
		cv::imshow("RawImage", color_img);
		cv::imshow("RawImage_Auto", auto_img) ;
		cv::imshow("RThetaImage", color_rth) ;
        
		/// unknown ///
		k = cv::waitKey(30);
    	if( k == 27 )
    		break;
    	else if (k == 112){
    		p = !p;
    	}

		// int row = color_img.rows ;
		// int col = color_img.cols ;

		/// Resize Image for sending through ROS ///
		cv::resize(color_img, color_img, cv::Size(), 0.5, 0.5, interpolation) ;
		cv::resize(color_rth, color_rth, cv::Size(), 0.5, 0.5, interpolation) ;
		int roww = color_img.rows ;
		int coll = color_img.cols ;

		/// ROS Message ///
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", color_rth).toImageMsg();
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
}