#include <stdlib.h>
#include <bvt_sdk.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

BVTSDK::Sonar sonar ;
BVTSDK::Head head = NULL ;
BVTSDK::Ping ping = NULL ;
BVTSDK::ImageGenerator img_gen ;
BVTSDK::ColorMapper color_map ;
cv::Mat thres_rth ;
int binaryThreshold = 50 ;

void ThresholdChange( int, void* ){
    //cv::threshold(thres_rth, thres_rth, binaryThreshold, 255, cv::THRESH_BINARY_INV) ;
}

int main(int argc, char** argv){

    // * Setup path * //
    std::string rootPath = "/home/paicaloid/bvtsdk/";
	std::string dataPath = rootPath + "data/";
	std::string mapperPath = rootPath + "colormaps/bone.cmap";
	std::string fileName = "Apr_27_2017_12842_6.son";
	std::string fullPath = dataPath + fileName;

    // * Create Windowname * //
	cv::namedWindow("Threshold" , cv::WINDOW_NORMAL);
	cv::namedWindow("RThetaImage", cv::WINDOW_NORMAL );
    cv::createTrackbar("Threshold", "Threshold", &binaryThreshold, 255, ThresholdChange) ;
	
    // * Open .son file * //
	sonar.Open("FILE" , fullPath);
	head = sonar.GetHead(0);
	ping = head.GetPing(0);
	img_gen.SetHead(head);
	color_map.Load(mapperPath);
	color_map.SetAutoMode(1);

    while(1){
        
        ping = head.GetPing(-1) ;

        // * Generate RTheta Color image * //
        BVTSDK::MagImage rth = img_gen.GetImageRTheta(ping);		// Get RTheta image from ping
		BVTSDK::ColorImage cimgrth = color_map.MapImage(rth);			// Covert to ColorImage with Colormapping
		int height_rth = cimgrth.GetHeight();
		int width_rth = cimgrth.GetWidth();
		cv::Mat color_rth(height_rth , width_rth , CV_8UC4 , cimgrth.GetBits());
        cv::Mat thres_rth(height_rth , width_rth , CV_8UC4 , cimgrth.GetBits());
        
        cvtColor(color_rth, color_rth, cv::COLOR_RGB2GRAY);
        cvtColor(thres_rth, thres_rth, cv::COLOR_RGB2GRAY);
        
        cv::imshow("RThetaImage", color_rth) ;
        
        //cv::threshold(thres_rth, thres_rth, binaryThreshold, 255, cv::THRESH_TOZERO) ;
        cv::inRange(thres_rth, 30, 50, thres_rth) ;
        cv::imshow("Threshold", thres_rth) ;
        cv::waitKey(100) ;
    }
}