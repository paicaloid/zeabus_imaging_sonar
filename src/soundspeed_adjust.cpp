#include <stdlib.h>
#include <bvt_sdk.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

BVTSDK::Sonar sonar ;
BVTSDK::Head head = NULL ;
BVTSDK::Ping ping = NULL ;
BVTSDK::ImageGenerator img_gen ;
BVTSDK::ColorMapper mapper ;
BVTSDK::ColorMapper AdapMapper ;
cv::Mat thres_rth ;

int binaryThreshold = 50 ;
int pos = 1500;
int set_gamma = 50 ;

double getPSNR(const cv::Mat& I1, const cv::Mat& I2){
    cv::Mat s1;
    absdiff(I1, I2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2

    cv::Scalar s = sum(s1);         // sum elements per channel

    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels

    if( sse <= 1e-10) // for small values return zero
        return 0;
    else{
        double  mse =sse /(double)(I1.channels() * I1.total());
        double psnr = 10.0*log10((255*255)/mse);
        return psnr;
    }
}

void SpeedChange( int , void*){
	img_gen.SetSoundSpeedOverride(pos);
}

void Set_Gamma( int , void* ){
	float int2float = set_gamma/100.0 ;
	AdapMapper.SetGamma(int2float) ;
}


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
    cv::namedWindow("AdapRThetaImage", cv::WINDOW_NORMAL);
    cv::createTrackbar("Threshold", "Threshold", &binaryThreshold, 255, ThresholdChange) ;
	cv::createTrackbar("Sound Speed" , "RThetaImage" , &pos , 2000 , SpeedChange);
    cv::createTrackbar("Gamma" , "AdapRThetaImage" , &set_gamma , 100 , Set_Gamma);

    // * Open .son file * //
	sonar.Open("FILE" , fullPath);
	head = sonar.GetHead(0);
	ping = head.GetPing(0);
	img_gen.SetHead(head);
	mapper.Load(mapperPath);
	mapper.SetAutoMode(1);

    // ! Create AdapMapper for measure PSNR //
    AdapMapper.Load(mapperPath);
    AdapMapper.SetAutoMode(0);

    while(1){
        
        ping = head.GetPing(-1) ;

        // * Generate RTheta Color image * //
        BVTSDK::MagImage rth = img_gen.GetImageRTheta(ping);
		BVTSDK::ColorImage cimgrth = mapper.MapImage(rth);
        BVTSDK::ColorImage Adaprth = AdapMapper.MapImage(rth);
		int height_rth = cimgrth.GetHeight();
		int width_rth = cimgrth.GetWidth();
		cv::Mat color_rth(height_rth , width_rth , CV_8UC4 , cimgrth.GetBits());
        cv::Mat thres_rth(height_rth , width_rth , CV_8UC4 , cimgrth.GetBits());
        cv::Mat adaptive_rth(height_rth , width_rth , CV_8UC4 , Adaprth.GetBits());
        
        cvtColor(color_rth, color_rth, cv::COLOR_RGB2GRAY);
        cvtColor(thres_rth, thres_rth, cv::COLOR_RGB2GRAY);
        cvtColor(adaptive_rth, adaptive_rth, cv::COLOR_RGB2GRAY);
        
        cv::imshow("RThetaImage", color_rth) ;
        cv::imshow("AdapRThetaImage", adaptive_rth) ;
        
        //cv::threshold(thres_rth, thres_rth, binaryThreshold, 255, cv::THRESH_TOZERO) ;
        cv::inRange(thres_rth, 210, 250, thres_rth) ;
        cv::imshow("Threshold", thres_rth) ;
        cv::waitKey(100) ;
        double psnr = getPSNR(color_rth, adaptive_rth) ;
        printf("%f\n",psnr) ;
    }
}