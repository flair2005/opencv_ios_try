 #include <stdio.h>
#include <iomanip>
#include <string.h>
#include <vector>
#include <SlamSystem.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

cv::Mat getAImage(int id, std::string imageAddr){
    std::stringstream ss;
    ss<<imageAddr<<id <<".png";
    std::string frameName = ss.str();
    cv::Mat img = cv::imread(frameName, CV_LOAD_IMAGE_GRAYSCALE);
    if (img.type()!= CV_8UC1){
        img.convertTo(img, CV_8UC1);
    }
    return img;
};

int main(int argc, char **argv) {
    
    std::string imageAddr = argv[1];
    std::string confAddr = argv[2];
    int startFrame = atoi(argv[3]);
    int endFrame = atoi(argv[4]);
    float fx = atof(argv[5]);
    float fy = atof(argv[6]);
    float cx = atof(argv[7]);
    float cy = atof(argv[8]);
    std::string caseName = argv[9];
    int saveOutput = 0;
    std::string outputRoot;
    if (argc > 10){
        saveOutput = 1;
        outputRoot = argv[10];
    }

    Eigen::Matrix3f K;
    
    K(0,0) = fx;
    K(1,1) = fy;
    K(0,2) = cx;
    K(1,2) = cy;
    
    lsd_slam::SlamSystem *system=NULL;
    
    {
        cv::Mat img = getAImage(startFrame ,imageAddr);
        system = new lsd_slam::SlamSystem(img.cols, img.rows, K);
        system->randomInit(img.data, startFrame, startFrame);
    }
    
    for (int i=startFrame+1;i<endFrame;i++){
        cv::Mat img = getAImage(startFrame ,imageAddr);
        system->trackFrame(img.data, i, false, 0);
    }
}
