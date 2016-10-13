#include "ImageProcessor.hpp"

int ImageProcessor::ProcessImage(cv::Mat img){
    MapPoints.clear();
    MapPoints.resize(img.rows);
    if (img.type()!= CV_8UC1){
        img.convertTo(img, CV_8UC1);
    }
    for(int i=0;i<img.rows;i++){
        int centerX =(int)(img.cols/2);
        int centerY =(int)(img.rows/2);
        MapPoints[i].posi.x = img.at<unsigned char>(i,centerX-1);
        MapPoints[i].posi.y = img.at<unsigned char>(i,centerX);
        MapPoints[i].posi.z = img.at<unsigned char>(i,centerX+1);
        std::cout<<MapPoints[i].posi<<std::endl;
    }
    return 1;
}
