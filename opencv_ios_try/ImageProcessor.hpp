//
//  ImageProcessor.hpp
//  opencv_ios_try
//
//  Created by test on 10/11/16.
//  Copyright © 2016 test. All rights reserved.
//

#ifndef ImageProcessor_hpp
#define ImageProcessor_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "CommonStruct.h"

class SlamSystem;

class ImageProcessor{
public:
    int ProcessImage(cv::Mat img);
    void getMP(std::vector<MapPointChamo>& mps, cv::Mat& pose);
    std::vector<MapPointChamo> MapPoints;
    std::vector<cv::Mat> imgs;
    SlamSystem* system;
};
#endif /* ImageProcessor_hpp */
