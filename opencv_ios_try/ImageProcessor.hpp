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

class ImageProcessor{
public:
    int ProcessImage(cv::Mat img);
    std::vector<MapPointChamo> MapPoints;
};
#endif /* ImageProcessor_hpp */
