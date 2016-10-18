//
//  SlamSystem.hpp
//  opencv_ios_try
//
//  Created by test on 10/18/16.
//  Copyright © 2016 test. All rights reserved.
//

#ifndef SlamSystem_hpp
#define SlamSystem_hpp

#include <stdio.h>
#include "deque"
#include <Eigen/Dense> 


class TrackingReference;
class SE3Tracker;
class DepthMap;
class Frame;
class FramePoseStruct;

typedef Eigen::Matrix<float, 7, 7> Matrix7x7;

class SlamSystem
{
public:
    
    // settings. Constant from construction onward.
    int width;
    int height;
    Eigen::Matrix3f K;
    
    SlamSystem(int w, int h, Eigen::Matrix3f K);
    SlamSystem(const SlamSystem&) = delete;
    SlamSystem& operator=(const SlamSystem&) = delete;
    ~SlamSystem();
    void randomInit(unsigned char* image, double timeStamp, int id);
    void trackFrame(unsigned char* image, unsigned int frameID, bool blockUntilMapped, double timestamp);
    
private:
    TrackingReference* trackingReference; // tracking reference for current keyframe. only used by tracking.
    SE3Tracker* tracker;
    DepthMap* map;
    
    std::shared_ptr<Frame> latestTrackedFrame;
    std::deque< Frame* > newKeyFrames;
    std::shared_ptr<Frame> currentKeyFrame;
};
#endif /* SlamSystem_hpp */
