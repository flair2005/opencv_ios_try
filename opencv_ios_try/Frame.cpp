//
//  Frame.cpp
//  opencv_ios_try
//
//  Created by test on 10/17/16.
//  Copyright © 2016 test. All rights reserved.
//

#include "Frame.hpp"
#include "FramePoseStruct.hpp"

int privateFrameAllocCount = 0;

Frame::Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image)
{
    initialize(id, width, height, K, timestamp);
    
    float* maxPt = data.image[0] + data.width[0]*data.height[0];
    
    for(float* pt = data.image[0]; pt < maxPt; pt++)
    {
        *pt = *image;
        image++;
    }
    data.imageValid[0] = true;
    privateFrameAllocCount++;
}

void Frame::initialize(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp)
{
    data.id = id;
    
    pose = new FramePoseStruct(this);
    
    data.K[0] = K;
    data.fx[0] = K(0,0);
    data.fy[0] = K(1,1);
    data.cx[0] = K(0,2);
    data.cy[0] = K(1,2);
    
    data.KInv[0] = K.inverse();
    data.fxInv[0] = data.KInv[0](0,0);
    data.fyInv[0] = data.KInv[0](1,1);
    data.cxInv[0] = data.KInv[0](0,2);
    data.cyInv[0] = data.KInv[0](1,2);
    
    data.timestamp = timestamp;
    
    data.hasIDepthBeenSet = false;
    depthHasBeenUpdatedFlag = false;

    
    for (int level = 0; level < PYRAMID_LEVELS; ++ level)
    {
        data.width[level] = width >> level;
        data.height[level] = height >> level;
        
        data.imageValid[level] = false;
        data.gradientsValid[level] = false;
        data.maxGradientsValid[level] = false;
        data.idepthValid[level] = false;
        data.idepthVarValid[level] = false;
        
        data.image[level] = 0;
        data.gradients[level] = 0;
        data.maxGradients[level] = 0;
        data.idepth[level] = 0;
        data.idepthVar[level] = 0;
        data.reActivationDataValid = false;
        
        // 		data.refIDValid[level] = false;
        
        if (level > 0)
        {
            data.fx[level] = data.fx[level-1] * 0.5;
            data.fy[level] = data.fy[level-1] * 0.5;
            data.cx[level] = (data.cx[0] + 0.5) / ((int)1<<level) - 0.5;
            data.cy[level] = (data.cy[0] + 0.5) / ((int)1<<level) - 0.5;
            
            data.K[level]  << data.fx[level], 0.0, data.cx[level], 0.0, data.fy[level], data.cy[level], 0.0, 0.0, 1.0;	// synthetic
            data.KInv[level] = (data.K[level]).inverse();
            
            data.fxInv[level] = data.KInv[level](0,0);
            data.fyInv[level] = data.KInv[level](1,1);
            data.cxInv[level] = data.KInv[level](0,2);
            data.cyInv[level] = data.KInv[level](1,2);
        }
    }
    
    data.validity_reAct = 0;
    data.idepthVar_reAct = 0;
    data.idepth_reAct = 0;
    
    data.refPixelWasGood = 0;
    
}

