//
//  TrackingReference.cpp
//  opencv_ios_try
//
//  Created by test on 10/18/16.
//  Copyright © 2016 test. All rights reserved.
//

#include "TrackingReference.hpp"
#include "Frame.hpp"

TrackingReference::TrackingReference()
{
    frameID=-1;
    keyframe = 0;
    wh_allocated = 0;
    for (int level = 0; level < PYRAMID_LEVELS; ++ level)
    {
        posData[level] = nullptr;
        gradData[level] = nullptr;
        colorAndVarData[level] = nullptr;
        pointPosInXYGrid[level] = nullptr;
        numData[level] = 0;
    }
}
void TrackingReference::releaseAll()
{
    for (int level = 0; level < PYRAMID_LEVELS; ++ level)
    {
        if(posData[level] != nullptr) delete[] posData[level];
        if(gradData[level] != nullptr) delete[] gradData[level];
        if(colorAndVarData[level] != nullptr) delete[] colorAndVarData[level];
        if(pointPosInXYGrid[level] != nullptr)
            Eigen::internal::aligned_free((void*)pointPosInXYGrid[level]);
        numData[level] = 0;
    }
    wh_allocated = 0;
}
void TrackingReference::clearAll()
{
    for (int level = 0; level < PYRAMID_LEVELS; ++ level)
        numData[level] = 0;
}
TrackingReference::~TrackingReference()
{
    invalidate();
    releaseAll();
}

void TrackingReference::importFrame(Frame* sourceKF)
{
    keyframe = sourceKF;
    frameID=keyframe->id();
    
    
    // reset allocation if dimensions differ (shouldnt happen usually)
    if(sourceKF->width(0) * sourceKF->height(0) != wh_allocated)
    {
        releaseAll();
        wh_allocated = sourceKF->width(0) * sourceKF->height(0);
    }
    clearAll();
}

void TrackingReference::invalidate()
{
    keyframe = 0;
}

void TrackingReference::makePointCloud(int level)
{
    assert(keyframe != 0);
    
    if(numData[level] > 0)
        return;	// already exists.
    
    int w = keyframe->width(level);
    int h = keyframe->height(level);
    
    float fxInvLevel = keyframe->fxInv(level);
    float fyInvLevel = keyframe->fyInv(level);
    float cxInvLevel = keyframe->cxInv(level);
    float cyInvLevel = keyframe->cyInv(level);
    
    const float* pyrIdepthSource = keyframe->idepth(level);
    const float* pyrIdepthVarSource = keyframe->idepthVar(level);
    const float* pyrColorSource = keyframe->image(level);
    const Eigen::Vector4f* pyrGradSource = keyframe->gradients(level);
    
    if(posData[level] == nullptr) posData[level] = new Eigen::Vector3f[w*h];
    if(pointPosInXYGrid[level] == nullptr)
        pointPosInXYGrid[level] = (int*)Eigen::internal::aligned_malloc(w*h*sizeof(int));;
    if(gradData[level] == nullptr) gradData[level] = new Eigen::Vector2f[w*h];
    if(colorAndVarData[level] == nullptr) colorAndVarData[level] = new Eigen::Vector2f[w*h];
    
    Eigen::Vector3f* posDataPT = posData[level];
    int* idxPT = pointPosInXYGrid[level];
    Eigen::Vector2f* gradDataPT = gradData[level];
    Eigen::Vector2f* colorAndVarDataPT = colorAndVarData[level];
    
    for(int x=1; x<w-1; x++)
        for(int y=1; y<h-1; y++)
        {
            int idx = x + y*w;
            
            if(pyrIdepthVarSource[idx] <= 0 || pyrIdepthSource[idx] == 0) continue;
            
            *posDataPT = (1.0f / pyrIdepthSource[idx]) * Eigen::Vector3f(fxInvLevel*x+cxInvLevel,fyInvLevel*y+cyInvLevel,1);
            *gradDataPT = pyrGradSource[idx].head<2>();
            *colorAndVarDataPT = Eigen::Vector2f(pyrColorSource[idx], pyrIdepthVarSource[idx]);
            *idxPT = idx;
            
            posDataPT++;
            gradDataPT++;
            colorAndVarDataPT++;
            idxPT++;
        }
    
    numData[level] = posDataPT - posData[level];
}
