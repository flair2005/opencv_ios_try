//
//  FramePoseStruct.cpp
//  opencv_ios_try
//
//  Created by test on 10/17/16.
//  Copyright © 2016 test. All rights reserved.
//

#include "FramePoseStruct.hpp"
#include "Frame.hpp"

FramePoseStruct::FramePoseStruct(Frame* frame)
{
    thisToParent_raw = Sophus::Sim3();
    this->frame = frame;
    frameID = frame->id();
    trackingParent = 0;
}

int FramePoseStruct::cacheValidCounter = 0;

int privateFramePoseStructAllocCount = 0;

FramePoseStruct::~FramePoseStruct()
{
    privateFramePoseStructAllocCount--;
}

void FramePoseStruct::setPoseGraphOptResult(Sim3 camToWorld)
{
    if(!isInGraph)
        return;
    
    
    camToWorld_new = camToWorld;
    hasUnmergedPose = true;
}

void FramePoseStruct::applyPoseGraphOptResult()
{
    if(!hasUnmergedPose)
        return;
    
    
    camToWorld = camToWorld_new;
    isOptimized = true;
    hasUnmergedPose = false;
    cacheValidCounter++;
}
void FramePoseStruct::invalidateCache()
{
    cacheValidFor = -1;
}

Sim3 FramePoseStruct::getCamToWorld(int recursionDepth)
{
    // prevent stack overflow
    assert(recursionDepth < 5000);
    
    // if the node is in the graph, it's absolute pose is only changed by optimization.
    if(isOptimized) return camToWorld;
    
    
    // return chached pose, if still valid.
    if(cacheValidFor == cacheValidCounter)
        return camToWorld;
    
    // return id if there is no parent (very first frame)
    if(trackingParent == nullptr)
        return camToWorld = Sim3();
    
    // abs. pose is computed from the parent's abs. pose, and cached.
    cacheValidFor = cacheValidCounter;
    
    return camToWorld = trackingParent->getCamToWorld(recursionDepth+1) * thisToParent_raw;
}
