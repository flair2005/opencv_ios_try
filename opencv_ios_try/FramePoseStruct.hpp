//
//  FramePoseStruct.hpp
//  opencv_ios_try
//
//  Created by test on 10/17/16.
//  Copyright © 2016 test. All rights reserved.
//

#ifndef FramePoseStruct_hpp
#define FramePoseStruct_hpp

#include <stdio.h>
#include "SophusUtil.h"

class Frame;

class FramePoseStruct {
public:
    FramePoseStruct(Frame* frame);
    virtual ~FramePoseStruct();
    
    FramePoseStruct* trackingParent;
    
    // set initially as tracking result (then it's a SE(3)),
    // and is changed only once, when the frame becomes a KF (->rescale).
    Sophus::Sim3 thisToParent_raw;

    int frameID;
    Frame* frame;
    
    
    // whether this poseStruct is registered in the Graph. if true MEMORY WILL BE HANDLED BY GRAPH
    bool isRegisteredToGraph;
    
    // whether pose is optimized (true only for KF, after first applyPoseGraphOptResult())
    bool isOptimized;
    
    // true as soon as the vertex is added to the g2o graph.
    bool isInGraph;
    
    void setPoseGraphOptResult(Sim3 camToWorld);
    void applyPoseGraphOptResult();
    Sim3 getCamToWorld(int recursionDepth = 0);
    void invalidateCache();
private:
    int cacheValidFor;
    static int cacheValidCounter;
    
    // absolute position (camToWorld).
    // can change when optimization offset is merged.
    Sim3 camToWorld;
    
    // new, optimized absolute position. is added on mergeOptimization.
    Sim3 camToWorld_new;
    
    // whether camToWorld_new is newer than camToWorld
    bool hasUnmergedPose;

};

#endif /* FramePoseStruct_hpp */
