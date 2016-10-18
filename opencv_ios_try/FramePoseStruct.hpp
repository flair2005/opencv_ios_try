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
#include "sim3.hpp"

class Frame;

class FramePoseStruct {
public:
    FramePoseStruct(Frame* frame);
    
    FramePoseStruct* trackingParent;
    
    // set initially as tracking result (then it's a SE(3)),
    // and is changed only once, when the frame becomes a KF (->rescale).
    Sophus::Sim3 thisToParent_raw;
    
    
    int frameID;
    Frame* frame;
};

#endif /* FramePoseStruct_hpp */
