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
