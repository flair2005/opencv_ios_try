//
//  KeyFrameGraph.cpp
//  opencv_ios_try
//
//  Created by test on 10/18/16.
//  Copyright © 2016 test. All rights reserved.
//

#include "KeyFrameGraph.hpp"
#include "opencv2/opencv.hpp"
#include "FramePoseStruct.hpp"
#include "Frame.hpp"

KeyFrameGraph::KeyFrameGraph()
{

}

KeyFrameGraph::~KeyFrameGraph()
{
    for(FramePoseStruct* p : allFramePoses)
        delete p;
}


void KeyFrameGraph::addFrame(Frame* frame)
{
    FramePoseStruct* pose = frame->pose;
    allFramePoses.push_back(pose);
}

void KeyFrameGraph::dumpMap(std::string folder)
{

}
