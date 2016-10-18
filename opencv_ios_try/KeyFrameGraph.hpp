//
//  KeyFrameGraph.hpp
//  opencv_ios_try
//
//  Created by test on 10/18/16.
//  Copyright © 2016 test. All rights reserved.
//

#ifndef KeyFrameGraph_hpp
#define KeyFrameGraph_hpp

#include <stdio.h>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <unordered_map>
#include <g2o/core/sparse_optimizer.h>
#include <deque>
#include "sim3.hpp"

class Frame;
class FramePoseStruct;

struct KFConstraintStruct
{
    inline KFConstraintStruct()
    {
        firstFrame = secondFrame = 0;
        information.setZero();
        robustKernel = 0;
        
        usage = meanResidual = meanResidualD = meanResidualP = 0;
        reciprocalConsistency = 0;
        
        
        idxInAllEdges = -1;
    }
    
    ~KFConstraintStruct();
    
    Frame* firstFrame;
    Frame* secondFrame;
    Sophus::Sim3d secondToFirst;
    Eigen::Matrix<double, 7, 7> information;
    g2o::RobustKernel* robustKernel;
    
    float usage;
    float meanResidualD;
    float meanResidualP;
    float meanResidual;
    
    float reciprocalConsistency;
    
    int idxInAllEdges;
};

class KeyFrameGraph
{
public:
    /** Constructs an empty pose graph. */
    KeyFrameGraph();
    
    /** Deletes the g2o graph. */
    ~KeyFrameGraph();
    
    /** Adds a new KeyFrame to the graph. */
    void addKeyFrame(Frame* frame);
    
    /** Adds a new Frame to the graph. Doesnt actually keep the frame, but only it's pose-struct. */
    void addFrame(Frame* frame);
    
    void dumpMap(std::string folder);
    
    std::vector< Frame* > keyframesAll;
    
    std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int> > idToKeyFrame;
    
    std::vector< KFConstraintStruct* > edgesAll;
    
    std::vector<FramePoseStruct*> allFramePoses;
    
    std::deque<Frame*> keyframesForRetrack;

};

#endif /* KeyFrameGraph_hpp */
