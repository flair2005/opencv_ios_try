//
//  TrackingReference.hpp
//  opencv_ios_try
//
//  Created by test on 10/18/16.
//  Copyright © 2016 test. All rights reserved.
//

#ifndef TrackingReference_hpp
#define TrackingReference_hpp

#include <stdio.h>
#include <Eigen/Dense>
#include "setting.hpp"

class Frame;
class TrackingReference
{
public:
    
    /** Creates an empty TrackingReference with optional preallocation per level. */
    TrackingReference();
    ~TrackingReference();
    void importFrame(Frame* source);
    
    Frame* keyframe;
    int frameID;
    
    void makePointCloud(int level);
    void clearAll();
    void invalidate();
    Eigen::Vector3f* posData[PYRAMID_LEVELS];	// (x,y,z)
    Eigen::Vector2f* gradData[PYRAMID_LEVELS];	// (dx, dy)
    Eigen::Vector2f* colorAndVarData[PYRAMID_LEVELS];	// (I, Var)
    int* pointPosInXYGrid[PYRAMID_LEVELS];	// x + y*width
    int numData[PYRAMID_LEVELS];
    
private:
    int wh_allocated;
    void releaseAll();
};

#endif /* TrackingReference_hpp */
