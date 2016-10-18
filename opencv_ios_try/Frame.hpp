#ifndef Frame_hpp
#define Frame_hpp

#include <stdio.h>
#include <Eigen/Dense> 
#include "setting.hpp"

class FramePoseStruct;

class Frame
{
public:
    Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image);
    FramePoseStruct* pose;
    // Accessors
    /** Returns the unique frame id. */
    inline int id() const;
    
    /** Returns the frame's image width. */
    inline int width(int level = 0) const;
    /** Returns the frame's image height. */
    inline int height(int level = 0) const;
    
    /** Returns the frame's intrinsics matrix. */
    inline const Eigen::Matrix3f& K(int level = 0) const;
    /** Returns the frame's inverse intrinsics matrix. */
    inline const Eigen::Matrix3f& KInv(int level = 0) const;
    /** Returns K(0, 0). */
    inline float fx(int level = 0) const;
    /** Returns K(1, 1). */
    inline float fy(int level = 0) const;
    /** Returns K(0, 2). */
    inline float cx(int level = 0) const;
    /** Returns K(1, 2). */
    inline float cy(int level = 0) const;
    /** Returns KInv(0, 0). */
    inline float fxInv(int level = 0) const;
    /** Returns KInv(1, 1). */
    inline float fyInv(int level = 0) const;
    /** Returns KInv(0, 2). */
    inline float cxInv(int level = 0) const;
    /** Returns KInv(1, 2). */
    inline float cyInv(int level = 0) const;
    
    /** Returns the frame's recording timestamp. */
    inline double timestamp() const;
    
    
    bool depthHasBeenUpdatedFlag;

private:
    void initialize(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp);
    struct Data
    {
        int id;
        
        int width[PYRAMID_LEVELS], height[PYRAMID_LEVELS];
        
        Eigen::Matrix3f K[PYRAMID_LEVELS], KInv[PYRAMID_LEVELS];
        float fx[PYRAMID_LEVELS], fy[PYRAMID_LEVELS], cx[PYRAMID_LEVELS], cy[PYRAMID_LEVELS];
        float fxInv[PYRAMID_LEVELS], fyInv[PYRAMID_LEVELS], cxInv[PYRAMID_LEVELS], cyInv[PYRAMID_LEVELS];
        
        double timestamp;
        
        
        float* image[PYRAMID_LEVELS];
        bool imageValid[PYRAMID_LEVELS];
        
        Eigen::Vector4f* gradients[PYRAMID_LEVELS];
        bool gradientsValid[PYRAMID_LEVELS];
        
        float* maxGradients[PYRAMID_LEVELS];
        bool maxGradientsValid[PYRAMID_LEVELS];
        
        
        bool hasIDepthBeenSet;
        
        // negative depthvalues are actually allowed, so setting this to -1 does NOT invalidate the pixel's depth.
        // a pixel is valid iff idepthVar[i] > 0.
        float* idepth[PYRAMID_LEVELS];
        bool idepthValid[PYRAMID_LEVELS];
        
        // MUST contain -1 for invalid pixel (that dont have depth)!!
        float* idepthVar[PYRAMID_LEVELS];
        bool idepthVarValid[PYRAMID_LEVELS];
        
        // data needed for re-activating the frame. theoretically, this is all data the
        // frame contains.
        unsigned char* validity_reAct;
        float* idepth_reAct;
        float* idepthVar_reAct;
        bool reActivationDataValid;
        
        
        // data from initial tracking, indicating which pixels in the reference frame ware good or not.
        // deleted as soon as frame is used for mapping.
        bool* refPixelWasGood;
    };
    Data data;
};

inline int Frame::id() const
{
    return data.id;
}

inline int Frame::width(int level) const
{
    return data.width[level];
}

inline int Frame::height(int level) const
{
    return data.height[level];
}

inline const Eigen::Matrix3f& Frame::K(int level) const
{
    return data.K[level];
}
inline const Eigen::Matrix3f& Frame::KInv(int level) const
{
    return data.KInv[level];
}
inline float Frame::fx(int level) const
{
    return data.fx[level];
}
inline float Frame::fy(int level) const
{
    return data.fy[level];
}
inline float Frame::cx(int level) const
{
    return data.cx[level];
}
inline float Frame::cy(int level) const
{
    return data.cy[level];
}
inline float Frame::fxInv(int level) const
{
    return data.fxInv[level];
}
inline float Frame::fyInv(int level) const
{
    return data.fyInv[level];
}
inline float Frame::cxInv(int level) const
{
    return data.cxInv[level];
}
inline float Frame::cyInv(int level) const
{
    return data.cyInv[level];
}

inline double Frame::timestamp() const
{
    return data.timestamp;
}


#endif /* Frame_hpp */
