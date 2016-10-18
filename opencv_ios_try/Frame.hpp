#ifndef Frame_hpp
#define Frame_hpp

#include <stdio.h>
#include <Eigen/Dense> 
#include "setting.hpp"
#include "FrameMemory.hpp"

class FramePoseStruct;
class DepthMapPixelHypothesis;

class Frame
{
public:
    Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image);
    ~Frame();
    void setDepth(const DepthMapPixelHypothesis* newDepth);
    
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
    
    inline float* image(int level = 0);
    inline const Eigen::Vector4f* gradients(int level = 0);
    inline const float* maxGradients(int level = 0);
    inline bool hasIDepthBeenSet() const;
    inline const float* idepth(int level = 0);
    inline const float* idepthVar(int level = 0);
    inline const unsigned char* validity_reAct();
    inline const float* idepth_reAct();
    inline const float* idepthVar_reAct();
    
    inline bool* refPixelWasGood();
    inline bool* refPixelWasGoodNoCreate();
    inline void clear_refPixelWasGood();
    
    enum DataFlags
    {
        IMAGE			= 1<<0,
        GRADIENTS		= 1<<1,
        MAX_GRADIENTS	= 1<<2,
        IDEPTH			= 1<<3,
        IDEPTH_VAR		= 1<<4,
        REF_ID			= 1<<5,
        
        ALL = IMAGE | GRADIENTS | MAX_GRADIENTS | IDEPTH | IDEPTH_VAR | REF_ID
    };
    
    Eigen::Vector3f* permaRef_posData;	// (x,y,z)
    Eigen::Vector2f* permaRef_colorAndVarData;	// (I, Var)
    int permaRefNumPts;
    
    bool depthHasBeenUpdatedFlag;
    bool isActive;
private:
    void initialize(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp);
    void require(int dataFlags, int level = 0);
    void release(int dataFlags, bool pyramidsOnly, bool invalidateOnly);
    
    void buildImage(int level);
    void releaseImage(int level);
    
    void buildGradients(int level);
    void releaseGradients(int level);
    
    void buildMaxGradients(int level);
    void releaseMaxGradients(int level);
    
    void buildIDepthAndIDepthVar(int level);
    void releaseIDepth(int level);
    void releaseIDepthVar(int level);
    
    void printfAssert(const char* message) const;
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

inline float* Frame::image(int level)
{
    if (! data.imageValid[level])
        require(IMAGE, level);
    return data.image[level];
}
inline const Eigen::Vector4f* Frame::gradients(int level)
{
    if (! data.gradientsValid[level])
        require(GRADIENTS, level);
    return data.gradients[level];
}
inline const float* Frame::maxGradients(int level)
{
    if (! data.maxGradientsValid[level])
        require(MAX_GRADIENTS, level);
    return data.maxGradients[level];
}
inline bool Frame::hasIDepthBeenSet() const
{
    return data.hasIDepthBeenSet;
}
inline const float* Frame::idepth(int level)
{
    if (! data.hasIDepthBeenSet)
    {
        printfAssert("Frame::idepth(): idepth has not been set yet!");
        return nullptr;
    }
    if (! data.idepthValid[level])
        require(IDEPTH, level);
    return data.idepth[level];
}
inline const unsigned char* Frame::validity_reAct()
{
    if( !data.reActivationDataValid)
        return 0;
    return data.validity_reAct;
}
inline const float* Frame::idepth_reAct()
{
    if( !data.reActivationDataValid)
        return 0;
    return data.idepth_reAct;
}
inline const float* Frame::idepthVar_reAct()
{
    if( !data.reActivationDataValid)
        return 0;
    return data.idepthVar_reAct;
}
inline const float* Frame::idepthVar(int level)
{
    if (! data.hasIDepthBeenSet)
    {
        printfAssert("Frame::idepth(): idepth has not been set yet!");
        return nullptr;
    }
    if (! data.idepthVarValid[level])
        require(IDEPTH_VAR, level);
    return data.idepthVar[level];
}


inline bool* Frame::refPixelWasGood()
{
    if( data.refPixelWasGood == 0)
    {
        
        if(data.refPixelWasGood == 0)
        {
            int width = data.width[SE3TRACKING_MIN_LEVEL];
            int height = data.height[SE3TRACKING_MIN_LEVEL];
            data.refPixelWasGood = (bool*)FrameMemory::getInstance().getBuffer(sizeof(bool) * width * height);
            
            memset(data.refPixelWasGood, 0xFFFFFFFF, sizeof(bool) * (width * height));
        }
    }
    return data.refPixelWasGood;
}


inline bool* Frame::refPixelWasGoodNoCreate()
{
    return data.refPixelWasGood;
}

inline void Frame::clear_refPixelWasGood()
{
    FrameMemory::getInstance().returnBuffer(reinterpret_cast<float*>(data.refPixelWasGood));
    data.refPixelWasGood=0;
}


#endif /* Frame_hpp */
