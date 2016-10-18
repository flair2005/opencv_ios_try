
#ifndef SE3Tracker_hpp
#define SE3Tracker_hpp

#include <stdio.h>
#include "setting.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "SophusUtil.h"
#include "LGSX.h"

class Frame;
class TrackingReference;
class SE3Tracker
{
public:
    int width, height;
    Eigen::Matrix3f K, KInv;
    float fx,fy,cx,cy;
    float fxi,fyi,cxi,cyi;
    DenseDepthTrackerSettings settings;
    
    SE3Tracker(int w, int h, Eigen::Matrix3f K);
    SE3Tracker(const SE3Tracker&) = delete;
    SE3Tracker& operator=(const SE3Tracker&) = delete;
    ~SE3Tracker();
    SE3 trackFrame(TrackingReference* reference, Frame* frame, const SE3& frameToReference_initialEstimate);
    SE3 trackFrameOnPermaref(Frame* reference, Frame* frame, SE3 referenceToFrame);
    float checkPermaRefOverlap(Frame* reference, SE3 referenceToFrame);
    
    float pointUsage;
    float lastGoodCount;
    float lastMeanRes;
    float lastBadCount;
    float lastResidual;
    float affineEstimation_a;
    float affineEstimation_b;
    bool diverged;
    bool trackingWasGood;
private:
    float* buf_warped_residual;
    float* buf_warped_dx;
    float* buf_warped_dy;
    float* buf_warped_x;
    float* buf_warped_y;
    float* buf_warped_z;
    float* buf_d;
    float* buf_idepthVar;
    float* buf_weight_p;
    int buf_warped_size;

    float calcResidualAndBuffers(
                                 const Eigen::Vector3f* refPoint,
                                 const Eigen::Vector2f* refColVar,
                                 int* idxBuf,
                                 int refNum,
                                 Frame* frame,
                                 const Sophus::SE3f& referenceToFrame,
                                 int level,
                                 bool plotResidual = false);
    float calcWeightsAndResidual(const Sophus::SE3f& referenceToFrame);
    void calculateWarpUpdate(LGS6 &ls);
    
    int iterationNumber;
    float affineEstimation_a_lastIt;
    float affineEstimation_b_lastIt;
};

#endif /* SE3Tracker_hpp */
