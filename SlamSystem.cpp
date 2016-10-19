#include "SlamSystem.hpp"
#include "Frame.hpp"
#include "DepthMap.hpp"
#include "KeyFrameGraph.hpp"
#include "TrackingReference.hpp"
#include "SophusUtil.h"
#include "FramePoseStruct.hpp"
#include "SE3Tracker.hpp"


SlamSystem::SlamSystem(int w, int h, Eigen::Matrix3f K)
{
    if(w%16 != 0 || h%16!=0)
    {
        printf("image dimensions must be multiples of 16! Please crop your images / video accordingly.\n");
        assert(false);
    }
    
    this->width = w;
    this->height = h;
    this->K = K;
    
    createNewKeyFrame = false;
    currentKeyFrame =  nullptr;
    keyFrameGraph = new KeyFrameGraph();
    
    trackingReference = new TrackingReference();
    
    isInited = false;
    
    map =  new DepthMap(w,h,K);
    
    tracker = new SE3Tracker(w,h,K);
    for (int level = 4; level < PYRAMID_LEVELS; ++level)
        tracker->settings.maxItsPerLvl[level] = 0;
}

SlamSystem::~SlamSystem()
{
    latestTrackedFrame.reset();
    currentKeyFrame.reset();
}

void SlamSystem::randomInit(unsigned char* image, double timeStamp, int id)
{
    currentKeyFrame.reset(new Frame(id, width, height, K, timeStamp, image));
    map->initializeRandomly(currentKeyFrame.get());
    keyFrameGraph->addFrame(currentKeyFrame.get());
    keyFrameGraph->idToKeyFrame[currentKeyFrame->id()] = currentKeyFrame;
}

void SlamSystem::trackFrame(unsigned char* image, unsigned int frameID, bool blockUntilMapped, double timestamp)
{
    std::shared_ptr<Frame> trackingNewFrame(new Frame(frameID, width, height, K, timestamp, image));
    
    bool my_createNewKeyframe = createNewKeyFrame;	// pre-save here, to make decision afterwards.
    if(trackingReference->keyframe != currentKeyFrame.get() || currentKeyFrame->depthHasBeenUpdatedFlag)
    {
        trackingReference->importFrame(currentKeyFrame.get());
        currentKeyFrame->depthHasBeenUpdatedFlag = false;
    }
    
    FramePoseStruct* trackingReferencePose = trackingReference->keyframe->pose;
    
    SE3 frameToReference_initialEstimate = se3FromSim3(
                                                       trackingReferencePose->getCamToWorld().inverse() * keyFrameGraph->allFramePoses.back()->getCamToWorld());

    SE3 newRefToFrame_poseUpdate = tracker->trackFrame(
                                                       trackingReference,
                                                       trackingNewFrame.get(),
                                                       frameToReference_initialEstimate);

    if(manualTrackingLossIndicated || tracker->diverged || (keyFrameGraph->keyframesAll.size() > INITIALIZATION_PHASE_COUNT && !tracker->trackingWasGood))
    {
        trackingReference->invalidate();
        
        manualTrackingLossIndicated = false;
        return;
    }
    
    keyFrameGraph->addFrame(trackingNewFrame.get());

    latestTrackedFrame = trackingNewFrame;
    if (!my_createNewKeyframe && currentKeyFrame->numMappedOnThisTotal > MIN_NUM_MAPPED)
    {
        Sophus::Vector3d dist = newRefToFrame_poseUpdate.translation() * currentKeyFrame->meanIdepth;
        float minVal = fmin(0.2f + keyFrameGraph->keyframesAll.size() * 0.8f / INITIALIZATION_PHASE_COUNT,1.0f);
        
        if(keyFrameGraph->keyframesAll.size() < INITIALIZATION_PHASE_COUNT)	minVal *= 0.7;
        
//        lastTrackingClosenessScore = trackableKeyFrameSearch->getRefFrameScore(dist.dot(dist), tracker->pointUsage);
//        
//        if (lastTrackingClosenessScore > minVal)
//        {
//            createNewKeyFrame = true;
//        }
    }
    
    if(unmappedTrackedFrames.size() < 50 || (unmappedTrackedFrames.size() < 100 && trackingNewFrame->getTrackingParent()->numMappedOnThisTotal < 10))
        unmappedTrackedFrames.push_back(trackingNewFrame);
}
