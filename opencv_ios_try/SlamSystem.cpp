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
    trackingIsGood = true;
    isInited = false;
    
    map =  new DepthMap(w,h,K);
    haveUnmergedOptimizationOffset = false;
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
        trackingIsGood = false;
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
    
    updateKeyframe();
    
//    if(currentKeyFrame == 0)
//        return;
//    
//    if(currentKeyFrame->idxInKeyframes < 0)
//    {
//        if(currentKeyFrame->numMappedOnThisTotal >= MIN_NUM_MAPPED){
//            finishCurrentKeyframe();
//        }
//        else{
//            discardCurrentKeyframe();
//        }
//        map->invalidate();
//        printf("Finished KF %d as Mapping got disabled!\n",currentKeyFrame->id());
//        
//        changeKeyframe(true, true, 1.0f);
//    }
//    
//    mergeOptimizationOffset();
//    
//    
//    // set mappingFrame
//    if(trackingIsGood)
//    {
//        if (createNewKeyFrame)
//        {
//            finishCurrentKeyframe();
//            changeKeyframe(false, true, 1.0f);
//        }
//        else
//        {
//            bool didSomething = updateKeyframe();
//            if(!didSomething)
//                return;
//        }
//        
//        return;
//    }
//    else
//    {
//        // invalidate map if it was valid.
//        if(map->isValid())
//        {
//            if(currentKeyFrame->numMappedOnThisTotal >= MIN_NUM_MAPPED)
//                finishCurrentKeyframe();
//            else
//                discardCurrentKeyframe();
//            
//            map->invalidate();
//        }
//        return;
//    }
}

void SlamSystem::mergeOptimizationOffset()
{
    if(haveUnmergedOptimizationOffset)
    {
        for(unsigned int i=0;i<keyFrameGraph->keyframesAll.size(); i++)
            keyFrameGraph->keyframesAll[i]->pose->applyPoseGraphOptResult();
        haveUnmergedOptimizationOffset = false;
    }

}

void SlamSystem::finishCurrentKeyframe()
{
    map->finalizeKeyFrame();
    
//    mappingTrackingReference->importFrame(currentKeyFrame.get());
//    currentKeyFrame->setPermaRef(mappingTrackingReference);
//    mappingTrackingReference->invalidate();
//    
//    if(currentKeyFrame->idxInKeyframes < 0)
//    {
//        currentKeyFrame->idxInKeyframes = keyFrameGraph->keyframesAll.size();
//        keyFrameGraph->keyframesAll.push_back(currentKeyFrame.get());
//        keyFrameGraph->totalPoints += currentKeyFrame->numPoints;
//        keyFrameGraph->totalVertices ++;
//        
//        newKeyFrames.push_back(currentKeyFrame.get());
//    }
}

void SlamSystem::discardCurrentKeyframe()
{
    if(currentKeyFrame->idxInKeyframes >= 0)
    {
        printf("WARNING: trying to discard a KF that has already been added to the graph... finalizing instead.\n");
        finishCurrentKeyframe();
        return;
    }

    map->invalidate();
    for(FramePoseStruct* p : keyFrameGraph->allFramePoses)
    {
        if(p->trackingParent != 0 && p->trackingParent->frameID == currentKeyFrame->id())
            p->trackingParent = 0;
    }
    keyFrameGraph->idToKeyFrame.erase(currentKeyFrame->id());
}

void SlamSystem::changeKeyframe(bool noCreate, bool force, float maxScore)
{
//    Frame* newReferenceKF=0;
//    std::shared_ptr<Frame> newKeyframeCandidate = latestTrackedFrame;
//    if(doKFReActivation)
//    {
//        newReferenceKF = trackableKeyFrameSearch->findRePositionCandidate(newKeyframeCandidate.get(), maxScore);
//    }
//    
//    if(newReferenceKF != 0)
//        loadNewCurrentKeyframe(newReferenceKF);
//    else
//    {
//        if(force)
//        {
//            if(noCreate)
//            {
//                trackingIsGood = false;
//                nextRelocIdx = -1;
//                printf("mapping is disabled & moved outside of known map. Starting Relocalizer!\n");
//            }
//            else
//                createNewCurrentKeyframe(newKeyframeCandidate);
//        }
//    }
//    createNewKeyFrame = false;
}

bool SlamSystem::updateKeyframe()
{
    std::shared_ptr<Frame> reference = nullptr;
    std::deque< std::shared_ptr<Frame> > references;

    // remove frames that have a different tracking parent.
    while(unmappedTrackedFrames.size() > 0 &&
          (!unmappedTrackedFrames.front()->hasTrackingParent() ||
           unmappedTrackedFrames.front()->getTrackingParent() != currentKeyFrame.get()))
    {
        unmappedTrackedFrames.front()->clear_refPixelWasGood();
        unmappedTrackedFrames.pop_front();
    }
    
    // clone list
    if(unmappedTrackedFrames.size() > 0)
    {
        for(unsigned int i=0;i<unmappedTrackedFrames.size(); i++)
            references.push_back(unmappedTrackedFrames[i]);
        
        std::shared_ptr<Frame> popped = unmappedTrackedFrames.front();
        unmappedTrackedFrames.pop_front();
        
        map->updateKeyframe(references);
        
        popped->clear_refPixelWasGood();
        references.clear();
    }
    else
    {
        return false;
    }
    
    return true;
}
