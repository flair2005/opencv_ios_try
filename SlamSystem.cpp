#include "SlamSystem.hpp"
#include "Frame.hpp"

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
    
    keyFrameGraph->idToKeyFrameMutex.lock();
    keyFrameGraph->idToKeyFrame.insert(std::make_pair(currentKeyFrame->id(), currentKeyFrame));
    keyFrameGraph->idToKeyFrameMutex.unlock();
}

void SlamSystem::trackFrame(unsigned char* image, unsigned int frameID, bool blockUntilMapped, double timestamp)
{
    std::shared_ptr<Frame> trackingNewFrame(new Frame(frameID, width, height, K, timestamp, image));
}
