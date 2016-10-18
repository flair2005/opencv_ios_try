
#include "DepthMap.hpp"
#include "DepthMapPixelHypothesis.hpp"
#include "Frame.hpp"

DepthMap::DepthMap(int w, int h, const Eigen::Matrix3f& K)
{
    width = w;
    height = h;
    
    activeKeyFrame = 0;
    otherDepthMap = new DepthMapPixelHypothesis[width*height];
    currentDepthMap = new DepthMapPixelHypothesis[width*height];
    
    validityIntegralBuffer = (int*)Eigen::internal::aligned_malloc(width*height*sizeof(int));
    
    this->K = K;
    fx = K(0,0);
    fy = K(1,1);
    cx = K(0,2);
    cy = K(1,2);
    
    KInv = K.inverse();
    fxi = KInv(0,0);
    fyi = KInv(1,1);
    cxi = KInv(0,2);
    cyi = KInv(1,2);
    reset();
}

DepthMap::~DepthMap()
{
    delete[] otherDepthMap;
    delete[] currentDepthMap;
    
    Eigen::internal::aligned_free((void*)validityIntegralBuffer);
}


void DepthMap::reset()
{
    for(DepthMapPixelHypothesis* pt = otherDepthMap+width*height-1; pt >= otherDepthMap; pt--)
        pt->isValid = false;
    for(DepthMapPixelHypothesis* pt = currentDepthMap+width*height-1; pt >= currentDepthMap; pt--)
        pt->isValid = false;
}

void DepthMap::initializeRandomly(Frame* new_frame)
{
    activeKeyFrame = new_frame;
    activeKeyFrameImageData = activeKeyFrame->image(0);
    
    const float* maxGradients = new_frame->maxGradients();
    
    for(int y=1;y<height-1;y++)
    {
        for(int x=1;x<width-1;x++)
        {
            if(maxGradients[x+y*width] > MIN_ABS_GRAD_CREATE)
            {
                float idepth = 0.5f + 1.0f * ((rand() % 100001) / 100000.0f);
                currentDepthMap[x+y*width] = DepthMapPixelHypothesis(
                                                                     idepth,
                                                                     idepth,
                                                                     VAR_RANDOM_INIT_INITIAL,
                                                                     VAR_RANDOM_INIT_INITIAL,
                                                                     20);
            }
            else
            {
                currentDepthMap[x+y*width].isValid = false;
                currentDepthMap[x+y*width].blacklisted = 0;
            }
        }
    }
    
    
    activeKeyFrame->setDepth(currentDepthMap);
}

