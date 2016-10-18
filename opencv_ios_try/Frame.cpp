//
//  Frame.cpp
//  opencv_ios_try
//
//  Created by test on 10/17/16.
//  Copyright © 2016 test. All rights reserved.
//

#include "Frame.hpp"
#include "FramePoseStruct.hpp"
#include "FrameMemory.hpp"
#include "setting.hpp"
#include "DepthMapPixelHypothesis.hpp"

int privateFrameAllocCount = 0;

Frame::Frame(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp, const unsigned char* image)
{
    initialize(id, width, height, K, timestamp);
    
    data.image[0] = FrameMemory::getInstance().getFloatBuffer(data.width[0]*data.height[0]);
    
    float* maxPt = data.image[0] + data.width[0]*data.height[0];
    
    for(float* pt = data.image[0]; pt < maxPt; pt++)
    {
        *pt = *image;
        image++;
    }
    data.imageValid[0] = true;
    privateFrameAllocCount++;
}

Frame::~Frame()
{
    FrameMemory::getInstance().deactivateFrame(this);
    
    pose->frame = 0;
    
    for (int level = 0; level < PYRAMID_LEVELS; ++ level)
    {
        FrameMemory::getInstance().returnBuffer(data.image[level]);
        FrameMemory::getInstance().returnBuffer(reinterpret_cast<float*>(data.gradients[level]));
        FrameMemory::getInstance().returnBuffer(data.maxGradients[level]);
        FrameMemory::getInstance().returnBuffer(data.idepth[level]);
        FrameMemory::getInstance().returnBuffer(data.idepthVar[level]);
    }
    
    FrameMemory::getInstance().returnBuffer((float*)data.validity_reAct);
    FrameMemory::getInstance().returnBuffer(data.idepth_reAct);
    FrameMemory::getInstance().returnBuffer(data.idepthVar_reAct);
    
    privateFrameAllocCount--;
}

void Frame::setDepth(const DepthMapPixelHypothesis* newDepth)
{
    if(data.idepth[0] == 0)
        data.idepth[0] = FrameMemory::getInstance().getFloatBuffer(data.width[0]*data.height[0]);
    if(data.idepthVar[0] == 0)
        data.idepthVar[0] = FrameMemory::getInstance().getFloatBuffer(data.width[0]*data.height[0]);
    
    float* pyrIDepth = data.idepth[0];
    float* pyrIDepthVar = data.idepthVar[0];
    float* pyrIDepthMax = pyrIDepth + (data.width[0]*data.height[0]);
    
    float sumIdepth=0;
    int numIdepth=0;
    
    for (; pyrIDepth < pyrIDepthMax; ++ pyrIDepth, ++ pyrIDepthVar, ++ newDepth) //, ++ pyrRefID)
    {
        if (newDepth->isValid && newDepth->idepth_smoothed >= -0.05)
        {
            *pyrIDepth = newDepth->idepth_smoothed;
            *pyrIDepthVar = newDepth->idepth_var_smoothed;
            
            numIdepth++;
            sumIdepth += newDepth->idepth_smoothed;
        }
        else
        {
            *pyrIDepth = -1;
            *pyrIDepthVar = -1;
        }
    }
    
    data.idepthValid[0] = true;
    data.idepthVarValid[0] = true;
    release(IDEPTH | IDEPTH_VAR, true, true);
    data.hasIDepthBeenSet = true;
    depthHasBeenUpdatedFlag = true;
}

void Frame::release(int dataFlags, bool pyramidsOnly, bool invalidateOnly)
{
    for (int level = (pyramidsOnly ? 1 : 0); level < PYRAMID_LEVELS; ++ level)
    {
        if ((dataFlags & IMAGE) && data.imageValid[level])
        {
            data.imageValid[level] = false;
            if(!invalidateOnly)
                releaseImage(level);
        }
        if ((dataFlags & GRADIENTS) && data.gradientsValid[level])
        {
            data.gradientsValid[level] = false;
            if(!invalidateOnly)
                releaseGradients(level);
        }
        if ((dataFlags & MAX_GRADIENTS) && data.maxGradientsValid[level])
        {
            data.maxGradientsValid[level] = false;
            if(!invalidateOnly)
                releaseMaxGradients(level);
        }
        if ((dataFlags & IDEPTH) && data.idepthValid[level])
        {
            data.idepthValid[level] = false;
            if(!invalidateOnly)
                releaseIDepth(level);
        }
        if ((dataFlags & IDEPTH_VAR) && data.idepthVarValid[level])
        {
            data.idepthVarValid[level] = false;
            if(!invalidateOnly)
                releaseIDepthVar(level);
        }
    }
}

void Frame::initialize(int id, int width, int height, const Eigen::Matrix3f& K, double timestamp)
{
    data.id = id;
    
    pose = new FramePoseStruct(this);
    
    data.K[0] = K;
    data.fx[0] = K(0,0);
    data.fy[0] = K(1,1);
    data.cx[0] = K(0,2);
    data.cy[0] = K(1,2);
    
    data.KInv[0] = K.inverse();
    data.fxInv[0] = data.KInv[0](0,0);
    data.fyInv[0] = data.KInv[0](1,1);
    data.cxInv[0] = data.KInv[0](0,2);
    data.cyInv[0] = data.KInv[0](1,2);
    
    data.timestamp = timestamp;
    
    data.hasIDepthBeenSet = false;
    depthHasBeenUpdatedFlag = false;
    
    permaRefNumPts = 0;
    permaRef_colorAndVarData = 0;
    permaRef_posData = 0;

    
    for (int level = 0; level < PYRAMID_LEVELS; ++ level)
    {
        data.width[level] = width >> level;
        data.height[level] = height >> level;
        
        data.imageValid[level] = false;
        data.gradientsValid[level] = false;
        data.maxGradientsValid[level] = false;
        data.idepthValid[level] = false;
        data.idepthVarValid[level] = false;
        
        data.image[level] = 0;
        data.gradients[level] = 0;
        data.maxGradients[level] = 0;
        data.idepth[level] = 0;
        data.idepthVar[level] = 0;
        data.reActivationDataValid = false;
        
        // 		data.refIDValid[level] = false;
        
        if (level > 0)
        {
            data.fx[level] = data.fx[level-1] * 0.5;
            data.fy[level] = data.fy[level-1] * 0.5;
            data.cx[level] = (data.cx[0] + 0.5) / ((int)1<<level) - 0.5;
            data.cy[level] = (data.cy[0] + 0.5) / ((int)1<<level) - 0.5;
            
            data.K[level]  << data.fx[level], 0.0, data.cx[level], 0.0, data.fy[level], data.cy[level], 0.0, 0.0, 1.0;	// synthetic
            data.KInv[level] = (data.K[level]).inverse();
            
            data.fxInv[level] = data.KInv[level](0,0);
            data.fyInv[level] = data.KInv[level](1,1);
            data.cxInv[level] = data.KInv[level](0,2);
            data.cyInv[level] = data.KInv[level](1,2);
        }
    }
    
    data.validity_reAct = 0;
    data.idepthVar_reAct = 0;
    data.idepth_reAct = 0;
    
    data.refPixelWasGood = 0;
    
}

void Frame::require(int dataFlags, int level)
{
    if ((dataFlags & IMAGE) && ! data.imageValid[level])
    {
        buildImage(level);
    }
    if ((dataFlags & GRADIENTS) && ! data.gradientsValid[level])
    {
        buildGradients(level);
    }
    if ((dataFlags & MAX_GRADIENTS) && ! data.maxGradientsValid[level])
    {
        buildMaxGradients(level);
    }
    if (((dataFlags & IDEPTH) && ! data.idepthValid[level])
        || ((dataFlags & IDEPTH_VAR) && ! data.idepthVarValid[level]))
    {
        buildIDepthAndIDepthVar(level);
    }
}


void Frame::buildImage(int level)
{
    if (level == 0)
    {
        printf("Frame::buildImage(0): Loading image from disk is not implemented yet! No-op.\n");
        return;
    }
    
    require(IMAGE, level - 1);
    
    if(data.imageValid[level])
        return;
    
    int width = data.width[level - 1];
    int height = data.height[level - 1];
    const float* source = data.image[level - 1];
    
    if (data.image[level] == 0)
        data.image[level] = FrameMemory::getInstance().getFloatBuffer(data.width[level] * data.height[level]);
    float* dest = data.image[level];
    
    int wh = width*height;
    const float* s;
    for(int y=0;y<wh;y+=width*2)
    {
        for(int x=0;x<width;x+=2)
        {
            s = source + x + y;
            *dest = (s[0] +
                     s[1] +
                     s[width] +
                     s[1+width]) * 0.25f;
            dest++;
        }
    }
    
    data.imageValid[level] = true;
}

void Frame::releaseImage(int level)
{
    if (level == 0)
    {
        printf("Frame::releaseImage(0): Storing image on disk is not supported yet! No-op.\n");
        return;
    }
    FrameMemory::getInstance().returnBuffer(data.image[level]);
    data.image[level] = 0;
}

void Frame::buildGradients(int level)
{
    require(IMAGE, level);
    
    if(data.gradientsValid[level])
        return;
    
    int width = data.width[level];
    int height = data.height[level];
    if(data.gradients[level] == 0)
        data.gradients[level] = (Eigen::Vector4f*)FrameMemory::getInstance().getBuffer(sizeof(Eigen::Vector4f) * width * height);
    const float* img_pt = data.image[level] + width;
    const float* img_pt_max = data.image[level] + width*(height-1);
    Eigen::Vector4f* gradxyii_pt = data.gradients[level] + width;
    
    // in each iteration i need -1,0,p1,mw,pw
    float val_m1 = *(img_pt-1);
    float val_00 = *img_pt;
    float val_p1;
    
    for(; img_pt < img_pt_max; img_pt++, gradxyii_pt++)
    {
        val_p1 = *(img_pt+1);
        
        *((float*)gradxyii_pt) = 0.5f*(val_p1 - val_m1);
        *(((float*)gradxyii_pt)+1) = 0.5f*(*(img_pt+width) - *(img_pt-width));
        *(((float*)gradxyii_pt)+2) = val_00;
        
        val_m1 = val_00;
        val_00 = val_p1;
    }
    
    data.gradientsValid[level] = true;
}

void Frame::releaseGradients(int level)
{
    FrameMemory::getInstance().returnBuffer(reinterpret_cast<float*>(data.gradients[level]));
    data.gradients[level] = 0;
}



void Frame::buildMaxGradients(int level)
{
    require(GRADIENTS, level);
    
    if(data.maxGradientsValid[level]) return;
    
    int width = data.width[level];
    int height = data.height[level];
    if (data.maxGradients[level] == 0)
        data.maxGradients[level] = FrameMemory::getInstance().getFloatBuffer(width * height);
    
    float* maxGradTemp = FrameMemory::getInstance().getFloatBuffer(width * height);
    
    
    // 1. write abs gradients in real data.
    Eigen::Vector4f* gradxyii_pt = data.gradients[level] + width;
    float* maxgrad_pt = data.maxGradients[level] + width;
    float* maxgrad_pt_max = data.maxGradients[level] + width*(height-1);
    
    for(; maxgrad_pt < maxgrad_pt_max; maxgrad_pt++, gradxyii_pt++)
    {
        float dx = *((float*)gradxyii_pt);
        float dy = *(1+(float*)gradxyii_pt);
        *maxgrad_pt = sqrtf(dx*dx + dy*dy);
    }
    
    // 2. smear up/down direction into temp buffer
    maxgrad_pt = data.maxGradients[level] + width+1;
    maxgrad_pt_max = data.maxGradients[level] + width*(height-1)-1;
    float* maxgrad_t_pt = maxGradTemp + width+1;
    for(;maxgrad_pt<maxgrad_pt_max; maxgrad_pt++, maxgrad_t_pt++)
    {
        float g1 = maxgrad_pt[-width];
        float g2 = maxgrad_pt[0];
        if(g1 < g2) g1 = g2;
        float g3 = maxgrad_pt[width];
        if(g1 < g3)
            *maxgrad_t_pt = g3;
        else
            *maxgrad_t_pt = g1;
    }
    
    float numMappablePixels = 0;
    // 2. smear left/right direction into real data
    maxgrad_pt = data.maxGradients[level] + width+1;
    maxgrad_pt_max = data.maxGradients[level] + width*(height-1)-1;
    maxgrad_t_pt = maxGradTemp + width+1;
    for(;maxgrad_pt<maxgrad_pt_max; maxgrad_pt++, maxgrad_t_pt++)
    {
        float g1 = maxgrad_t_pt[-1];
        float g2 = maxgrad_t_pt[0];
        if(g1 < g2) g1 = g2;
        float g3 = maxgrad_t_pt[1];
        if(g1 < g3)
        {
            *maxgrad_pt = g3;
            if(g3 >= MIN_ABS_GRAD_CREATE)
                numMappablePixels++;
        }
        else
        {
            *maxgrad_pt = g1;
            if(g1 >= MIN_ABS_GRAD_CREATE)
                numMappablePixels++;
        }
    }
    
    if(level==0)
        //this->numMappablePixels = numMappablePixels;
    
    FrameMemory::getInstance().returnBuffer(maxGradTemp);
    
    data.maxGradientsValid[level] = true;
}

void Frame::releaseMaxGradients(int level)
{
    FrameMemory::getInstance().returnBuffer(data.maxGradients[level]);
    data.maxGradients[level] = 0;
}

void Frame::buildIDepthAndIDepthVar(int level)
{
    if (! data.hasIDepthBeenSet)
    {
        printfAssert("Frame::buildIDepthAndIDepthVar(): idepth has not been set yet!\n");
        return;
    }
    if (level == 0)
    {
        printf("Frame::buildIDepthAndIDepthVar(0): Loading depth from disk is not implemented yet! No-op.\n");
        return;
    }
    
    require(IDEPTH, level - 1);

    
    if(data.idepthValid[level] && data.idepthVarValid[level])
        return;

    
    int width = data.width[level];
    int height = data.height[level];
    
    if (data.idepth[level] == 0)
        data.idepth[level] = FrameMemory::getInstance().getFloatBuffer(width * height);
    if (data.idepthVar[level] == 0)
        data.idepthVar[level] = FrameMemory::getInstance().getFloatBuffer(width * height);
    
    int sw = data.width[level - 1];
    
    const float* idepthSource = data.idepth[level - 1];
    const float* idepthVarSource = data.idepthVar[level - 1];
    float* idepthDest = data.idepth[level];
    float* idepthVarDest = data.idepthVar[level];
    
    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            int idx = 2*(x+y*sw);
            int idxDest = (x+y*width);
            
            float idepthSumsSum = 0;
            float ivarSumsSum = 0;
            int num=0;
            
            // build sums
            float ivar;
            float var = idepthVarSource[idx];
            if(var > 0)
            {
                ivar = 1.0f / var;
                ivarSumsSum += ivar;
                idepthSumsSum += ivar * idepthSource[idx];
                num++;
            }
            
            var = idepthVarSource[idx+1];
            if(var > 0)
            {
                ivar = 1.0f / var;
                ivarSumsSum += ivar;
                idepthSumsSum += ivar * idepthSource[idx+1];
                num++;
            }
            
            var = idepthVarSource[idx+sw];
            if(var > 0)
            {
                ivar = 1.0f / var;
                ivarSumsSum += ivar;
                idepthSumsSum += ivar * idepthSource[idx+sw];
                num++;
            }
            
            var = idepthVarSource[idx+sw+1];
            if(var > 0)
            {
                ivar = 1.0f / var;
                ivarSumsSum += ivar;
                idepthSumsSum += ivar * idepthSource[idx+sw+1];
                num++;
            }
            
            if(num > 0)
            {
                float depth = ivarSumsSum / idepthSumsSum;
                idepthDest[idxDest] = 1.0f / depth;
                idepthVarDest[idxDest] = num / ivarSumsSum;
            }
            else
            {
                idepthDest[idxDest] = -1;
                idepthVarDest[idxDest] = -1;
            }
        }
    }
    
    data.idepthValid[level] = true;
    data.idepthVarValid[level] = true;
}

void Frame::releaseIDepth(int level)
{
    if (level == 0)
    {
        printf("Frame::releaseIDepth(0): Storing depth on disk is not supported yet! No-op.\n");
        return;
    }
    
    FrameMemory::getInstance().returnBuffer(data.idepth[level]);
    data.idepth[level] = 0;
}


void Frame::releaseIDepthVar(int level)
{
    if (level == 0)
    {
        printf("Frame::releaseIDepthVar(0): Storing depth variance on disk is not supported yet! No-op.\n");
        return;
    }
    FrameMemory::getInstance().returnBuffer(data.idepthVar[level]);
    data.idepthVar[level] = 0;
}

void Frame::printfAssert(const char* message) const
{
    assert(!message);
    printf("%s\n", message);
}

