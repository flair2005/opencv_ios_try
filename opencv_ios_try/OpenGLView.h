//
//  OpenGLView.h
//  HelloGL
//
//  Created by wanglixin on 16/6/30.
//  Copyright © 2016年 Mac. All rights reserved.
//

#import <UIKit/UIKit.h>

#import <QuartzCore/QuartzCore.h>
#include <OpenGLES/ES2/gl.h>
#include <OpenGLES/ES2/glext.h>
#include <opencv2/opencv.hpp>
#include "CommonStruct.h"

// Add to top of file
typedef struct {
    float Position[3];
    float Color[4];
} Vertex;

@interface OpenGLView : UIView
{
    double curTime;
    double lastTime;
    float fov;
    float dim;
    float asp;
    float ph;
    float th;
    float fixZ;
    CAEAGLLayer* _eaglLayer;
    EAGLContext* _context;
    GLuint _colorRenderBuffer;
    
    GLuint _positionSlot;
    GLuint _colorSlot;
    
    GLuint _projectionUniform;
    
    GLuint _modelViewUniform;
    
    float _currentRotation;
    
    GLuint _depthRenderBuffer;
    
    cv::Mat projMat;
    cv::Mat viewMat;
    std::vector<Vertex> Vertices;
    
}
- (void) SetMapPoints: (std::vector<MapPointChamo>&) verts;
@end
