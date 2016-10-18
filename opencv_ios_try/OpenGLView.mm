//
//  OpenGLView.m
//  HelloGL
//
//  Created by wanglixin on 16/6/30.
//  Copyright © 2016年 Mac. All rights reserved.
//

#import "OpenGLView.h"
#include <math.h>
#define Cos(th) cos(3.1415926/180*(th))
#define Sin(th) sin(3.1415926/180*(th))
#define Tan(x) tan(3.1415926/180*(x))

@implementation OpenGLView

+ (Class)layerClass {
    return [CAEAGLLayer class];
}

- (void)setupLayer {
    _eaglLayer = (CAEAGLLayer*) self.layer;
    _eaglLayer.opaque = YES;
}

- (void)setupContext {
    EAGLRenderingAPI api = kEAGLRenderingAPIOpenGLES2;
    _context = [[EAGLContext alloc] initWithAPI:api];
    if (!_context) {
        NSLog(@"Failed to initialize OpenGLES 2.0 context");
        exit(1);
    }
    
    if (![EAGLContext setCurrentContext:_context]) {
        NSLog(@"Failed to set current OpenGL context");
        exit(1);
    }
}

- (void)setupRenderBuffer {
    glGenRenderbuffers(1, &_colorRenderBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, _colorRenderBuffer);
    [_context renderbufferStorage:GL_RENDERBUFFER fromDrawable:_eaglLayer];
}

- (void)setupFrameBuffer {
    GLuint framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, _colorRenderBuffer);
    
    
    // Add to end of setupFrameBuffer
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, _depthRenderBuffer);
}


// Add new method before init
- (void)setupDisplayLink {
    CADisplayLink *displayLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(render)];
    [displayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
}

// Add new method right after setupRenderBuffer
- (void)setupDepthBuffer {
    glGenRenderbuffers(1, &_depthRenderBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, _depthRenderBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, self.frame.size.width, self.frame.size.height);
}

cv::Mat populateFromFrustumLeft(GLfloat left, GLfloat right, GLfloat bottom, GLfloat top, GLfloat near, GLfloat far) {
    cv::Mat projMat_t(4,4,CV_32FC1);
    
    projMat_t.at<float>(0,0)  = (2.0 * near) / (right - left);
    projMat_t.at<float>(1,0)  = 0.0;
    projMat_t.at<float>(2,0)  = 0.0;
    projMat_t.at<float>(3,0) = 0.0;
    
    projMat_t.at<float>(0,1)  = 0.0;
    projMat_t.at<float>(1,1)  = (2.0 * near) / (top - bottom);
    projMat_t.at<float>(2,1)  = 0.0;
    projMat_t.at<float>(3,1) = 0.0;
    
    projMat_t.at<float>(0,2)  = (right + left) / (right - left);
    projMat_t.at<float>(1,2)  = (top + bottom) / (top - bottom);
    projMat_t.at<float>(2,2) = -(far + near) / (far - near);
    projMat_t.at<float>(3,2) = -1.0;
    
    projMat_t.at<float>(0,3)  = 0.0;
    projMat_t.at<float>(1,3)  = 0.0;
    projMat_t.at<float>(2,3) = -(2.0 * far * near) / (far - near);
    projMat_t.at<float>(3,3) = 0.0;
    return projMat_t;
}

cv::Mat LookAt(cv::Mat posi, cv::Mat tar, cv::Mat up) {
    cv::Mat dir =tar - posi;
    cv::Mat z =dir;
    cv::normalize(z, z);
    cv::Mat x =  up.cross(z); // x = up cross z
    cv::normalize(x, x);
    cv::Mat y= z.cross(x); // y = z cross x
    cv::Mat re = cv::Mat::eye(3,3,CV_32FC1);
    x.copyTo(re.col(0).rowRange(0, 3));
    y.copyTo(re.col(1).rowRange(0, 3));
    z.copyTo(re.col(2).rowRange(0, 3));
    
    return re;
}

- (void) Projection
{
    float zn = dim / 4;
    float zf = dim * 100;
    float nearPlanH = zn*Tan(fov / 2);
    projMat = populateFromFrustumLeft(-nearPlanH*asp*dim / 3, nearPlanH*asp*dim / 3, -nearPlanH*dim / 3, nearPlanH*dim / 3, zn, zf).t();
}
- (void) SetViewMat
{
    cv::Mat posi(3,1,CV_32FC1);
    posi.at<float>(0) = Cos(ph)*Sin(th)*fixZ;
    posi.at<float>(1) = Sin(ph)*fixZ;
    posi.at<float>(2) = Cos(ph)*Cos(th)*fixZ;
    cv::Mat tar = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat up = cv::Mat::zeros(3, 1, CV_32FC1);
    up.at<float>(1) = 1;
    viewMat = cv::Mat::eye(4, 4, CV_32FC1);
    cv::Mat rMat = LookAt(posi, tar, up);
    rMat.copyTo(viewMat.colRange(0, 3).rowRange(0, 3));
    viewMat.at<float>(2,3) =-fixZ;
    viewMat=viewMat.t();
    //std::cout<<viewMat<<std::endl;
}

- (void) mouseMoveEvent: (UIGestureRecognizer *)sender
{
    UIPanGestureRecognizer* panReco = (UIPanGestureRecognizer* )sender;
    CGPoint touchPoint = [panReco velocityInView:self];
    //NSLog(@"Pan:%f,%f",touchPoint.x,touchPoint.y);
    th = (int)(th + touchPoint.x/50) % 360;      //  Translate x movement to azimuth
    ph = (int)(ph + touchPoint.y/50) % 360;      //  Translate y movement to elevation
    [self SetViewMat];
    [self render];
}

- (void) wheelEvent: (UIGestureRecognizer *)sender
{
    UIPinchGestureRecognizer* pinchReco = (UIPinchGestureRecognizer* )sender;
    //NSLog(@"Pinch:%f",pinchReco.velocity);
    if (pinchReco.velocity > 0)
        dim -= 0.1;
    else
        dim += 0.1;
    [self Projection];
    [self render];
}


- (void)render {
//    curTime = CACurrentMediaTime();
//    double deltaTime = curTime-lastTime;
//    if (deltaTime>0.01){
//        //NSLog(@"FPS: %f", 1/(float)deltaTime);
//        lastTime = curTime;
//    }else{
//        return;
//    }
    glClearColor(0, 104.0/255.0, 55.0/255.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glUniformMatrix4fv(_projectionUniform, 1, 0, (GLfloat*)projMat.data);
    glUniformMatrix4fv(_modelViewUniform, 1, 0, (GLfloat*)viewMat.data);
    
    
    // 1
    glViewport(0, 0, self.frame.size.width, self.frame.size.height);
    
    // 2
    glVertexAttribPointer(_positionSlot, 3, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex), 0);
    glVertexAttribPointer(_colorSlot, 4, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex), (GLvoid*) (sizeof(float) *3));
    
    // 3
    glDrawArrays(GL_POINTS, 0, Vertices.size());
    [_context presentRenderbuffer:GL_RENDERBUFFER];
    glDisable(GL_DEPTH_TEST);
}

- (GLuint)compileShader:(NSString*)shaderName withType:(GLenum)shaderType {
    // 1
    NSString* shaderPath = [[NSBundle mainBundle] pathForResource:shaderName
                                                           ofType:@"glsl"];
    NSError* error;
    NSString* shaderString = [NSString stringWithContentsOfFile:shaderPath
                                                       encoding:NSUTF8StringEncoding error:&error];
    if (!shaderString) {
        NSLog(@"Error loading shader: %@", error.localizedDescription);
        exit(1);
    }
    
    // 2
    GLuint shaderHandle = glCreateShader(shaderType);
    
    // 3
    const char* shaderStringUTF8 = [shaderString UTF8String];
    int shaderStringLength = [shaderString length];
    glShaderSource(shaderHandle, 1, &shaderStringUTF8, &shaderStringLength);
    
    // 4
    glCompileShader(shaderHandle);
    
    // 5
    GLint compileSuccess;
    glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &compileSuccess);
    if (compileSuccess == GL_FALSE) {
        GLchar messages[256];
        glGetShaderInfoLog(shaderHandle, sizeof(messages), 0, &messages[0]);
        NSString *messageString = [NSString stringWithUTF8String:messages];
        NSLog(@"%@", messageString); 
        exit(1); 
    } 
    
    return shaderHandle;
}

- (void)compileShaders {
    // 1
    GLuint vertexShader = [self compileShader:@"SimpleVertex"
                                     withType:GL_VERTEX_SHADER];
    GLuint fragmentShader = [self compileShader:@"SimpleFragment"
                                       withType:GL_FRAGMENT_SHADER];
    
    // 2
    GLuint programHandle = glCreateProgram();
    glAttachShader(programHandle, vertexShader);
    glAttachShader(programHandle, fragmentShader);
    glLinkProgram(programHandle);
    
    // 3
    GLint linkSuccess;
    glGetProgramiv(programHandle, GL_LINK_STATUS, &linkSuccess);
    if (linkSuccess == GL_FALSE) {
        GLchar messages[256];
        glGetProgramInfoLog(programHandle, sizeof(messages), 0, &messages[0]);
        NSString *messageString = [NSString stringWithUTF8String:messages];
        NSLog(@"%@", messageString);
        exit(1);
    }
    
    // 4
    glUseProgram(programHandle);
    
    // 5
    _positionSlot = glGetAttribLocation(programHandle, "Position");
    _colorSlot = glGetAttribLocation(programHandle, "SourceColor");
    glEnableVertexAttribArray(_positionSlot);
    glEnableVertexAttribArray(_colorSlot);
    
    // Add to bottom of compileShaders
    _projectionUniform = glGetUniformLocation(programHandle, "Projection");
    
    // Add to end of compileShaders
    _modelViewUniform = glGetUniformLocation(programHandle, "Modelview");
}

- (void)setupVBOs {
    glGenBuffers(1, &vertexBuffer);
}

- (void)updateVBOs {
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, Vertices.size(), Vertices.data(), GL_STATIC_DRAW);
    [self render];
}


// Replace initWithFrame with this
- (id)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];
    if (self) {
        asp = frame.size.width / frame.size.height;
        dim = 3;
        th = ph = 0;
        fixZ = 6;
        fov =60;
        UIPanGestureRecognizer *panReco = [[UIPanGestureRecognizer alloc]
                                           initWithTarget:self
                                           action:@selector(mouseMoveEvent:)];
        [self addGestureRecognizer:panReco];
        UIPinchGestureRecognizer *pinchReco = [[UIPinchGestureRecognizer alloc]
                                               initWithTarget:self
                                               action:@selector(wheelEvent:)];
        [self addGestureRecognizer:pinchReco];
        [self setupLayer];
        [self setupContext];
        
        // Add to initWithFrame, right before call to setupRenderBuffer
        [self setupDepthBuffer];
        
        [self setupRenderBuffer];
        [self setupFrameBuffer];
        
        [self compileShaders];
        [self setupVBOs];
        //[self render];
            //[self setupDisplayLink];
        [self Projection];
        [self SetViewMat];
        
    }
    return self;
}

- (void) SetMapPoints: (std::vector<MapPointChamo>&) verts{
    cv::Point3f aviPosi;
    cv::Point3f aviAbsPosi;
    for (int i=0; i<verts.size();i++){
        aviPosi = aviPosi+ verts[i].posi ;
        aviAbsPosi.x = aviPosi.x+ abs(verts[i].posi.x);
        aviAbsPosi.y = aviPosi.y+ abs(verts[i].posi.y);
        aviAbsPosi.z = aviPosi.z+ abs(verts[i].posi.z);
    }
    aviPosi.x = aviPosi.x/verts.size();
    aviPosi.y = aviPosi.y/verts.size();
    aviPosi.z = aviPosi.z/verts.size();
    aviAbsPosi.x = aviAbsPosi.x/verts.size();
    aviAbsPosi.y = aviAbsPosi.y/verts.size();
    aviAbsPosi.z = aviAbsPosi.z/verts.size();
    Vertices.resize(verts.size());
    float range =10;
    for (int i=0; i<verts.size();i++){
        Vertices[i].Position[0] = (verts[i].posi.x- aviPosi.x)/aviAbsPosi.x*range ;
        Vertices[i].Position[1] = (verts[i].posi.y- aviPosi.y)/aviAbsPosi.y*range ;
        Vertices[i].Position[2] = (verts[i].posi.z- aviPosi.z)/aviAbsPosi.z*range ;
        Vertices[i].Color[0] =rand()/ (float)RAND_MAX;
        Vertices[i].Color[1] =rand()/ (float)RAND_MAX;
        Vertices[i].Color[2] =rand()/ (float)RAND_MAX;
        Vertices[i].Color[3] =1;
    }
    [self updateVBOs];
}

// Replace dealloc method with this
- (void)dealloc
{
//    [_context release];
    _context = nil;
//    [super dealloc];
}


@end
