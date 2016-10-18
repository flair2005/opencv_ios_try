//
//  ViewController.m
//  opencv_ios_try
//
//  Created by test on 9/28/16.
//  Copyright © 2016 test. All rights reserved.
//

#import "ViewController.h"
#import "common_header.h"
#import <opencv2/opencv.hpp>
#include "ImageProcessor.hpp"


@interface ViewController ()

@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    //configuretion
    btnHeight = 20;
    
    //init some parameters
    CGRect screenBounds = [[UIScreen mainScreen] bounds];
    centerX =screenBounds.size.width/2;
    centerY =screenBounds.size.height/2;
    
    //layout the buttons
    UIButton *btn = [UIButton buttonWithType:UIButtonTypeRoundedRect];
    btn.frame = CGRectMake(0, 0, centerX, btnHeight);
    btn.backgroundColor  = [UIColor clearColor];
    [btn setTitle:@"Take Photo" forState:UIControlStateNormal];
    [self.view addSubview:btn];
    [btn addTarget:self action:@selector(takePhoto) forControlEvents:UIControlEventTouchDown];
    
    UIButton *btnClear = [UIButton buttonWithType:UIButtonTypeRoundedRect];
    btnClear.frame = CGRectMake(0, btnHeight, centerX, btnHeight);
    btnClear.backgroundColor  = [UIColor clearColor];
    [btnClear setTitle:@"Clear Map" forState:UIControlStateNormal];
    [self.view addSubview:btnClear];
    [btnClear addTarget:self action:@selector(clearMap) forControlEvents:UIControlEventTouchDown];
    
    //layout the consolo
    self.consoloLabel = [[UITextView alloc] initWithFrame:CGRectMake(0, centerY, centerX, centerY)];
    self.consoloLabel.text = @"Consolo:";
    self.consoloLabel.editable = false;
    [self.view addSubview:self.consoloLabel];
    
    //layout the frame window
    self.frameLabel = [[UIImageView alloc] initWithFrame:CGRectMake(centerX, 0, centerX, centerY)];
    self.frameLabel.contentMode = UIViewContentModeScaleAspectFit;
    UIImage *image = [UIImage imageNamed:@"loli.jpeg"];
    self.frameLabel.image = image;
    [self.view addSubview:self.frameLabel];
    
    //layout the opengl viewport
    CGRect mapRect = CGRectMake(screenBounds.size.width/2,screenBounds.size.height/2, screenBounds.size.width/2, screenBounds.size.height/2);
    self.glView = [[OpenGLView alloc] initWithFrame:mapRect];
    [self.view addSubview:self.glView];
    
    //init other objects
    self.picker = [[UIImagePickerController alloc]init];//创建
    self.picker.delegate = self;//设置为托
    self.picker.sourceType = UIImagePickerControllerSourceTypeCamera;//图像来源
    self.picker.allowsEditing=NO;//允许编辑图片
    
    imgProc = new ImageProcessor();
    UIImage* reImg;
    reImg = [UIImage imageNamed:@"1.png"];
    cv::Mat imgMat = [mm_Try cvMatFromUIImage: reImg];
    imgProc->ProcessImage(imgMat);
}

- (void)imagePickerController:(UIImagePickerController *)picker didFinishPickingImage:(UIImage *)image editingInfo:(NSDictionary *)editingInfo{
    /*添加处理选中图像代码*/
    UIImage* reImg = [mm_Try convertImag: image];
    if (reImg.size.width > reImg.size.height) {
        self.frameLabel.contentMode = UIViewContentModeScaleAspectFit;
    } else {
        self.frameLabel.contentMode = UIViewContentModeScaleAspectFill;
    }
    
    if (imgProc->imgs.size()==0){
        reImg = [UIImage imageNamed:@"1.png"];
    }else{
        reImg = [UIImage imageNamed:@"2.png"];
    }
    self.frameLabel.image = reImg;
    [picker.view removeFromSuperview];//退出照相机
    cv::Mat imgMat = [mm_Try cvMatFromUIImage: reImg];
    imgProc->ProcessImage(imgMat);
    if (imgProc->MapPoints.size()>0){
        [self.glView SetMapPoints:imgProc->MapPoints];
    }
}
- (void)imagePickerControllerDidCancel:(UIImagePickerController *)picker{
    [picker.view removeFromSuperview];
}

- (void) takePhoto{
    [self.view addSubview:self.picker.view];
}

- (void) clearMap{
    int c =[mm_Try add:3 b:4];
    NSString *stringInt = [NSString stringWithFormat:@"%d",c];
    self.consoloLabel.text = stringInt;
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

@end

