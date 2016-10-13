//
//  ViewController.h
//  opencv_ios_try
//
//  Created by test on 9/28/16.
//  Copyright © 2016 test. All rights reserved.
//

#import <UIKit/UIKit.h>

#import "OpenGLView.h"

class ImageProcessor;

@interface ViewController : UIViewController<UIImagePickerControllerDelegate,UINavigationControllerDelegate>{
    int centerX;
    int centerY;
    int btnHeight;
    ImageProcessor* imgProc;
}
@property UITextView *consoloLabel;
@property UIImageView *frameLabel;
@property UIImagePickerController* picker;
@property (nonatomic, strong) OpenGLView *glView;

- (void) takePhoto;
- (void) clearMap;
@end

