//
//  setting.hpp
//  opencv_ios_try
//
//  Created by test on 10/17/16.
//  Copyright © 2016 test. All rights reserved.
//

#ifndef setting_hpp
#define setting_hpp

#include <stdio.h>
#define SE3TRACKING_MIN_LEVEL 1
#define SE3TRACKING_MAX_LEVEL 5

#define SIM3TRACKING_MIN_LEVEL 1
#define SIM3TRACKING_MAX_LEVEL 5

#define QUICK_KF_CHECK_LVL 4

#define PYRAMID_LEVELS (SE3TRACKING_MAX_LEVEL > SIM3TRACKING_MAX_LEVEL ? SE3TRACKING_MAX_LEVEL : SIM3TRACKING_MAX_LEVEL)
#endif /* setting_hpp */
