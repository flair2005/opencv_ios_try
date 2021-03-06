//
//  setting.cpp
//  opencv_ios_try
//
//  Created by test on 10/17/16.
//  Copyright © 2016 test. All rights reserved.
//

#include "setting.hpp"

RunningStats runningStats;


bool autoRun = true;
bool autoRunWithinFrame = true;

int debugDisplay = 0;

bool onSceenInfoDisplay = true;
bool displayDepthMap = true;
bool dumpMap = false;
bool doFullReConstraintTrack = false;

// dyn config
bool printPropagationStatistics = false;
bool printFillHolesStatistics = false;
bool printObserveStatistics = false;
bool printObservePurgeStatistics = false;
bool printRegularizeStatistics = false;
bool printLineStereoStatistics = false;
bool printLineStereoFails = false;

bool printTrackingIterationInfo = false;

bool printFrameBuildDebugInfo = false;
bool printMemoryDebugInfo = false;

bool printKeyframeSelectionInfo = false;
bool printConstraintSearchInfo = false;
bool printOptimizationInfo = false;
bool printRelocalizationInfo = false;

bool printThreadingInfo = false;
bool printMappingTiming = false;
bool printOverallTiming = false;

bool plotTrackingIterationInfo = false;
bool plotSim3TrackingIterationInfo = false;
bool plotStereoImages = false;
bool plotTracking = false;


float freeDebugParam1 = 1;
float freeDebugParam2 = 1;
float freeDebugParam3 = 1;
float freeDebugParam4 = 1;
float freeDebugParam5 = 1;

float KFDistWeight = 4;
float KFUsageWeight = 3;

float minUseGrad = 5;
float cameraPixelNoise2 = 4*4;
float depthSmoothingFactor = 1;

bool allowNegativeIdepths = true;
bool useMotionModel = false;
bool useSubpixelStereo = true;
bool multiThreading = true;
bool useAffineLightningEstimation = true;



bool useFabMap = false;
bool doSlam = true;
bool doKFReActivation = true;
bool doMapping = true;

int maxLoopClosureCandidates = 10;
int maxOptimizationIterations = 100;
int propagateKeyFrameDepthCount = 0;
float loopclosureStrictness = 1.5;
float relocalizationTH = 0.7;


bool saveKeyframes =  false;
bool saveAllTracked =  false;
bool saveLoopClosureImages =  false;
bool saveAllTrackingStages = false;
bool saveAllTrackingStagesInternal = false;

bool continuousPCOutput = false;


bool fullResetRequested = false;
bool manualTrackingLossIndicated = false;


std::string packagePath = "";
