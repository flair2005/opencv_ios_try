/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ROSOutput3DWrapper.h"
#include "util/SophusUtil.h"
#include "util/settings.h"

#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "sophus/sim3.hpp"
#include "GlobalMapping/g2oTypeSim3Sophus.h"

namespace lsd_slam
{


ROSOutput3DWrapper::ROSOutput3DWrapper(int width, int height)
{

}

ROSOutput3DWrapper::~ROSOutput3DWrapper()
{
}


void ROSOutput3DWrapper::publishKeyframe(Frame* f)
{

}

void ROSOutput3DWrapper::publishTrackedFrame(Frame* kf)
{

}



void ROSOutput3DWrapper::publishKeyframeGraph(KeyFrameGraph* graph)
{
}

void ROSOutput3DWrapper::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
{
	// unimplemented ... do i need it?
}

void ROSOutput3DWrapper::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
{
	// unimplemented ... do i need it?
}

void ROSOutput3DWrapper::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
{
}

}
