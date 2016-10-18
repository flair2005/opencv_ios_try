#pragma once

#include "sim3.hpp"
#include "se3.hpp"

typedef Sophus::SE3d SE3;
typedef Sophus::Sim3d Sim3;
typedef Sophus::SO3d SO3;
#define toSophus(x) ((x).cast<double>())
#define sophusType double

inline Sim3 sim3FromSE3(const SE3& se3, sophusType scale)
{
	Sim3 result(se3.unit_quaternion(), se3.translation());
	result.setScale(scale);
	return result;
}

inline SE3 se3FromSim3(const Sim3& sim3)
{
	return SE3(sim3.quaternion(), sim3.translation());
}

