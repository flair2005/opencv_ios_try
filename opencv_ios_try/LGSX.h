#pragma once

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include "setting.hpp"

typedef Eigen::Matrix<float, 6, 1> Vector6;
typedef Eigen::Matrix<float, 6, 6> Matrix6x6;

typedef Eigen::Matrix<float, 7, 1> Vector7;
typedef Eigen::Matrix<float, 7, 7> Matrix7x7;

typedef Eigen::Matrix<float, 4, 1> Vector4;
typedef Eigen::Matrix<float, 4, 4> Matrix4x4;

class LGS4
{
public:
  Matrix4x4 A;
  Vector4 b;

  float error;
  size_t num_constraints;

  inline void initialize(const size_t maxnum_constraints)
  {
    A.setZero();
    b.setZero();
    memset(SSEData,0, sizeof(float)*4*15);
    error = 0;
    this->num_constraints = 0;
  }

  inline void finishNoDivide()
  {
  }

  inline void update(const Vector4& J, const float& res, const float& weight)
  {
    A.noalias() += J * J.transpose() * weight;
    b.noalias() -= J * (res * weight);
    error += res * res * weight;
    num_constraints += 1;
  }

private:
  EIGEN_ALIGN16 float SSEData[4*15];
};



/**
 * Builds 4dof LGS (used for depth-lgs, at it has only 7 non-zero entries in jacobian)
 * only used to accumulate data, NOT really as LGS
 */
class LGS6
{
public:
  Matrix6x6 A;
  Vector6 b;

  float error;
  size_t num_constraints;

  inline void initialize(const size_t maxnum_constraints)
  {
    A.setZero();
    b.setZero();
    memset(SSEData,0, sizeof(float)*4*28);

    error = 0;
    this->num_constraints = 0;
  }

  inline void finishNoDivide()
  {
  }


  void finish()
  {
    finishNoDivide();
    A /= (float) num_constraints;
    b /= (float) num_constraints;
    error /= (float) num_constraints;
  }

  inline void update(const Vector6& J, const float& res, const float& weight)
  {
    A.noalias() += J * J.transpose() * weight;
    b.noalias() -= J * (res * weight);
    error += res * res * weight;
    num_constraints += 1;
  }

private:
  EIGEN_ALIGN16 float SSEData[4*28];
};

class LGS7
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Matrix7x7 A;
  Vector7 b;

  float error;
  size_t num_constraints;

  void initializeFrom(const LGS6& ls6, const LGS4& ls4)
  {
  	// set zero
  	A.setZero();
  	b.setZero();

  	// add ls6
  	A.topLeftCorner<6,6>() = ls6.A;
  	b.head<6>() = ls6.b;

  	// add ls4
  	int remap[4] = {2,3,4,6};
  	for(int i=0;i<4;i++)
  	{
  		b[remap[i]] += ls4.b[i];
  		for(int j=0;j<4;j++)
  			A(remap[i], remap[j]) += ls4.A(i,j);
  	}

  	num_constraints = ls6.num_constraints + ls4.num_constraints;
  }
};

