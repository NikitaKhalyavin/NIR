#pragma once
#define EIGEN_DONT_PARALLELIZE

#include <Eigen/Dense>
#include <Eigen/Geometry>

using Eigen::Affine3f;
class BoundingVolume
{
protected:
	Affine3f transformToParent;
public:
	BoundingVolume(){};
	void setTransformToParent(Affine3f transform);
};
