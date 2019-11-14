#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include "bounding_volume.h"

using Eigen::Vector3f;
using Eigen::Affine3f;
using Eigen::Matrix3f;


class Sphere_GJK : public BoundingVolume
{
private:

	float radius;
	Matrix3f fromPartSC;

public:

	Sphere_GJK(){};
	Sphere_GJK(const Sphere_GJK& input);
	Sphere_GJK(const float radius, const Affine3f thisToPartSC);

	void create(const float radius);

	Vector3f supportFunction(const Vector3f& direction) const;
};
