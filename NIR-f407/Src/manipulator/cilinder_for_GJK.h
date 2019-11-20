#pragma once


#include "../Src/correct_Eigen_include.h"

#include "bounding_volume.h"

using Eigen::Vector3f;
using Eigen::Affine3f;
using Eigen::Matrix3f;

class Cilinder_GJK : public BoundingVolume
{
private:

	float radius;
	float highest;
	Matrix3f fromPartSC;

public:

	Cilinder_GJK(){};
	Cilinder_GJK(const Cilinder_GJK& input);
	Cilinder_GJK(const float radius, const float highest, const Affine3f thisToPartSC);

	void create(const float radius, const float highest);

	Vector3f supportFunction(const Vector3f& direction) const;
};
