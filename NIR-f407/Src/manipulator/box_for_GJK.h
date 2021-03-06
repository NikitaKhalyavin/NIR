#pragma once


#include "../Src/correct_Eigen_include.h"


#include "bounding_volume.h"

using Eigen::Vector3f;
using Eigen::Affine3f;
using Eigen::Matrix3f;

class OBB_GJK : public BoundingVolume
{
private:

	Vector3f localSC_Sizes;
	Matrix3f fromPartSC;
	
public:

	OBB_GJK(){};
	OBB_GJK(const OBB_GJK& input);
	OBB_GJK(const Vector3f sizes, const Affine3f thisToPartSC);

	void create(const Vector3f sizes);

	Vector3f supportFunction(const Vector3f& direction) const;
    float getSpeedInDirection(const Vector3f& partLinearSpeed, const Vector3f& partAngleSpeed, const Vector3f& distance) const;
};
