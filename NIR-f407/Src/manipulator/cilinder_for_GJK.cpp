#include "pch.h"
#include "cilinder_for_GJK.h"

Cilinder_GJK::Cilinder_GJK(const float radius, const float highest, const Affine3f thisToPartSC)
{
	transformToParent = thisToPartSC;
	create(radius, highest);
}

Cilinder_GJK::Cilinder_GJK(const Cilinder_GJK& input)
{
	this->transformToParent = input.transformToParent;
	create(input.radius, input.highest);
}

void Cilinder_GJK::create(const float radius, const float highest)
{
	this->radius = radius;
	this->highest = highest;
	Matrix3f rotateToPartSC;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			rotateToPartSC(i, j) = transformToParent(i, j);
		}
	}
	fromPartSC = rotateToPartSC.inverse();
}

Vector3f Cilinder_GJK::supportFunction(const Vector3f& direction) const
{
	Vector3f temp = fromPartSC * direction;
	if (temp.isZero())
		return temp;
	
	float tempProjToXYLength = sqrt(temp(0) * temp(0) + temp(1) * temp(1));
  const float precize = 0.0001f;
	if (tempProjToXYLength < precize)
	{
		temp(0) = radius; temp(1) = 0;
	}
	else
	{
		temp /= tempProjToXYLength;
		temp *= radius;
	}
	if (temp(2) > 0)
	{
		temp(2) = highest;
	}
	else
	{
		temp(2) = -highest;
	}
	return transformToParent * temp;
}
