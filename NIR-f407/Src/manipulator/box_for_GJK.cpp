#include "pch.h"
#include "box_for_GJK.h"

OBB_GJK::OBB_GJK(const Vector3f sizes, const Affine3f thisToPartSC)
{
	transformToParent = thisToPartSC;
	create(sizes);
}

OBB_GJK::OBB_GJK(const OBB_GJK& input)
{
	this->transformToParent = input.transformToParent;
	create(input.localSC_Sizes);
}

void OBB_GJK::create(const Vector3f sizes)
{
	localSC_Sizes = sizes;
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

Vector3f OBB_GJK::supportFunction(const Vector3f& direction) const
{
	Vector3f temp = fromPartSC * direction;
	if (temp.isZero())
		return temp;
	Vector3f result;
	unsigned const int dimNumber = 3;
	for (unsigned int i = 0; i < dimNumber; i++)
	{
		if (temp(i) < 0)
		{
			result(i) = -localSC_Sizes(i) / 2;
		}
		else
		{
			result(i) = localSC_Sizes(i) / 2;
		}
	}
	return transformToParent * result;
}
