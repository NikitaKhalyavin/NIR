#pragma once

#include "../Src/correct_Eigen_include.h"

class BoundingVolume
{
protected:
	Affine3f transformToParent;
public:
	BoundingVolume(){};
	void setTransformToParent(Affine3f transform);
};
