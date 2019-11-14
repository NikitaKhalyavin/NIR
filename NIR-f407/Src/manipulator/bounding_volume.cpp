#include "pch.h"
#include "bounding_volume.h"

void BoundingVolume::setTransformToParent(Affine3f transform)
{
	this->transformToParent = transform;
}

