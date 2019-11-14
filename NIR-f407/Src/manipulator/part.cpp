#include "pch.h"

#include "part.h"

Part::Part(const Vector3f jointPosition, const Vector3f jointAxis, bool isRotating)
{
	translateToParentSC = Affine3f::Identity();
	translateToParentSC.translate(jointPosition);
	movingAxis = jointAxis;
	this->isRotating = isRotating;
	parent = NULL;
	fullTransform = translateToParentSC;
	
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			rotateToGlobalSC(i, j) = fullTransform(i, j);
		}
	}
	//rotateToGlobalSC = (fullTransform.matrix).topLeftCorner(3, 3);
	rotateToOwnSC = rotateToGlobalSC.inverse();
	minPosition = NAN;
	maxPosition = NAN;
}


void Part::setMovingLimits(float min, float max)
{
	minPosition = min;
	maxPosition = max;
}

void Part::updateTransform(float coordinate)
{
	ownTransform = translateToParentSC;
	if (isRotating)
	{
		ownTransform.rotate(Eigen::AngleAxisf(coordinate, movingAxis));
	}
	else
	{
		ownTransform.translate(movingAxis * coordinate);
	}

	if (parent == NULL)
	{
		fullTransform = ownTransform;
	}
	else
	{
		fullTransform = parent->fullTransform * this->ownTransform;
	}
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			rotateToGlobalSC(i, j) = fullTransform(i, j);
		}
	}

	rotateToOwnSC = rotateToGlobalSC.inverse();
	roughBounding.update(fullTransform, rotateToGlobalSC, rotateToOwnSC);
}


void Part::setRoughBounding(Vector3f sizes, Vector3f centerPosition, Vector3f angles)
{
	Affine3f OBB_Transform = Affine3f::Identity();
	OBB_Transform.rotate(Eigen::AngleAxisf(angles(0), Vector3f::UnitX()));
	OBB_Transform.rotate(Eigen::AngleAxisf(angles(1), Vector3f::UnitY()));
	OBB_Transform.rotate(Eigen::AngleAxisf(angles(2), Vector3f::UnitZ()));
	Matrix3f OBB_Rotate;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			OBB_Rotate(i, j) = OBB_Transform(i, j);
		}
	}
	//OBB_Rotate = OBB_Transform.matrix.block<3, 3>(0, 0);
	roughBounding.create(sizes, centerPosition, OBB_Rotate);
	roughBounding.update(fullTransform, rotateToGlobalSC, rotateToOwnSC);
}

void Part::setParent(Part* parent)
{
	this->parent = parent;
}

void Part::addBox(const Vector3f sizes, const Vector3f position, const Vector3f angles)
{
	Affine3f transform = Affine3f::Identity();
	transform.translate(position);
	transform.rotate(Eigen::AngleAxisf(angles(0), Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf(angles(1), Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(angles(2), Vector3f::UnitZ()));
	OBB_GJK newBox(sizes, transform);
	boxes.push_back(newBox);
}

void Part::addSphere(float radius, const Vector3f position, const Vector3f angles)
{
	Affine3f transform = Affine3f::Identity();
	transform.translate(position);
	transform.rotate(Eigen::AngleAxisf(angles(0), Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf(angles(1), Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(angles(2), Vector3f::UnitZ()));
	Sphere_GJK newSphere(radius, ownTransform);
	spheres.push_back(newSphere);
}

void Part::addCilinder(float  radius, float highest, const Vector3f position, const Vector3f angles)
{
	Affine3f transform = Affine3f::Identity();
	transform.translate(position);
	transform.rotate(Eigen::AngleAxisf(angles(0), Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf(angles(1), Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(angles(2), Vector3f::UnitZ()));
	Cilinder_GJK newCilinder(radius, highest, ownTransform);
	cilinders.push_back(newCilinder);
}

bool Part::checkRoughBoundingCollision(const Part& other) const
{
	return this->roughBounding.checkCollisionBySAT(other.roughBounding);
}

template <typename T>
Vector3f Part::getNearestPointToVolume(const Part& other, const T& volume, const Affine3f& fullTransform, const Matrix3f& rotate) const
{
	float min = INFINITY;
	Vector3f nearest(0, 0, 0);
	for (int i = 0; i < other.boxes.size(); i++)
	{
		Vector3f pairOfVolumesNearest = calculateDistanceByGJK(volume, other.boxes[i], fullTransform, other.fullTransform, rotate, other.rotateToOwnSC);
		float dist = sqrt(pairOfVolumesNearest.dot(pairOfVolumesNearest));
		if (dist < min)
		{
			min = dist;
			nearest = pairOfVolumesNearest;
		}
	}
	for (int i = 0; i < other.spheres.size(); i++)
	{
		Vector3f pairOfVolumesNearest = calculateDistanceByGJK(volume, other.spheres[i], fullTransform, other.fullTransform, rotate, other.rotateToOwnSC);
		float dist = sqrt(pairOfVolumesNearest.dot(pairOfVolumesNearest));
		if (dist < min)
		{
			min = dist;
			nearest = pairOfVolumesNearest;
		}
	}
	for (int i = 0; i < other.cilinders.size(); i++)
	{
		Vector3f pairOfVolumesNearest = calculateDistanceByGJK(volume, other.cilinders[i], fullTransform, other.fullTransform, rotate, other.rotateToOwnSC);
		float dist = sqrt(pairOfVolumesNearest.dot(pairOfVolumesNearest));
		if (dist < min)
		{
			min = dist;
			nearest = pairOfVolumesNearest;
		}
	}
	return nearest;
}

Vector3f Part::getNearestPoint(const Part& other) const
{
	float min = INFINITY;
	Vector3f nearest(0, 0, 0);
	for (int i = 0; i < this->boxes.size(); i++)
	{
		Vector3f nearestPointToPart = getNearestPointToVolume(other, this->boxes[i], fullTransform, rotateToOwnSC);
		float dist = sqrt(nearestPointToPart.dot(nearestPointToPart));
		if (dist < min)
		{
			min = dist;
			nearest = nearestPointToPart;
		}
	}
	for (int i = 0; i < this->spheres.size(); i++)
	{
		Vector3f nearestPointToPart = getNearestPointToVolume(other, this->spheres[i], fullTransform, rotateToOwnSC);
		float dist = sqrt(nearestPointToPart.dot(nearestPointToPart));
		if (dist < min)
		{
			min = dist;
			nearest = nearestPointToPart;
		}
	}
	for (int i = 0; i < this->cilinders.size(); i++)
	{
		Vector3f nearestPointToPart = getNearestPointToVolume(other, this->cilinders[i], fullTransform, rotateToOwnSC);
		float dist = sqrt(nearestPointToPart.dot(nearestPointToPart));
		if (dist < min)
		{
			min = dist;
			nearest = nearestPointToPart;
		}
	}
	return nearest;
}
