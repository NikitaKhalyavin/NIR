#pragma once


#include "../Src/correct_Eigen_include.h"
#include <vector>

#include "OBB.h"
#include "GJK.h"

class Part
{
private:
	Part* parent;
	OBB roughBounding;

	Affine3f translateToParentSC;
	Affine3f fullTransform;

	Matrix3f rotateToOwnSC;
	Matrix3f rotateToGlobalSC;
	
	Affine3f nextFullTransform;
	Matrix3f nextRotateToOwnSC;
	
	bool isRotating;
	Vector3f movingAxis;

	std::vector<Cilinder_GJK> cilinders;
	std::vector<Sphere_GJK> spheres;
	std::vector<OBB_GJK> boxes;

	float minPosition;
	float maxPosition;

	template <typename T>
	float getNextCollisionTimeForVolume(const Part& other, const T& volume) const;

	template <typename T1, typename T2>
	float getNextCollisionTimeForPairOfVolumes(const Part& other, const T1& ownVolume, const T2& otherVolume) const;

	
public:
	Part(const Vector3f jointPosition, const Vector3f jointAxis, bool isRotating);

	void setParent(Part* parent);
	void setMovingLimits(float minLimit, float maxLimit);
	
	float getMinPosition() { return minPosition; }
	float getMaxPosition() { return maxPosition; }

	void setRoughBounding(const Vector3f sizes, const Vector3f centerPosition, const Vector3f angles);
	
	void addBox(const Vector3f sizes, const Vector3f position, const Vector3f angles);
	void addSphere(float radius, const Vector3f position, const Vector3f angles);
	void addCilinder(float  radius, float highest, const Vector3f position, const Vector3f angles);


	void updateTransform(float coordinate, float nextCoordinate);

	bool checkRoughBoundingCollision(const Part& other) const;
	float getNextCollisionTime(const Part& other) const;

};
