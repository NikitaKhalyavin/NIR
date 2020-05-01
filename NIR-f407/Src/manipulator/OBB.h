#pragma once


#include "../Src/correct_Eigen_include.h"

class OBB
{
private:

	typedef enum { front, top, right } PlaneName;

	const static unsigned int numberOfDims = 3;
	Vector3f sizes;

	Vector3f guideVectors[numberOfDims];
	Vector3f GSC_GuideVectors[numberOfDims];

	Vector3f centerPosition;
	Vector3f GSC_CenterPosition;

	Matrix3f turnToLocalSC;
	
	float getLocalProjectionOnGSC_Vector(const Vector3f& vector) const;

	bool checkCollisionByGuideVector(const OBB& other, unsigned int vectorNumber) const;
	bool checkCollisionByGSK_Vector(const OBB& other, Vector3f& GSK_Vector) const;
    
    void copy(const OBB& other);

public:

	OBB();
	//OBB(const OBB& input) = default;
	OBB(const Vector3f sizes, const Vector3f centerPosition, const Matrix3f rotation);
	void create(const Vector3f sizes, const Vector3f centerPosition, const Matrix3f rotation);
    OBB (const OBB& other) 
    {
        copy(other);
    }
	OBB& operator = (const OBB& other)
    {
        copy(other);
        return *this;
    }

	void update(const Affine3f& transformToGSK, const Matrix3f& rotateToGSK, const Matrix3f& rotateToPartSC);
	bool checkCollisionBySAT(const OBB& other) const;
};

/*
class OBB : public BoundingVolume
{
private:
	
	typedef enum {
		frontBottomLeft, frontBottomRight, frontTopLeft, frontTopRight,
		backBottomLeft, backBottomRight, backTopLeft, backTopRight
	} PointName;
	typedef enum { front, top, right } PlaneName;

	const static unsigned int numberOfPoints = 8;
	Vector3f eigenSC_Points[numberOfPoints];
	Vector3f GSC_Points[numberOfPoints];

	Vector3f getNormalToPlane(const PlaneName plane);
	float getProjectionOnGSC_Vector(const Vector3f vector, const PointName point);
	bool checkCollisionForOneNormal(OBB& other, Vector3f normalEigenFront);

public:
	
	OBB();
	OBB(const OBB& input) = default;
	OBB(const Affine3f transform, const Vector3f sizes);

	void create(const Vector3f sizes);
	void setTransform(const Affine3f thisToGSC);
	
	bool checkCollisionBySAT(OBB& other);
};
*/
