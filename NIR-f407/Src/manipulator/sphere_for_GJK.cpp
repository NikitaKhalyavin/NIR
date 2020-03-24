#include "pch.h"
#include "sphere_for_GJK.h"

Sphere_GJK::Sphere_GJK(const float radius, const Affine3f thisToPartSC)
{
	transformToParent = thisToPartSC;
	create(radius);
}

Sphere_GJK::Sphere_GJK(const Sphere_GJK& input)
{
	this->transformToParent = input.transformToParent;
	create(input.radius);
}

void Sphere_GJK::create(const float radius)
{
	this->radius = radius;
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

Vector3f Sphere_GJK::supportFunction(const Vector3f& direction) const
{
	Vector3f temp = fromPartSC * direction;
	if (temp.isZero())
		return temp;
	float tempLength = sqrt(temp.dot(temp));
	temp /= tempLength;
	temp *= radius;
	return transformToParent * temp;
}

float Sphere_GJK::getSpeedInDirection(const Vector3f& partLinearSpeed, const Vector3f& partAngleSpeed, const Vector3f& distance) const
{
    Vector3f ownSC_Distance = fromPartSC * distance;
    
    Vector3f zeroOnPartPosition = transformToParent * Vector3f(0,0,0);
    Vector3f linearSpeed = partLinearSpeed + partAngleSpeed.cross(zeroOnPartPosition); 
    Vector3f ownSC_LinearSpeed = fromPartSC * linearSpeed;
    
    float distanceLength = sqrt(ownSC_Distance.dot(ownSC_Distance));
    float speed = ownSC_LinearSpeed.dot(ownSC_Distance) / distanceLength;
    
    return speed;
}
