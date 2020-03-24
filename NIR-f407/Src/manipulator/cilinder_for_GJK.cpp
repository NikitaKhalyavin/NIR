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

float Cilinder_GJK::getSpeedInDirection(const Vector3f& partLinearSpeed, const Vector3f& partAngleSpeed, const Vector3f& distance) const
{
    Vector3f ownSC_Distance = fromPartSC * distance;
    
    Vector3f zeroOnPartPosition = transformToParent * Vector3f(0,0,0);
    Vector3f linearSpeed = partLinearSpeed + partAngleSpeed.cross(zeroOnPartPosition); 
    Vector3f ownSC_LinearSpeed = fromPartSC * linearSpeed;
    
    float distanceLength = sqrt(ownSC_Distance.dot(ownSC_Distance));
    Vector3f distanceDirection = ownSC_Distance / distanceLength;
    
    float speed = ownSC_LinearSpeed.dot(distanceDirection);
    
    
    //cam speed calculation for cylinders is the same as for boxes, but cylinders are invariant for horizontal turning
    float angleVertical = atan2(distanceDirection(1), sqrt( pow(distanceDirection(0),2) + pow(distanceDirection(2),2) ));
    
    //Go to separating flat CS
    float angleX = -angleVertical;
    
    //So, we can use equals for rectangles to simulate a cylinder
    //while angles are zeros any box axis is equals to appropriate SP's axis
    
    /*
          ^Z
          |
    +-----+-----+
    |     |     |
    |     |     |
    +-----+-----+--->Y
    |     |     |
    |     |     |
    +-----+-----+
    When the square is rotating, the ordinate of the highest point is calculating by formula:
    hMax = sin(angle) * c, where c is the length of square's gipothenuse, and angle is the angle between axis Y and giphothenuse by the highest point
    the highest point is changing periodically, so
    hMax = sin( (angle mod Pi/2) + Pi/4) * c
    
          ^Z
          |  b
    +-----+-----+
    |     |     |a
    +-----+-----+--->Y
    |     |     |
    +-----+-----+
    If we use a rectangle, the period is Pi
    hMax = max( cos( (angle mod Pi) - Fi_0), cos( (angle mod Pi) + Fi_0)) * c, where Fi_0 is atan2(a,b), a is the highest of rectangle, b is that's length
    
    Next angle means angle mod Pi
    
    To prevent the differentiation of function max, we'll divide the period on two parts
             cos(angle - Fi_0) * c, if angle <= 2*Fi_0
    hMax = {    
             cos(angle + Fi_0) * c, otherwise
    
    the speed of the highest point:
                        -sin(angle - Fi_0) * c, if angle <= 2*Fi_0
    d(hMax)/d(angle) = {    
                        -sin(angle + Fi_0) * c, otherwise
    
    hMaxSpeed = d(hMax)/d(angle) * d(angle)/dt
    the abs of hMaxSpeed is the continous function
    
    */
    
    
    //zero angle - when one of box angles is collinear to axis X
    
    float Fi_0 = atan2(this->highest, this->radius);
    
    Matrix3f turnToPlaneSc;
    turnToPlaneSc(0,0) = 1;
    turnToPlaneSc(0,1) = 0;
    turnToPlaneSc(0,2) = 0;
    turnToPlaneSc(1,0) = 0;
    turnToPlaneSc(1,1) = cos(angleX);
    turnToPlaneSc(1,2) = -sin(angleX);
    turnToPlaneSc(2,0) = 0;
    turnToPlaneSc(2,1) = sin(angleX);
    turnToPlaneSc(2,2) = cos(angleX);
    
    Vector3f angleSpeedInPlaneSC = turnToPlaneSc * partAngleSpeed;
    
    float angleSpeedX = angleSpeedInPlaneSC(0);
    
    float gipothenusa = sqrt(pow(highest, 2) + pow(radius, 2));
    
    float hMax_dAngleX;
    
    if(angleX <=  2*Fi_0)
    {
        hMax_dAngleX = -sin(angleX - Fi_0) * gipothenusa;
    }
    else
    {
        hMax_dAngleX = -sin(angleX + Fi_0) * gipothenusa;
    }

    
    float camSpeed = abs(hMax_dAngleX * angleSpeedX);
    speed += camSpeed;
    
    return speed;
}
