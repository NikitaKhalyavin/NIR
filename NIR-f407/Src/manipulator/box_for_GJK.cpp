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

float OBB_GJK::getSpeedInDirection(const Vector3f& partLinearSpeed, const Vector3f& partAngleSpeed, const Vector3f& distance) const
{
    Vector3f ownSC_Distance = fromPartSC * distance;
    
    Vector3f zeroOnPartPosition = transformToParent * Vector3f(0,0,0);
    Vector3f linearSpeed = partLinearSpeed + partAngleSpeed.cross(zeroOnPartPosition); 
    Vector3f ownSC_LinearSpeed = fromPartSC * linearSpeed;
    
    float distanceLength = sqrt(ownSC_Distance.dot(ownSC_Distance));
    Vector3f distanceDirection = ownSC_Distance / distanceLength;
    
    float speed = ownSC_LinearSpeed.dot(distanceDirection);
        
    //get direction in spherical SC
    //the start position of separation plane is the plane xOz
    float angleHorizontal = atan2(distanceDirection(0), distanceDirection(2));
    float angleVertical = atan2(distanceDirection(1), sqrt( pow(distanceDirection(0),2) + pow(distanceDirection(2),2) ));
    
    //Go to separating flat CS
    //first turning around axis Y
    float angleY = -angleHorizontal;
    float angleX = -angleVertical;
    
    //while angles are zeros any box axis is equals to appropriate SP's axis
    
    /*
          ^Z
          |
    +-----+-----+
    |     |     |
    |     |     |
    +-----+-----+--->X
    |     |     |
    |     |     |
    +-----+-----+
    When the square is rotating, the ordinate of the highest point is calculating by formula:
    hMax = sin(angle) * c, where c is the length of square's gipothenuse, and angle is the angle between axis Z and giphothenuse by the highest point
    the highest point is changing periodically, so
    hMax = sin( (angle mod Pi/2) + Pi/4) * c
    
          ^Z
          |  b
    +-----+-----+
    |     |     |a
    +-----+-----+--->X
    |     |     |
    +-----+-----+
    If we use a rectangle, the period is Pi
    hMax = max( cos( (angle mod Pi) - Fi_0), cos( (angle mod Pi) + Fi_0)) * c, where Fi_0 is atan2(b,a), a is the highest of box, b is that's length
    
    Next angle means angle mod Pi
    
    To prevent the differentiation of function max, we'll divide the period on two parts
             cos(angle - Fi_0) * c, if angle <= 2*Fi_0
    hMax = {    
             cos(angle + Fi_0) * c, otherwise
    
    the speed of the highest point:
                  -sin(angle - Fi_0) * c, if angle <= 2*Fi_0
    hMaxSpeed = {    
                  -sin(angle + Fi_0) * c, otherwise
    
    the abs of hMaxSpeed is the continous function
    
    Next we'll go to the 3d space
    
            ^Z
            |
        +---+-+-----+
       /    |/     /|
      +-----+-----+ +
     /     /     /|/|
    +-----+-----+ +-+--->Y
    |     |     |/|/
    +-----+-----+ +
    |    /|     |/
    +---+-+-----+
       /
      <X
    
    we'll turn the giphothenuse around two axis, which are normals for axis Z
    zero position of gipothenuse is colleniar to axisZ
    first turn is around axisY, next - around axisX
    first: rNew = r * cos(angleY)
    second: rFinal = rNew * cos(angleX)

    So, the highest of gipothenuse equals r*cos(angleY)*cos(angleX)
    Similar to flat version,
                
    
             cos(angleY - Fi_0Y) * cos(angleX - Fi_0X) * c, if angleY <= 2*Fi_0Y and angleX <= 2*Fi_0X
             
             cos(angleY + Fi_0Y) * cos(angleX - Fi_0X) * c, if angleY > 2*Fi_0Y and angleX <= 2*Fi_0X
    hMax = {                                                                                                ,
             cos(angleY - Fi_0Y) * cos(angleX + Fi_0X) * c, if angleY <= 2*Fi_0Y and angleX > 2*Fi_0X
    
             cos(angleY + Fi_0Y) * cos(angleX + Fi_0X) * c, otherwise

    where Fi_0Y = atan2(sizeX, sizeZ), Fi_0X = atan2(sizeY, sizeZ), gipothenuse c = sqrt(sizeX^2 + sizeY^2 + sizeZ^2)
    
                          -cos(angleY - Fi_0Y) * sin(angleX - Fi_0X) * c, if angleY <= 2*Fi_0Y and angleX <= 2*Fi_0X
            
                          -cos(angleY + Fi_0Y) * sin(angleX - Fi_0X) * c, if angleY > 2*Fi_0Y and angleX <= 2*Fi_0X
    d(hMax)/d(angleX) = {                                                                                                ,
                          -cos(angleY - Fi_0Y) * sin(angleX + Fi_0X) * c, if angleY <= 2*Fi_0Y and angleX > 2*Fi_0X
    
                          -cos(angleY + Fi_0Y) * sin(angleX + Fi_0X) * c, otherwise
                         
                          -sin(angleY - Fi_0Y) * cos(angleX - Fi_0X) * c, if angleY <= 2*Fi_0Y and angleX <= 2*Fi_0X
                                               
                          -sin(angleY + Fi_0Y) * cos(angleX - Fi_0X) * c, if angleY > 2*Fi_0Y and angleX <= 2*Fi_0X
    d(hMax)/d(angleY) = {                                                                                                ,
                          -sin(angleY - Fi_0Y) * cos(angleX + Fi_0X) * c, if angleY <= 2*Fi_0Y and angleX > 2*Fi_0X
                                               
                          -sin(angleY + Fi_0Y) * cos(angleX + Fi_0X) * c, otherwise
    
    d(hMax)/dt = d(hMax)/d(angleY)*d(angleY)/dt + d(hMax)/d(angleX)*d(angleX)/dt
    
    */
    
    
    //zero angle - when one of box angles is collinear to axis X
    
    float Fi_0Y = atan2(localSC_Sizes(0), localSC_Sizes(2));
    float Fi_0X = atan2(localSC_Sizes(1), localSC_Sizes(2));
    
    Matrix3f turnToPlaneSc;
    turnToPlaneSc(0,0) = cos(angleY);
    turnToPlaneSc(0,1) = 0;
    turnToPlaneSc(0,2) = -sin(angleY);
    turnToPlaneSc(1,0) = 0;
    turnToPlaneSc(1,1) = 1;
    turnToPlaneSc(1,2) = 0;
    turnToPlaneSc(2,0) = sin(angleY);
    turnToPlaneSc(2,1) = 0;
    turnToPlaneSc(2,2) = cos(angleY);
    
    turnToPlaneSc(0,0) *= 1;
    turnToPlaneSc(0,1) *= 0;
    turnToPlaneSc(0,2) *= 0;
    turnToPlaneSc(1,0) *= 0;
    turnToPlaneSc(1,1) *= cos(angleX);
    turnToPlaneSc(1,2) *= -sin(angleX);
    turnToPlaneSc(2,0) *= 0;
    turnToPlaneSc(2,1) *= sin(angleX);
    turnToPlaneSc(2,2) *= cos(angleX);
    
    Vector3f angleSpeedInPlaneSC = turnToPlaneSc * partAngleSpeed;
    
    float angleSpeedX = angleSpeedInPlaneSC(0);
    float angleSpeedY = angleSpeedInPlaneSC(1);
    
    float gipothenusa = sqrt(pow(localSC_Sizes(0), 2) + pow(localSC_Sizes(1), 2) + pow(localSC_Sizes(2), 2));
    
    float hMax_dAngleX;
    float hMax_dAngleY;
    
    if(angleY <=  2*Fi_0Y)
    {
        if(angleX <=  2*Fi_0X)
        {
            hMax_dAngleY = -sin(angleY - Fi_0Y) * cos(angleX - Fi_0X) * gipothenusa;
            hMax_dAngleX = -cos(angleY - Fi_0Y) * sin(angleX - Fi_0X) * gipothenusa;
        }
        else
        {
            hMax_dAngleY = -sin(angleY - Fi_0Y) * cos(angleX + Fi_0X) * gipothenusa;
            hMax_dAngleX = -cos(angleY - Fi_0Y) * sin(angleX + Fi_0X) * gipothenusa;
        }
    }
    else
    {
        if(angleX <=  2*Fi_0X)
        {
            hMax_dAngleY = -sin(angleY + Fi_0Y) * cos(angleX - Fi_0X) * gipothenusa;
            hMax_dAngleX = -cos(angleY + Fi_0Y) * sin(angleX - Fi_0X) * gipothenusa;
        }
        else
        {
            hMax_dAngleY = -sin(angleY + Fi_0Y) * cos(angleX + Fi_0X) * gipothenusa;
            hMax_dAngleX = -cos(angleY + Fi_0Y) * sin(angleX + Fi_0X) * gipothenusa;
        }        
    }
    
    float camSpeed = abs(hMax_dAngleX * angleSpeedX) + abs(hMax_dAngleY * angleSpeedY);
    speed += camSpeed;
    
    return speed;
}
