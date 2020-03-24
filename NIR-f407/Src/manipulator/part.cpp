#include "part.h"

#define EPSILON 0.0001f

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

void Part::updateTransform(float coordinate, float speed)
{
	Affine3f ownTransform = translateToParentSC;
    Vector3f ownLinearSpeed = Vector3f(0,0,0);
    Vector3f ownAngleSpeed = Vector3f(0,0,0);
    
	if (isRotating)
	{
		ownTransform.rotate(Eigen::AngleAxisf(coordinate, movingAxis));	
        ownAngleSpeed = movingAxis * speed;
	}
	else
	{
		ownTransform.translate(movingAxis * coordinate);
        ownLinearSpeed = movingAxis * speed;
	}

	if (parent == NULL)
	{
		fullTransform = ownTransform;
        angleSpeed = ownAngleSpeed;
        linearSpeed = ownLinearSpeed;
	}
	else
	{
		fullTransform = parent->fullTransform * ownTransform;
        Matrix3f fullRotate;// = fullTransform.linear();
        Matrix3f parentFullRotate;// = parent->fullTransform.linear();
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                fullRotate(i, j) = fullTransform(i, j);
                parentFullRotate(i, j) = parent->fullTransform(i, j);
            }
        }
        angleSpeed = parent->angleSpeed + (fullRotate * ownAngleSpeed);
        Vector3f parentSC_JointPosition = ownTransform * Vector3f(0,0,0);
        linearSpeed = parent->linearSpeed + (parent->angleSpeed).cross(parentFullRotate * parentSC_JointPosition) + (fullRotate * ownLinearSpeed);
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
	Sphere_GJK newSphere(radius, transform);
	spheres.push_back(newSphere);
}

void Part::addCilinder(float  radius, float highest, const Vector3f position, const Vector3f angles)
{
	Affine3f transform = Affine3f::Identity();
	transform.translate(position);
	transform.rotate(Eigen::AngleAxisf(angles(0), Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf(angles(1), Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(angles(2), Vector3f::UnitZ()));
	Cilinder_GJK newCilinder(radius, highest, transform);
	cilinders.push_back(newCilinder);
}

bool Part::checkRoughBoundingCollision(const Part& other) const
{
	return this->roughBounding.checkCollisionBySAT(other.roughBounding);
}


template <typename T1, typename T2>
float Part::getNextCollisionTimeForPairOfVolumes(const Part& other, const T1& ownVolume, const T2& otherVolume) const
{
	Vector3f pairOfVolumesNearest = calculateDistanceByGJK(ownVolume, otherVolume, this->fullTransform, other.fullTransform, this->rotateToOwnSC, other.rotateToOwnSC);
	//Vector3f nextPairOfVolumesNearest = calculateDistanceByGJK(ownVolume, otherVolume, this->nextFullTransform, other.nextFullTransform, this->nextRotateToOwnSC, other.nextRotateToOwnSC);
	
	//Vector3f difference = nextPairOfVolumesNearest - pairOfVolumesNearest;
	/*
	float dist = sqrt(pairOfVolumesNearest.dot(pairOfVolumesNearest));
	float speedOfApproach = 0;//-difference.dot(pairOfVolumesNearest) / dist;
	
	if(speedOfApproach < 0.0001f)
	{
		return INFINITY;
		//distance between this pair of volumes is increasing
	}
	
	float nextDist = 0;//sqrt(nextPairOfVolumesNearest.dot(nextPairOfVolumesNearest));
	const float distancethresh = 0.1f;
	if(nextDist < distancethresh)
	{
		//parts are too close, stop moving
		return 0;
	}
	
	float timeToCollision = dist / speedOfApproach;
	*/
    
    float dist = sqrt(pairOfVolumesNearest.dot(pairOfVolumesNearest));
    
    const float criticalApproachDistance = 0.001f;
    const float safeApproachDistance = 0.1f;
    
    if(dist < criticalApproachDistance)
        return 0;
    
    Vector3f ownSC_LinearSpeed = this->rotateToOwnSC * this->linearSpeed;
    Vector3f otherSC_LinearSpeed = other.rotateToOwnSC * other.linearSpeed;
    Vector3f ownSC_AngleSpeed = this->rotateToOwnSC * this->angleSpeed;
    Vector3f otherSC_AngleSpeed = other.rotateToOwnSC * other.angleSpeed;
    
    float ownSpeed = ownVolume.getSpeedInDirection(ownSC_LinearSpeed, ownSC_AngleSpeed, this->rotateToOwnSC * pairOfVolumesNearest);
    float otherSpeed = otherVolume.getSpeedInDirection(otherSC_LinearSpeed, otherSC_AngleSpeed, - (other.rotateToOwnSC * pairOfVolumesNearest) );
    
    float speedOfApproach = ownSpeed + otherSpeed;
    if(speedOfApproach < EPSILON)
        return INFINITY;

    if(dist < safeApproachDistance)
        return 0;
    
    return (dist - safeApproachDistance) / speedOfApproach;
	//return timeToCollision;
}
	

template <typename T>
float Part::getNextCollisionTimeForVolume(const Part& other, const T& volume) const
{
	float minTime = INFINITY;
	for (int i = 0; i < other.boxes.size(); i++)
	{
		float pairCollisionTime = getNextCollisionTimeForPairOfVolumes(other, volume, other.boxes[i]);
		if(pairCollisionTime < minTime)
		{
			minTime = pairCollisionTime;
		}
	}
	for (int i = 0; i < other.spheres.size(); i++)
	{
		float pairCollisionTime = getNextCollisionTimeForPairOfVolumes(other, volume, other.spheres[i]);
		if(pairCollisionTime < minTime)
		{
			minTime = pairCollisionTime;
		}
	}
	for (int i = 0; i < other.cilinders.size(); i++)
	{
		float pairCollisionTime = getNextCollisionTimeForPairOfVolumes(other, volume, other.cilinders[i]);
		if(pairCollisionTime < minTime)
		{
			minTime = pairCollisionTime;
		}
	}
	return minTime;
}



float Part::getNextCollisionTime(const Part& other) const
{
	float minTime = INFINITY;

	for (int i = 0; i < this->spheres.size(); i++)
	{
		float pairCollisionTime = getNextCollisionTimeForVolume(other, this->spheres[i]);
        if(pairCollisionTime < minTime)
		{
			minTime = pairCollisionTime;
		}
	}
    for (int i = 0; i < this->boxes.size(); i++)
	{
		float pairCollisionTime = getNextCollisionTimeForVolume(other, this->boxes[i]);
		if(pairCollisionTime < minTime)
		{
			minTime = pairCollisionTime;
		}
	}
	for (int i = 0; i < this->cilinders.size(); i++)
	{
		float pairCollisionTime = getNextCollisionTimeForVolume(other, this->cilinders[i]);
		if(pairCollisionTime < minTime)
		{
			minTime = pairCollisionTime;
		}
	}
	return minTime;
}
