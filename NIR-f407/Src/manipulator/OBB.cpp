#include "../time_measurement.h"
#include "OBB.h"



OBB::OBB()
{
	for (unsigned int i = 0; i < numberOfDims; i++)
	{
		guideVectors[i] = Vector3f(0, 0, 0);
		sizes = Vector3f(0, 0, 0);
	}
	turnToLocalSC = Matrix3f::Identity();
}

OBB::OBB(const Vector3f sizes, const Vector3f centerPosition, const Matrix3f rotation)
{
	create(sizes, centerPosition, rotation);
	turnToLocalSC = Matrix3f::Identity();
}

void OBB::create(const Vector3f sizes, const Vector3f centerPosition, const Matrix3f rotation)
{
	this->sizes = sizes / 2;
	this->centerPosition = centerPosition;
	//transform to the partSC
	for (unsigned int i = 0; i < numberOfDims; i++)
	{
		Vector3f guideVectorInLocalSC(0, 0, 0);
		guideVectorInLocalSC(i) = 1;

		guideVectors[i] = rotation * guideVectorInLocalSC;
	}
}

void OBB::update(const Affine3f& transformToGSK, const Matrix3f& rotateToGSK, const Matrix3f& rotateToPartSC)
{
	turnToLocalSC = rotateToPartSC;
	for (unsigned int i = 0; i < numberOfDims; i++)
	{
		GSC_GuideVectors[i] = rotateToGSK * guideVectors[i];
	}
	GSC_CenterPosition = transformToGSK * centerPosition;
}

float OBB::getLocalProjectionOnGSC_Vector(const Vector3f& vector) const
{
	Vector3f localVector = turnToLocalSC * vector;
	float proj = 0;
	for (unsigned int i = 0; i < numberOfDims; i++)
	{
		float dimProj = guideVectors[i].dot(localVector) * sizes(i);
		proj += abs(dimProj);
	}
	return proj;
}


bool OBB::checkCollisionByGuideVector(const OBB& other, unsigned int vectorNumber) const
{
	float ownLocalDirectionProj = this->sizes(vectorNumber);
	float otherLocalDirectionProj = other.getLocalProjectionOnGSC_Vector(this->GSC_GuideVectors[vectorNumber]);

	float ownCenterProjection = this->GSC_CenterPosition.dot(this->GSC_GuideVectors[vectorNumber]);
	float otherCenterProjection = other.GSC_CenterPosition.dot(this->GSC_GuideVectors[vectorNumber]);

	float minOwn = ownCenterProjection - ownLocalDirectionProj;
	float maxOwn = ownCenterProjection + ownLocalDirectionProj;
	float minOther = otherCenterProjection - otherLocalDirectionProj;
	float maxOther = otherCenterProjection + otherLocalDirectionProj;

	if ((maxOther < minOwn) || (minOther > maxOwn))
	{
		return false;
	}
	return true;
}


bool OBB::checkCollisionByGSK_Vector(const OBB& other, Vector3f& GSK_Vector) const
{
	float ownLocalDirectionProj = this->getLocalProjectionOnGSC_Vector(GSK_Vector);
	float otherLocalDirectionProj = other.getLocalProjectionOnGSC_Vector(GSK_Vector);

	float ownCenterProjection = this->GSC_CenterPosition.dot(GSK_Vector);
	float otherCenterProjection = other.GSC_CenterPosition.dot(GSK_Vector);

	float minOwn = ownCenterProjection - ownLocalDirectionProj;
	float maxOwn = ownCenterProjection + ownLocalDirectionProj;
	float minOther = otherCenterProjection - otherLocalDirectionProj;
	float maxOther = otherCenterProjection + otherLocalDirectionProj;

	if ((maxOther < minOwn) || (minOther > maxOwn))
	{
		return false;
	}
	return true;
}


TimeStatisticCollector SAT_Statistic;

bool OBB::checkCollisionBySAT(const OBB& other) const
{
	SAT_Statistic.startMeasurement();
	//check own axis of boxes
	for (unsigned int i = 0; i < 3; i++)
	{
		if (!this->checkCollisionByGuideVector(other, i))
		{
			SAT_Statistic.stopMeasurement();
			return false;
		}
		if (!other.checkCollisionByGuideVector(*this, i))
		{
			SAT_Statistic.stopMeasurement();
			return false;
		}
	}

	//check cross-axis
	for (unsigned int i = 0; i < 3; i++)
	{
		for (unsigned int j = 0; j < 3; j++)
		{
			Vector3f GSK_Vector = this->GSC_GuideVectors[i].cross(other.GSC_GuideVectors[j]);

			if (GSK_Vector.isZero())
			{
				SAT_Statistic.stopMeasurement();
				return true;
			}
			if (!this->checkCollisionByGSK_Vector(other, GSK_Vector))
			{
				SAT_Statistic.stopMeasurement();
				return false;
			}

		}
	}
	SAT_Statistic.stopMeasurement();
	return true;
}
