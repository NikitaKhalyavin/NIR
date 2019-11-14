
#include "pch.h"
#include "main_interface.h"

void Robot::addPart(Part* newPart)
{
	parts.push_back(newPart);
}

void Robot::addPairOfPartsForChecking(int part1Index, int part2Index)
{
	PairOfPartsMayCollide newPair;
	newPair.part1 = parts[part1Index];
	newPair.part2 = parts[part2Index];
	pairsOfPartsForChecking.push_back(newPair);
}

float Robot::getSlowdownCoefficient(vector<float>& currentConfiguration, const vector<float>& receivedSpeed)
{
	float coefficientOfSlowdown = 1;
	const float safeDistance = 0.01;
	const float maxRelativeProj = 0.1;
	
	for (int i = 0; i < currentConfiguration.size(); i++)
	{
		float min = parts[i]->getMinPosition();
		if (isnan(min)) continue;
		float max = parts[i]->getMaxPosition();
		if (isnan(max)) continue;
		float position = currentConfiguration[i];
		float speed = receivedSpeed[i];

		float distToMin = min - position;
		float projForMin = speed / distToMin;
		float slowdownForMin = 1.0;
		if (projForMin > maxRelativeProj)
		{
			slowdownForMin = maxRelativeProj / projForMin;
		}
		if (slowdownForMin < coefficientOfSlowdown)
		{
			coefficientOfSlowdown = slowdownForMin;
		}

		float distToMax = max - position;
		float projForMax = speed / distToMax;
		float slowdownForMax = 1.0;
		if (projForMax > maxRelativeProj)
		{
			slowdownForMax = maxRelativeProj / projForMax;
		}
		if (slowdownForMax < coefficientOfSlowdown)
		{
			coefficientOfSlowdown = slowdownForMax;
		}

	}
	
	updatePartsConfiguration(currentConfiguration);

	vector<bool> isNear;
	vector<Vector3f> currentNearestPoint;
	for (int i = 0; i < pairsOfPartsForChecking.size(); i++)
	{
		Part* firstPart = pairsOfPartsForChecking[i].part1;
		Part* secondPart = pairsOfPartsForChecking[i].part2;
		bool isNearSecondPart = firstPart->checkRoughBoundingCollision(*secondPart);
		isNear.push_back(isNearSecondPart);
		Vector3f nearestPoint(0, 0, 0);
		if (isNearSecondPart)
		{
			nearestPoint = firstPart->getNearestPoint(*secondPart);
		}
		currentNearestPoint.push_back(nearestPoint);
	}

	for (int i = 0; i < currentConfiguration.size(); i++)
	{
		currentConfiguration[i] += receivedSpeed[i];
	}

	updatePartsConfiguration(currentConfiguration);
	
	for (int i = 0; i < pairsOfPartsForChecking.size(); i++)
	{
		Part* firstPart = pairsOfPartsForChecking[i].part1;
		Part* secondPart = pairsOfPartsForChecking[i].part2;
		bool isNearSecondPart = isNear[i];
		if (isNearSecondPart)
		{
			Vector3f nearestPoint = currentNearestPoint[i];
			Vector3f nextNearestPoint = firstPart->getNearestPoint(*secondPart);
			Vector3f difference = nextNearestPoint - nearestPoint;

			float proj = -( difference.dot(nearestPoint) / nearestPoint.dot(nearestPoint) );
			if (proj > 0)
			{
				if (nextNearestPoint.dot(nextNearestPoint) < safeDistance * safeDistance)
				{
					coefficientOfSlowdown = 0;
					return coefficientOfSlowdown;
				}

				float ownPairSlowdown = 1.0;
				if (proj > maxRelativeProj)
				{
					ownPairSlowdown = maxRelativeProj / proj;
				}
				if (ownPairSlowdown < coefficientOfSlowdown)
				{
					coefficientOfSlowdown = ownPairSlowdown;
				}
			}
		}
	}
	return coefficientOfSlowdown;
}

void Robot::updatePartsConfiguration(const vector<float>& newConfiguration)
{
	for (int i = 0; i < parts.size(); i++)
	{
		parts[i]->updateTransform(newConfiguration[i]);
	}
}
