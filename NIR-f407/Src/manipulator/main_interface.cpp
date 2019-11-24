
#include "pch.h"
#include "main_interface.h"

void Robot::addPart(Part* newPart)
{
	parts.push_back(newPart);
}

void Robot::addPairOfPartsForChecking(Part* part1, Part* part2)
{
	PairOfPartsMayCollide newPair;
	newPair.part1 = part1;
	newPair.part2 = part2;
	pairsOfPartsForChecking.push_back(newPair);
}

float Robot::getSlowdownCoefficient(float* currentConfiguration, const float* receivedSpeed, int numberOfParts)
{

	const float safeDistance = 0.01;
	const float maxRelativeProj = 0.1;
	
	if(numberOfParts != parts.size())
	{
		return 1;
	}
	
	float coefficientOfSlowdown = 1;
	
	for (int i = 0; i < parts.size(); i++)
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
	
	updatePartsConfiguration(currentConfiguration, numberOfParts);

	bool isNear[numberOfParts];
	Vector3f currentNearestPoint[numberOfParts];
	for (int i = 0; i < pairsOfPartsForChecking.size(); i++)
	{
		Part* firstPart = pairsOfPartsForChecking[i].part1;
		Part* secondPart = pairsOfPartsForChecking[i].part2;
		bool isNearSecondPart = firstPart->checkRoughBoundingCollision(*secondPart);
		isNear[i] = isNearSecondPart;
		Vector3f nearestPoint(0, 0, 0);
		if (isNearSecondPart)
		{
			nearestPoint = firstPart->getNearestPoint(*secondPart);
		}
		currentNearestPoint[i] = nearestPoint;
	}

	for (int i = 0; i < numberOfParts; i++)
	{
		currentConfiguration[i] += receivedSpeed[i];
	}

	updatePartsConfiguration(currentConfiguration, numberOfParts);
	
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

void Robot::updatePartsConfiguration(const float* newConfiguration, int numberOfParts)
{
	if(numberOfParts != parts.size())
	{
		return;
	}
	for (int i = 0; i < parts.size(); i++)
	{
		parts[i]->updateTransform(newConfiguration[i]);
	}
}
