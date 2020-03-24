
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

float Robot::getSlowdownCoefficient(const float* currentConfiguration, const float* nextConfiguration, int numberOfParts)
{

	const float safeDistance = 0.1;
	const float minAllowedTime = 10;
	
	if(numberOfParts != parts.size())
	{
		return 0;
	}
	
	float minTime = INFINITY;
	float coefficientOfSlowdown = 1;
	
	/*for (int i = 0; i < parts.size(); i++)
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
	*/
	updatePartsConfiguration(currentConfiguration, nextConfiguration, numberOfParts);

	for (int i = 0; i < pairsOfPartsForChecking.size(); i++)
	{
		Part* firstPart = pairsOfPartsForChecking[i].part1;
		Part* secondPart = pairsOfPartsForChecking[i].part2;
		bool isNearSecondPart = firstPart->checkRoughBoundingCollision(*secondPart);
		if (isNearSecondPart)
		{
			float time = firstPart->getNextCollisionTime(*secondPart);
			if(time < minTime)
			{
				minTime = time;
			}
		}
	}

	if(minTime < minAllowedTime)
	{
		float slowdown = minTime / minAllowedTime;
		if(slowdown < coefficientOfSlowdown)
			coefficientOfSlowdown = slowdown;
	}
	
	return coefficientOfSlowdown;
}

void Robot::updatePartsConfiguration(const float* currentConfiguration, const float* nextConfiguration, int numberOfParts)
{
	if(numberOfParts != parts.size())
	{
		return;
	}
	for (int i = 0; i < parts.size(); i++)
	{
		parts[i]->updateTransform(currentConfiguration[i], nextConfiguration[i]);
	}
}
