#pragma once

#include <vector>

#include "part.h"

using namespace std;

struct PairOfPartsMayCollide
{
	Part* part1;
	Part* part2;
};

class Robot
{
private:
	vector<Part*> parts;
	vector<PairOfPartsMayCollide> pairsOfPartsForChecking;

	void updatePartsConfiguration(const float* currentConfiguration, const float* nextConfiguration, int numberOfParts);
public:
	Robot(){};
	void addPart(Part* newPart);
	void addPairOfPartsForChecking(Part* part1, Part* part2);
	float getSlowdownCoefficient(const float* currentConfiguration, const float* nextConfiguration, int numberOfParts);
};
