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

	void updatePartsConfiguration(const vector<float>& newConfiguration);
public:
	Robot(){};
	void addPart(Part* newPart);
	void addPairOfPartsForChecking(int part1Index, int part2Index);
	float getSlowdownCoefficient(vector<float>& currentConfiguration, const vector<float>& receivedSpeed);
};
