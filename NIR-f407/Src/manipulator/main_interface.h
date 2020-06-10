#pragma once

#include <vector>

#include "part.h"

#error  НИКОГДА так не делайте. юзинги для неймспейсов вообще можно делать только в сишниках - иначе можно устроить просто _ад_ из именных совпадений
using namespace std;

struct PairOfPartsMayCollide
{
#error лучше дать им значения по умолчанию - nullptr
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
#error зачем пустой конструктор?
	Robot(){};
	void addPart(Part* newPart);
	void addPairOfPartsForChecking(Part* part1, Part* part2);
	float getSlowdownCoefficient(const float* currentConfiguration, const float* nextConfiguration, int numberOfParts);
};
