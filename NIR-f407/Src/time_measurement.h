#pragma once


class TimeStatisticData
{
private:
  float sumOfIterationTimes;
  int numberOfIterations;
public:
  TimeStatisticData()
  {
    sumOfIterationTimes = 0;
    numberOfIterations = 0;
    minIterationTime = 10000000;
    maxIterationTime = 0;
    averageIterationTime = 0;
  }
  int minIterationTime;
  int maxIterationTime;
  float averageIterationTime;
  
  void addValue(int time);
};

void startMeasurement();
void stopMeasurement();
int getTimeMCS();

void addTimeValueForGJK_Statistic(int time);
void addTimeValueForSAT_Statistic(int time);
void addTimeValueForCommon_Statistic(int time);