#include "time_measurement.h"

#include "main.h"

extern TIM_HandleTypeDef htim3;
unsigned int timerOverflowCount;

void startMeasurement()
{
  HAL_TIM_Base_Stop(&htim3);
  timerOverflowCount = 0;
  TIM3->CNT = 0;
  HAL_TIM_Base_Start(&htim3);
}

void stopMeasurement()
{
  HAL_TIM_Base_Stop(&htim3);
  timerOverflowCount = 0;
  TIM3->CNT = 0;
}

int getTimeMCS()
{
  HAL_TIM_Base_Stop(&htim3);
  unsigned int result = TIM3->CNT;
  result += timerOverflowCount * 1000;
  HAL_TIM_Base_Start(&htim3);
  return result;
}

void TimeStatisticData::addValue(int time)
{
  if(time < minIterationTime)
  {
    minIterationTime = time;
  }
  if(time > maxIterationTime)
  {
    maxIterationTime = time;
  }
  sumOfIterationTimes += time;
  numberOfIterations++;
  averageIterationTime = sumOfIterationTimes / (float)numberOfIterations;
}

TimeStatisticData GJK_Statistic;
TimeStatisticData SAT_Statistic;
TimeStatisticData Common_Statistic;

void addTimeValueForGJK_Statistic(int time)
{
  GJK_Statistic.addValue(time);
}
void addTimeValueForSAT_Statistic(int time)
{
  SAT_Statistic.addValue(time);
}
void addTimeValueForCommon_Statistic(int time)
{
  Common_Statistic.addValue(time);
}
