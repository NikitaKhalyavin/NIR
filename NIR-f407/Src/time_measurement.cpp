#include "time_measurement.h"

#include "main.h"

extern TIM_HandleTypeDef htim3;
uint32_t timerOverflowCount;

int TimeStatisticCollector::activeObjectsCount = 0;

void TimeStatisticCollector::startMeasurement()
{
    startTimeMCS = getTimeMCS();
    if(activeObjectsCount == 0)
        HAL_TIM_Base_Start(&htim3);
    activeObjectsCount++;
}

void TimeStatisticCollector::stopMeasurement()
{
    int endTimeMCS = getTimeMCS();
    activeObjectsCount--;
    if(activeObjectsCount == 0)
    {    
        HAL_TIM_Base_Stop(&htim3);
        timerOverflowCount = 0;
        TIM3->CNT = 0;
    }
    addValue(endTimeMCS - startTimeMCS);
}

int TimeStatisticCollector::getTimeMCS()
{
  unsigned int result = TIM3->CNT;
  result += timerOverflowCount * 1000;
  return result;
}


void TimeStatisticCollector::addValue(int time)
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
