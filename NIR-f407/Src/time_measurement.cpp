#include "time_measurement.h"

#include "main.h"

extern TIM_HandleTypeDef htim1;
uint32_t timerOverflowCount;

int TimeStatisticCollector::activeObjectsCount = 0;

void TimeStatisticCollector::startMeasurement()
{
    if(activeObjectsCount == 0)
    {
        HAL_TIM_Base_Start(&htim1);
    }
    activeObjectsCount++;
    startTimeMCS = getTimeMCS();
}

void TimeStatisticCollector::stopMeasurement()
{
    int endTimeMCS = getTimeMCS();
    
    activeObjectsCount--;
    if(activeObjectsCount == 0)
    {
        HAL_TIM_Base_Stop(&htim1);
        //TIM1->CNT = 0;
        timerOverflowCount = 0;
    }
    
    int difference = endTimeMCS - startTimeMCS;
    
    addValue(difference);
}

int TimeStatisticCollector::getTimeMCS()
{
  __disable_irq ();
  unsigned int result = TIM1->CNT; 
  result += timerOverflowCount * 1000;
  __enable_irq ();
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
