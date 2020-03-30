#pragma once


class TimeStatisticCollector
{
private:
    
    int startTimeMCS;
    static int activeObjectsCount;

    int getTimeMCS();
    void addValue(int time);
    
public:
    TimeStatisticCollector()
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
    float sumOfIterationTimes;
    int numberOfIterations;
    
    void startMeasurement();
    void stopMeasurement();
};


