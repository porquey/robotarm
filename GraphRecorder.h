#ifndef __robotarm__GraphRecorder__
#define __robotarm__GraphRecorder__

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cmath>

#define DEFAULT_TEST_TIME 20000000

using namespace std;

/// GraphRecorder class assists in recording values for a fixed amount of time
//NOTE: Does not record time, time must be passed as one of the values
class GraphRecorder
{
public:
    enum RecorderState {RECORDING, STOPPED};
    void writeValue(double a, double b);
    void writeValue(double a, double b, double c);
    void start(string s);
    void start(string s, clock_t time);
    void open(string s);
    void close();
private:
    RecorderState state;
    clock_t endTime;
    ofstream fs;
};

/// RampValue class increments a value linearly with time
class RampValue
{
public:
    void start(double ramp, clock_t time);
    void start(double ramp, clock_t time, clock_t delay);
    double getCurrentValue();

private:
    double totalTime;
    double startTime;
    double rampValue;
};

#endif