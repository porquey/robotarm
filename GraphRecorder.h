//
//  GraphRecorder.h
//  robotarm
//
//  Created by Forest Fraser on 5/09/15.
//  Copyright (c) 2015 UoA. All rights reserved.
//

#ifndef __robotarm__GraphRecorder__
#define __robotarm__GraphRecorder__

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cmath>

#define DEFAULT_TEST_TIME 20000000

using namespace std;

class GraphRecorder
{
public:
    void writeValue(double value, double time);
    void writeValue(double value, int iteration);
    void updateTarget();
    void open(char* s);
    void close();
    void start(char* s);
    void start(char* s, double time);
    void start(char* s, double variation, double time);

    
private:
    bool recording;
    double startTime;
    double endTime;
    ofstream fs;
};

class RampValue
{
public:
    void start(double ramp, double time);
    double getCurrentValue();

private:
    double totalTime;
    double startTime;
    double rampValue;
};


#endif /* defined(__robotarm__GraphRecorder__) */
