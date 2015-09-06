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

#define TEST_TIME 20000000

using namespace std;

class GraphRecorder
{
public:
    void writeValue(double value, double target, double time);
    void open(char* s);
    void close();
    void start(char* s);
    
private:
    bool recording;
    double targetTime;
    ofstream fs;
};


#endif /* defined(__robotarm__GraphRecorder__) */
