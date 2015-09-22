//
//  GraphRecorder.cpp
//  robotarm
//
//  Created by Forest Fraser on 5/09/15.
//  Copyright (c) 2015 UoA. All rights reserved.
//

#include "GraphRecorder.h"

void GraphRecorder::open(char* s){
    //Rewrite existing file
    fs.open(s,ios_base::out|ios_base::trunc);
}

void GraphRecorder::writeValue(double value, double time){

    if (recording){
        if (time > endTime){
            recording = false;
            close();
        }
        else{
            fs << (time-startTime)/1000000 << "\t" << value << endl;
        }
    }
}

void GraphRecorder::writeValue(double value, double target, double time){
    
    if (recording){
        if (time > endTime){
            recording = false;
            close();
        }
        else{
            fs << (time-startTime)/1000000 << "\t" << value << "\t" << target << endl;
        }
    }
}

void GraphRecorder::writeValue(double value, int iteration){
    if (recording){
        fs << value << "\t" << iteration << endl;
    }
}

void GraphRecorder::close(){
    cerr << endl << endl << "RECORDING STOPPED." << endl << endl;
    recording = false;
    fs.close();
}

void GraphRecorder::start(char* s){
    start(s, DEFAULT_TEST_TIME);
}

void GraphRecorder::start(char*s, double time){
    cerr << endl << endl << "RECORDING STARTED." << endl << endl;
    open(s);
    recording = true;
    startTime = clock();
    endTime = startTime + time;
}

void RampValue::start(double ramp, double time){
    rampValue = ramp;
    totalTime = time;
    startTime = clock();
}

void RampValue::start(double ramp, double time, double delay){
    rampValue = ramp;
    totalTime = time;
    startTime = clock()+delay;
}

double RampValue::getCurrentValue(){
    if (clock() < startTime){
        return 0;
    }
    double out = rampValue * (clock()-startTime)/totalTime;
    if (out > rampValue)
        return rampValue;
    else
        return out;
}

