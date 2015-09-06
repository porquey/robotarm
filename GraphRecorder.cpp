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

void GraphRecorder::writeValue(double value, double target, double time){
    if ((clock() - targetTime) > IDLE_TIME){
        recording = false;
    }
    if (recording){
        
        fs << value << "\t" << time << endl;
        
        if (targetTime == 0 && (abs(value - target) < 0.01))
        {
            targetTime = clock();
        }
    }
}

void GraphRecorder::close(){
    cerr << endl << endl << "RECORDING STOPPED." << endl << endl;
    recording = false;
    fs.close();
}

void GraphRecorder::start(char* s){
    cerr << endl << endl << "RECORDING STARTED." << endl << endl;
    open(s);
    recording = true;
    targetTime = 0;
}
