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

    if (recording){
        if (targetTime != 0 && (clock() - targetTime) > TEST_TIME){
            recording = false;
            close();
        }
        else{
            fs << value << "\t" << time << endl;
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
    targetTime = clock();
}
