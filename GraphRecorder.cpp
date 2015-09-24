#include "GraphRecorder.h"

/// Opens text file for writing
/// In: s: name of text file e.g. "test.txt"
void GraphRecorder::open(string s)
{
    // rewrite existing file
    fs.open(s,ios_base::out|ios_base::trunc);
}

/// Writes 2 values of type double to the text file seperated by a tab
/// In: a,b: values to record in text file
void GraphRecorder::writeValue(double a, double b)
{
    if (clock() > endTime)
        close();
    else if (state == RECORDING)
    {
        fs << a << "\t" << b << endl;
    }
}

/// Writes 3 values of type double to the text file seperated by a tab
/// In: a,b,c: values to record in text file
void GraphRecorder::writeValue(double a, double b, double c)
{
    if (clock() > endTime)
        close();
    else if (state == RECORDING)
    {
        fs << a << "\t" << b << "\t" << c << endl;
    }
}

/// Stops recording and closes the text file
void GraphRecorder::close()
{
    state = STOPPED;
    fs.close();
    cerr << endl << endl << "RECORDING STOPPED." << endl << endl;
}

/// Starts recording for a 20 seconds
/// In: s: name of text file to write to
void GraphRecorder::start(string s)
{
    start(s, DEFAULT_TEST_TIME);
}

/// Starts recording
/// In: s: name of text file to write to
///     time: time to record
void GraphRecorder::start(string s, clock_t time)
{
    cerr << endl << endl << "RECORDING STARTED." << endl << endl;
    open(s);
    endTime = clock() + time;
    state = RECORDING;
}

//--------------------------------------RampValue--------------------------------------//

/// Starts a ramp
/// In: value: end value of the ramp
///     time: duration of the ramp in microseconds
void RampValue::start(double ramp, clock_t time)
{
    rampValue = ramp;
    totalTime = time;
    startTime = clock();
}

/// Starts the ramp with a delay
/// In: value: end value of the ramp
///     time: duration of the ramp in microseconds
///     delay: duration of delay before the ramp starts
void RampValue::start(double value, clock_t time, clock_t delay)
{
    rampValue = value;
    totalTime = time;
    startTime = clock()+delay;
}

/// getCurrentValue
/// returns the value at the current time
/// Out: value (double)
double RampValue::getCurrentValue()
{
    if (clock() < startTime)
        return 0;
    double out = rampValue * (clock()-startTime)/totalTime;
    if (out > rampValue)
        return rampValue;
    else
        return out;
}

