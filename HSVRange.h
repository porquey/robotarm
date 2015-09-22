#ifndef HSV_RANGE_H
#define HSV_RANGE_H


#include <stdio.h>

struct HSVRanges
{
    int lowH = 0;
    int highH = 179;
    int lowS = 0;
    int highS = 255;
    int lowV = 0;
    int highV = 255;
};

#endif