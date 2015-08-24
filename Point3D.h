#ifndef POINT3D_H
#define POINT3D_H

#include <stdio.h>

class Point3D
{
public:
    Point3D();
    Point3D(double a, double b, double c);
    Point3D& operator= (const Point3D& other);
    Point3D& operator+ (Point3D& other);
    Point3D& operator- (Point3D& other);

public:
    double x, y, z;
};


#endif

