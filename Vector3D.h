#include <stdio.h>

class Vector3D
{
public:
    Vector3D();
    Vector3D(double a, double b, double c);
    Vector3D& operator= (const Vector3D& other);
    Vector3D& operator+ (Vector3D& other);
    Vector3D& operator- (Vector3D& other);

public:
    double x, y, z;
    
};
