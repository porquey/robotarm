#include "Point3D.h"

Point3D::Point3D()
: x(0)
, y(0)
, z(0)
{
    
}

Point3D::Point3D(double a, double b, double c)
: x(a)
, y(b)
, z(c)
{
    
}

Point3D& Point3D::operator= (const Point3D& other)
{
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
}

Point3D& Point3D::operator+ (Point3D& other)
{
    x = x + other.x;
    y = y + other.y;
    z = z + other.z;
    return *this;
}

Point3D& Point3D::operator- (Point3D& other)
{
    x = x - other.x;
    y = y - other.y;
    z = z - other.z;
    return *this;
}
