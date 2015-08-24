#include "Vector3D.h"

Vector3D::Vector3D()
: x(0)
, y(0)
, z(0)
{
    
}

Vector3D::Vector3D(double a, double b, double c)
: x(a)
, y(b)
, z(c)
{
    
}

Vector3D& Vector3D::operator= (const Vector3D& other)
{
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
}

Vector3D& Vector3D::operator+ (Vector3D& other)
{
    x = x + other.x;
    y = y + other.y;
    z = z + other.z;
    return *this;
}

Vector3D& Vector3D::operator- (Vector3D& other)
{
    x = x - other.x;
    y = y - other.y;
    z = z - other.z;
    return *this;
}
