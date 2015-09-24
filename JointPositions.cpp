#include "JointPositions.h"

/// ReprojectPoints
/// In: pt: 3D point
///     cameraMatrix1: camera matrix of camera 1
///     cameraMatrix2: camera matrix of camera 2
///     translation: translation matrix between cameras
/// Out: pt1: 2D point in camera 1
///      pt2: 2D point in camera 2
void ReprojectPoints(Point3f pt, Point2f &pt1, Point2f &pt2, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &translation)
{
    // extract constants from matrices
    double fx1 = cameraMatrix1.at<double>(0, 0);
    double cx1 = cameraMatrix1.at<double>(0, 2);
    double fy1 = cameraMatrix1.at<double>(1, 1);
    double cy1 = cameraMatrix1.at<double>(1, 2);
    double fx2 = cameraMatrix2.at<double>(0, 0);
    double cx2 = cameraMatrix2.at<double>(0, 2);
    double fy2 = cameraMatrix2.at<double>(1, 1);
    double cy2 = cameraMatrix2.at<double>(1, 2);
    double xTrans = translation.at<double>(0, 0);
    double yTrans = translation.at<double>(0, 1);
    double zTrans = translation.at<double>(0, 2);
    
    // calculate 2D points
    pt1.x = (pt.x * fx1) / (pt.z + zTrans) + cx1;
    pt1.y = -((pt.y) * fy1 - yTrans/2) / (pt.z + zTrans) + cy1;
    pt2.x = (pt.z * fx2) / (xTrans - pt.x) + cx2;
    pt2.y = -((pt.y) * fy2 + yTrans/2) / (xTrans - pt.x) + cy2;
    
}

/// ReprojectPoints
/// In: pt1: 2D point in camera 1
///     pt2: 2D point in camera 2
///     cameraMatrix1: camera matrix of camera 1
///     cameraMatrix2: camera matrix of camera 2
///     translation: translation matrix between cameras
/// Out: 3D point
Point3f Calculate3DPoint(Point2f pt1, Point2f pt2, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &translation){
    
    // extract constants from matrices
    double fx1 = cameraMatrix1.at<double>(0, 0);
    double cx1 = cameraMatrix1.at<double>(0, 2);
    double fy1 = cameraMatrix1.at<double>(1, 1);
    double cy1 = cameraMatrix1.at<double>(1, 2);
    double fx2 = cameraMatrix2.at<double>(0, 0);
    double cx2 = cameraMatrix2.at<double>(0, 2);
    double fy2 = cameraMatrix2.at<double>(1, 1);
    double cy2 = cameraMatrix2.at<double>(1, 2);
    double xTrans = translation.at<double>(0, 0);
    double yTrans = translation.at<double>(0, 1);
    double zTrans = translation.at<double>(0, 2);
    
    // calculate 3D point
    double x1 = pt1.x - cx1;
    double x2 = pt2.x - cx2;
    double y1 = pt1.y - cy1;
    double y2 = pt2.y - cy2;
    double a = (x1)/fx1;
    double b = (x2)/fx2;
    double xPos = (b*xTrans + zTrans)/(1/a+b);
    double zPos = (xTrans - a*zTrans)/(1/b+a);
    double yPos = -(y1/fy1 * (zPos+zTrans) - yTrans + y2/fy2 * (-xPos+xTrans))/2;
    return Point3f(xPos, yPos, zPos);
}

/// CalculateDisplacement
/// In: a, b: 2D points
/// Out: displacement
double CalculateDisplacement(Point a, Point b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

/// CalculateAngle
/// In: a, b: 3D vectors
/// Out: angle
double CalculateAngle(Point3f a, Point3f b)
{
    // find length of each vector
    double mag1 = CalculateLength(a);
    double mag2 = CalculateLength(b);
    if(mag1 == 0 || mag2 == 0)
    {
        return 0;
    }
    
    // calculate angle
    double angle = acos((a.x * b.x + a.y * b.y + a.z * b.z)/(mag1 * mag2));
    return angle;
}

/// CalculateVector
/// In: a, b: 3D points
/// Out: 3D vector
Point3f CalculateVector(Point3f a, Point3f b)
{
    return a - b;
}

/// CalculateDotProduct
/// In: a, b: 3D vectors
/// Out: dot product
double CalculateDotProduct(Point3f a, Point3f b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// CalculateCrossProduct
/// In: a, b: 3D vectors
/// Out: cross product
Point3f CalculateCrossProduct(Point3f a, Point3f b)
{
    return Point3f(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

/// FindAngleDirection
/// In: baseVector, link1, link2: link vectors
/// Out: direction of angle (0 if straight)
int FindAngleDirection(Point3f baseVector, Point3f link1, Point3f link2)
{
    Point3f cross = CalculateCrossProduct(baseVector, CalculateCrossProduct(link1, link2));
    if(cross.y > 0)
    {
        return 1;
    }
    else if(cross.y < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

/// CalculateLength
/// In: a: 3D vector
/// Out: length
double CalculateLength(Point3f a)
{
    return sqrt(CalculateDotProduct(a, a));
}

/// CalculateUnitVector
/// In: a, b: 3D points
/// Out: unit vector
Point3f CalculateUnitVector(Point3f a, Point3f b){
    Point3f c = CalculateVector(a, b);
    double length = CalculateLength(a);
    c.x = c.x/length;
    c.y = c.y/length;
    c.z = c.z/length;
    return c;
}

/// MultiplyVector
/// In: a: 3D vector
///     length: constant
/// Out: vector
Point3f MultiplyVector(Point3f a, double length){
    a.x = a.x*length;
    a.y = a.y*length;
    a.z = a.z*length;
    return a;
}

/// Convert3fTo2f
/// In: a: 3D point
/// Out: 2D point
Point2f Convert3fTo2f(Point3f a){
    Point2f b;
    if ((a.x > 0 && a.z > 0) || (a.x < 0 && a.z < 0))
        b.x = sqrt(a.x*a.x + a.z*a.z);
    else
        b.x = -sqrt(a.x*a.x + a.z*a.z);
    
    b.y = a.y;
    
    return b;
}

// was used for GetStripVectors in BlobHueDetector, which is no longer used
/*
void DetermineBasePairs(KeyPoint *random, KeyPoint *sorted)
{
    KeyPoint a = random[0];
    KeyPoint b = random[1];
    KeyPoint c = random[2];
    KeyPoint d = random[3];
    
    if(a.pt.y > b.pt.y)
    {
        sorted[0] = a;
        sorted[1] = b;
    }
    else
    {
        sorted[0] = b;
        sorted[1] = a;
    }
    if(c.pt.y > d.pt.y)
    {
        sorted[2] = c;
        sorted[3] = d;
    }
    else
    {
        sorted[2] = d;
        sorted[3] = c;
    }
}


void DeterminePairs(KeyPoint *random, KeyPoint *sorted, Point pt1, Point pt2)
{
    KeyPoint a = random[0];
    KeyPoint b = random[1];
    KeyPoint c = random[2];
    KeyPoint d = random[3];
    
    if(CalculateDisplacement(a.pt, pt1) < CalculateDisplacement(b.pt, pt1))
    {
        sorted[0] = a;
        sorted[1] = b;
    }
    else
    {
        sorted[0] = b;
        sorted[1] = a;
    }
    if(CalculateDisplacement(c.pt, pt1) < CalculateDisplacement(d.pt, pt1))
    {
        sorted[2] = c;
        sorted[3] = d;
    }
    else
    {
        sorted[2] = d;
        sorted[3] = c;
    }
}

void CalculateJoint(Point3f *link1, Point3f *link2, Point3f& joint, double &angle)
{
    Point3f vec1 = CalculateVector(link1[0], link1[1]);
    Point3f vec2 = CalculateVector(link2[1], link2[0]);
    angle = CalculateAngle(vec1, vec2);
    if(angle == 0)
    {
        joint = link1[0];
        return;
    }
    
    double v1v2 = CalculateDotProduct(vec1, vec2);
    double v1v1 = CalculateDotProduct(vec1, vec1);
    double v2v2 = CalculateDotProduct(vec2, vec2);
    double p1v2 = CalculateDotProduct(link1[0], vec2);
    double p2v2 = CalculateDotProduct(link2[1], vec2);
    double p1v1 = CalculateDotProduct(link1[0], vec1);
    double p2v1 = CalculateDotProduct(link2[1], vec1);
    
    
    double s = (v1v1 * (p1v2 - p2v2) + v1v2 * (p2v1 - p1v1)) / (v2v2 * v1v1 - v1v2 * v1v2);
    double t = (p2v1 - p1v1 + s * v1v2) / v1v1;
    Point3f pt1 = link1[0] + t * vec1;
    Point3f pt2 = link2[1] + s * vec2;
    joint = (pt1 + pt2) * 0.5;
}
*/
