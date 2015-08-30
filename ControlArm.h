#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>

#include "Point3D.h"
#include "Vector3D.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace std;
using namespace cv;

class ControlArm
{
public:
    class FuzzyRule
    {
    public:
        FuzzyRule();
        FuzzyRule(Point3D a, Point3D b, Point3D c, Point3D w);
        FuzzyRule(FuzzyRule const &rule);
        void SetValues(Point3D a, Point3D b, Point3D c, Point3D w);
        Point3D GetWeighting(Point3D error);
    protected:
        Point3D min, centre, max;
        Point3D weighting;
    };
    
public:
    ControlArm();
    ControlArm(double l0, double l1, double l2);
    void SetArmPose(vector<Point3D> joints);
    void CalculateLinkLengths();
    void GetArmPose(double angles[3]);
    void GetCurrentPose(double angles[3]);
    void SetTarget(Point3D target);
    void UpdateArmPose(Point3D detected);
    void InitFuzzyController();

protected:
    void FindInverseKinematics();
    double CalculateAngle(Vector3D a, Vector3D b);
    Vector3D CalculateVector(Point3D a, Point3D b);
    double CalculateLength(Vector3D a);
    Vector3D PointToVec(const Point3D a);
    Point3D CalculateCompensationStep(Point3D detected);

private:

    double link0, link1, link2;
    Point3D jointPositions[5];
    double jointAngles[3];
    double currentAngles[3];
    Point3D targetPosition;
    vector<FuzzyRule> fuzzySet;
    
    
};