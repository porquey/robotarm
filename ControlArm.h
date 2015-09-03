#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>

#include "JointPositions.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace std;
using namespace cv;

class ControlArm
{
public:
    class PIDControl
    {
    public:
        PIDControl();
        double update(double angle, double dest);
        void reset();
        
    private:
        double lastError;
        double integral;
        double Kp;
        double Ki;
        double Kd;
    };
    
public:
    class FuzzyRule
    {
    public:
        FuzzyRule();
        FuzzyRule(Point3f a, Point3f b, Point3f c, Point3f w);
        FuzzyRule(FuzzyRule const &rule);
        void SetValues(Point3f a, Point3f b, Point3f c, Point3f w);
        Point3f GetWeighting(Point3f error);
    protected:
        Point3f min, centre, max;
        Point3f weighting;
    };
    
public:
    ControlArm();
    ControlArm(double l0, double l1, double l2);
    void SetArmPose(vector<Point3f> joints);
    void CalculateLinkLengths();
    void SetLinkLengths(double l0, double l1, double l2);
    void GetArmPose(double *angles);
    void GetCurrentPose(double *angles);
    void SendJointActuators(int diff0, int diff1, int diff2);
    void SetTarget(Point3f target);
    void UpdateArmPose(Point3f detected);
    void InitFuzzyController();
    
protected:
    void FindInverseKinematics();
    Point3f CalculateCompensationStep(Point3f detected);
    
private:
    
    double link0, link1, link2;
    Point3f jointPositions[5];
    double jointAngles[3];
    double currentAngles[3];
    Point3f targetPosition;
    vector<FuzzyRule> fuzzySet;
    
    
};