#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>

#include "JointPositions.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

#define MAX_ERROR 164

#define PI 3.14159265
#define HALF_PI 1.570796

using namespace std;
using namespace cv;

class ControlArm
{
public:
    class PIDControl
    {
    public:
        PIDControl();
        int update(double angle, double dest);
        void reset();
        double GetLastError();
    private:
        bool started;
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
    void CalculateLinkLengths(double &l0, double &l1, double &l2);
    void SetLinkLengths(double l0, double l1, double l2);
    void GetArmPose(double *angles);
    void GetCurrentPose(double *angles);
    void SendJointActuators(int diff0, int diff1, int diff2);
    void SetTarget(Point3f target);
    void SetFuzzyTarget(Point3f target);
    Point3f GetTarget();
    Point3f GetFuzzyTarget();
    bool UpdateArmPose(Point3f detected, double error0, double error1, double error2);
    void IncrementIteration();
    double GetError();
    void InitFuzzyController();
    void TerminateFuzzyController();
    
protected:
    void FindInverseKinematics();
    Point3f CalculateCompensationStep(Point3f detected);
    
private:
    
    double link0, link1, link2;
    Point3f jointPositions[4];
    double jointAngles[3];
    double currentAngles[3];
    Point3f targetPosition;
    Point3f fuzzyTarget;
    vector<FuzzyRule> fuzzySet;
    int it;
    bool startFuzzy;
};