#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>

#include "JointPositions.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

#define MAX_ERROR 164
#define MAX_INTEGRAL 1000

#define PI 3.14159265
#define HALF_PI 1.570796

using namespace std;
using namespace cv;

/// ControlArm class controls the angles of the joints to navigate an arm to a stationary or
/// moving target, using PI controllers and a fuzzy position error compensator
class ControlArm
{
public:
    /// PIDControl class implements a controller for each joint to set the angles accurately
    class PIDControl
    {
    public:
        // class constructor
        PIDControl();
        // updates the PI controller
        int Update(double value, double dest);
        // resets the controller
        void Reset();
        // gets the error
        double GetLastError();
        
    private:
        // class members
        bool started;
        double lastError;
        double integral;
        double Kp;
        double Ki;
        double Kd;
    };
    
public:
    /// FuzzyRule class implements a fuzzy rule for the fuzzy position error compensator
    class FuzzyRule
    {
    public:
        // class constructors
        FuzzyRule();
        FuzzyRule(Point3f a, Point3f b, Point3f c, Point3f w);
        FuzzyRule(FuzzyRule const &rule);
        // gets fuzzy rule weighting
        Point3f GetWeighting(Point3f error);
    protected:
        // class members
        Point3f min, centre, max;
        Point3f weighting;
    };
    
public:
    // class constructors
    ControlArm();
    ControlArm(double l0, double l1, double l2);
    // sets the arm pose from joint positions
    void SetArmPose(vector<Point3f> joints);
    // calculates the link lengths
    void CalculateLinkLengths(double &l0, double &l1, double &l2);
    // sets new link length values
    void SetLinkLengths(double l0, double l1, double l2);
    // gets the new joint angles
    void GetArmPose(double *angles);
    // gets the current joint angles
    void GetCurrentPose(double *angles);
    // sends the actuation pressure values
    void SendJointActuators(int diff0, int diff1, int diff2);
    // sets the target position
    void SetTarget(Point3f target);
    // sets the target fuzzy position
    void SetFuzzyTarget(Point3f target);
    // gets the target position
    Point3f GetTarget();
    // gets the fuzzy target position
    Point3f GetFuzzyTarget();
    // updates the fuzzy target position based on target position error
    bool UpdateArmPose(double error0, double error1, double error2);
    // gets the target position error
    double GetError();
    // initialises the fuzzy position error compensator
    void InitFuzzyController();
    // terminates the fuzzy position error compensator
    void TerminateFuzzyController();
    
protected:
    // calculates the new joint angles using inverse kinematics
    void FindInverseKinematics();
    // calculates the compensation step from the fuzzy rules and the error
    Point3f CalculateCompensationStep();
    
private:
    // class members
    double link0, link1, link2;
    Point3f jointPositions[4];
    double jointAngles[3];
    double lastJointAngles[3];
    double jointOffsets[3];
    double currentAngles[3];
    Point3f targetPosition;
    Point3f fuzzyTarget;
    vector<FuzzyRule> fuzzySet;
    bool startFuzzy;
    Point3f targetLast;
};