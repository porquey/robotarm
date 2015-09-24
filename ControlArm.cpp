#include "ControlArm.h"

/// ControlArm basic constructor
ControlArm::ControlArm()
{
    startFuzzy = false;
}

/// ControlArm link lengths constructor
ControlArm::ControlArm(double l0, double l1, double l2) :
link0(l0),
link1(l1),
link2(l2)
{
    startFuzzy = false;
}

/// SetArmPose
/// In: joints: vector of joint positions
void ControlArm::SetArmPose(vector<Point3f> joints)
{
    // set the current joint positions
    if(joints.size() == 4)
    {
        for(int i = 0; i < 4; i++)
        {
            jointPositions[i] = joints[i];
        }
    }
}

/// CalculateLinkLengths
/// Out: l0: base link length
///      l1: link 1 length
///      l2: link 2 length
void ControlArm::CalculateLinkLengths(double &l0, double &l1, double &l2)
{
    // detect and calculate link lengths
    link0 = CalculateLength(CalculateVector(jointPositions[1], jointPositions[0]));
    link1 = CalculateLength(CalculateVector(jointPositions[2], jointPositions[1]));
    link2 = CalculateLength(CalculateVector(jointPositions[3], jointPositions[2]));
    l0 = link0;
    l1 = link1;
    l2 = link2;
    cerr << "Link 0: " << link0 << " Link 1: " << link1 << " Link 2: " << link2 << endl;
}

/// SetLinkLengths
/// In: l0: base link length
///     l1: link 1 length
///     l2: link 2 length
void ControlArm::SetLinkLengths(double l0, double l1, double l2)
{
    link0 = l0;
    link1 = l1;
    link2 = l2;
}

/// GetArmPose
/// Out: angles: array of desired joint angles
void ControlArm::GetArmPose(double *angles)
{
    angles[0] = jointAngles[0];
    angles[1] = jointAngles[1];
    angles[2] = jointAngles[2];
}

/// GetCurrentPose
/// Out: angles: array of current joint angles
void ControlArm::GetCurrentPose(double *angles)
{
    // define vectors
    Point3f baseVector = jointPositions[0] - jointPositions[2];
    Point3f vector1 = jointPositions[1] - jointPositions[2];
    Point3f vector2 = jointPositions[3] - jointPositions[2];

    double x = baseVector.x;
    double z = baseVector.z;
    
    // calculate angles
    currentAngles[0] = abs(atan2(z, x));
    currentAngles[1] = CalculateAngle(CalculateVector(jointPositions[1], jointPositions[0]), CalculateVector(jointPositions[2], jointPositions[1]));
    currentAngles[2] = CalculateAngle(CalculateVector(jointPositions[2], jointPositions[1]), CalculateVector(jointPositions[3], jointPositions[2]));
    
    // if joint 2 is further away from camera 1 than joint 1, flip angles
    if (jointPositions[2].z > jointPositions[1].z){
        currentAngles[1] = -currentAngles[1];
        currentAngles[0] = -currentAngles[0];
    }
    
    // calculate cross product to find direction of last joint angle
    Point3f cross = CalculateCrossProduct(vector1, vector2);
    if(cross.x < 0){
        currentAngles[2] = -currentAngles[2];
    }
    
    // calculate joint angle offset if it crosses the axis
    if (lastJointAngles[0] != 0 && (currentAngles[0] * lastJointAngles[0]) < 0)
        jointOffsets[0] = abs(lastJointAngles[0]) < 0.4 ? abs(lastJointAngles[0]) : 0;
    if (lastJointAngles[1] != 0 && (currentAngles[1] * lastJointAngles[1]) < 0)
        jointOffsets[1] = abs(lastJointAngles[1]) < 0.4 ? abs(lastJointAngles[1]) : 0;
    if (lastJointAngles[2] != 0 && (currentAngles[2] * lastJointAngles[2]) < 0)
        jointOffsets[2] = abs(lastJointAngles[2]) < 0.4 ? abs(lastJointAngles[2]) : 0;
    
    // update last joint angles
    lastJointAngles[0] = currentAngles[0];
    lastJointAngles[1] = currentAngles[1];
    lastJointAngles[2] = currentAngles[2];
    
    // return current angles
    angles[0] = currentAngles[0];
    angles[1] = currentAngles[1];
    angles[2] = currentAngles[2];

}

/// SetTarget
/// In: target: target position
void ControlArm::SetTarget(Point3f target)
{
    // set target and reposition target if it is too far away
    targetPosition = target;
    if(CalculateLength(CalculateVector(targetPosition, jointPositions[1])) > link1 + link2)
    {
        Point3f pointDiff = targetPosition - jointPositions[1];
        double absDiff = CalculateLength(pointDiff);
        targetPosition.x = (int)((pointDiff.x / absDiff) * (link1 + link2));
        targetPosition.y = (int)((pointDiff.y / absDiff) * (link1 + link2));
        targetPosition.z = (int)((pointDiff.z / absDiff) * (link1 + link2));
        targetPosition = targetPosition + jointPositions[1];
    }
    
    // calculate inverse kinematics
    FindInverseKinematics();
}

/// SetFuzzyTarget
/// In: target: fuzzy target position
void ControlArm::SetFuzzyTarget(Point3f target)
{
    // set fuzzy target and reposition if too far away
    fuzzyTarget = target;
    if(CalculateLength(CalculateVector(fuzzyTarget, jointPositions[1])) > link1 + link2)
    {
        Point3f pointDiff = fuzzyTarget - jointPositions[1];
        double absDiff = CalculateLength(pointDiff);
        fuzzyTarget.x = (int)((pointDiff.x / absDiff) * (link1 + link2));
        fuzzyTarget.y = (int)((pointDiff.y / absDiff) * (link1 + link2));
        fuzzyTarget.z = (int)((pointDiff.z / absDiff) * (link1 + link2));
        fuzzyTarget = fuzzyTarget + jointPositions[1];
    }
    
    // calculate inverse kinematics
    FindInverseKinematics();
}

/// GetTarget
/// Out: target position
Point3f ControlArm::GetTarget()
{
    return targetPosition;
}

/// GetTarget
/// Out: fuzzy target position
Point3f ControlArm::GetFuzzyTarget()
{
    // return original target if fuzzy controller is not running
    if(startFuzzy)
    {
        return fuzzyTarget;
    }
    else
    {
        return targetPosition;
    }
}

/// UpdateArmPose
/// In: detected: detected end effector position
///     error0: base joint PI controller error
///     error1: joint 1 PI controller error
///     error2: joint 2 PI controller error
bool ControlArm::UpdateArmPose(double error0, double error1, double error2)
{
    if(startFuzzy)
    {
        // adjust the PI controller error threshold before fuzzy position error compensation
        // is performed
        double e = CalculateLength(CalculateVector(jointPositions[3], targetPosition));
        double k;
        if(e > 30)
        {
            k = 0.5;
            cerr << "KKK" << endl;
        }
        else if(e > 10)
        {
            k = 0.25;
            cerr << "KK" << endl;
        }
        else
        {
            k = 0.1;
            cerr << "K" << endl;
        }
    }
    
    // if target is moving adjust fuzzy target position
    if(CalculateLength(fuzzyTarget - targetPosition) > 10 && CalculateLength(targetLast - targetPosition) > 1)
    {
        fuzzyTarget = targetPosition;
        targetLast = targetPosition;
        return false;
    }
    
    // update last target position
    if(CalculateLength(targetLast - targetPosition) > 1)
    {
        targetLast = targetPosition;
    }
    
    if(startFuzzy)
    {
        // do nothing if target is reached within desired accuracy
        if(CalculateLength(jointPositions[3] - targetPosition) < 5)
        {
            cerr << "Reached distance of " << CalculateLength(jointPositions[3] - targetPosition) << " from target" << endl;
            cerr << "Target reached" << endl;
            return false;
        }
        // apply compensation step if target is not reached and PI controllers have steadied
        else if(abs(error0) <= 0.2 && abs(error1) <= 0.2 && abs(error2) <= 0.2)
        {
            SetFuzzyTarget(fuzzyTarget - CalculateCompensationStep());
            cerr << "Reached distance of " << CalculateLength(jointPositions[3] - targetPosition) << " from target" << endl;
            cerr << "New target is " << fuzzyTarget << endl;
            return true;
        }
        else
        {
            cerr << "Reached distance of " << CalculateLength(jointPositions[3] - targetPosition) << " from target" << endl;
            return true;
        }
    }
    // return if fuzzy controller is not running
    else
    {
        return false;
    }
}

/// SendJointActuators
/// In: diff0: pressure value for base joint
///     diff1: pressure value for joint 1
///     diff2: pressure value for joint 2
void ControlArm::SendJointActuators(int diff0, int diff1, int diff2){
    cout << diff0 << " " << diff1 << " " << diff2 << endl;
}

/// GetError
/// Out: absolute error
double ControlArm::GetError()
{
    return CalculateLength(CalculateVector(targetPosition, jointPositions[3]));
}

/// FindInverseKinematics
void ControlArm::FindInverseKinematics()
{
    double tempAngles[3];
    Point3f temp;
    
    // set target
    if(startFuzzy)
    {
        temp = fuzzyTarget;
    }
    else
    {
        temp = targetPosition;
    }
    
    // intermediate variables
    double r, s, d;
    double a = temp.x - jointPositions[0].x;
    double b = temp.z - jointPositions[0].z;
    
    // calculate base joint angle
    tempAngles[0] = atan2(b, a) + PI;
    if(tempAngles[0] > PI)
    {
        tempAngles[0] -= 2 * PI;
    }
    else if(tempAngles[0] < -PI)
    {
        tempAngles[0] += 2 * PI;
    }

    double x = temp.x - jointPositions[1].x;
    double z = temp.z - jointPositions[1].z;
    r = sqrt(x * x + z * z);
    s = temp.y - jointPositions[1].y;
    d = (r * r + s * s - link2 * link2 - link1 * link1) / (2 * link1 * link2);
    
    // calculate joint 2 angle
    tempAngles[2] = atan2(-sqrt(1 - d * d), d);
    
    double k1 = link1 + link2 * cos(tempAngles[2]);
    double k2 = link2 * sin(tempAngles[2]);

    // calculate joint 1 angle
    tempAngles[1] = atan2(s, r) - atan2(k2, k1);
    
    // adjust angles according to offsets
    tempAngles[2] = -tempAngles[2];
    tempAngles[1] = HALF_PI - tempAngles[1];
    
    // if base joint is rotated away from camera 1, flip other joint angles
    if (tempAngles[0] <= 0)
    {
        tempAngles[1] = -tempAngles[1];
        tempAngles[2] = -tempAngles[2];
    }

    // update desired joint angles
    if(tempAngles[0] == tempAngles[0])
    {
        jointAngles[0] = tempAngles[0];
    }
    if(tempAngles[1] == tempAngles[1])
    {
        jointAngles[1] = tempAngles[1];
    }
    if(tempAngles[2] == tempAngles[2])
    {
        jointAngles[2] = tempAngles[2];
    }
    
}

/// InitFuzzyController
void ControlArm::InitFuzzyController()
{
    // initialise and set up 5 fuzzy rules
    startFuzzy = true;
    fuzzySet.clear();
    
    // High Negative rule
    fuzzySet.push_back(FuzzyRule(Point3f(-100000, -100000, -100000), Point3f(-5, -5, -5), Point3f(-2.5, -2.5, -2.5), Point3f(-1.5, -1.5, -1.5)));
    // Low Negative rule
    fuzzySet.push_back(FuzzyRule(Point3f(-10, -10, -10), Point3f(-5, -5, -5), Point3f(0, 0, 0), Point3f(-0.4, -0.4, -0.4)));
    // Zero rule
    fuzzySet.push_back(FuzzyRule(Point3f(-5, -5, -5), Point3f(0, 0, 0), Point3f(5, 5, 5), Point3f(0, 0, 0)));
    // Low Positive rule
    fuzzySet.push_back(FuzzyRule(Point3f(0, 0, 0), Point3f(5, 5, 5), Point3f(10, 10, 10), Point3f(0.4, 0.4, 0.4)));
    // High Positive rule
    fuzzySet.push_back(FuzzyRule(Point3f(5, 5, 5), Point3f(10, 10, 10), Point3f(100000, 100000, 100000), Point3f(1.5, 1.5, 1.5)));
    
    // set fuzzy target
    SetFuzzyTarget(targetPosition);
}

/// TerminateFuzzyController
void ControlArm::TerminateFuzzyController()
{
    startFuzzy = false;
}

/// CalculateCompensationStep
/// Out: compensation step
Point3f ControlArm::CalculateCompensationStep()
{
    // calculate error between end effector and target
    Point3f error = jointPositions[3] - targetPosition;
    
    //calculate compensation step from fuzzy rules
    Point3f kSum;
    for(int i = 0; i < 5; i++)
    {
        Point3f k = fuzzySet[i].GetWeighting(error);
        kSum = kSum + k;
    }
    return kSum;
}

//----------------------------PIDControl---------------------------------//

/// PIDControl constructor
ControlArm::PIDControl::PIDControl()
{
    // initial values
    integral = 0;
    lastError = 0;
    started = false;
    Kp=500;
    Ki=25;
    Kd=0;
}

/// Update
/// In: value: current angle
///     dest: target angle
/// Out: pressure value to be sent to the DAC
int ControlArm::PIDControl::Update(double value, double dest)
{
    // calculate the error
    double error = value - dest;
    if (abs(error) > 100000)
    {
        return 0;
    }
    
    // start controller
    if (abs(error-lastError) > 1 && started)
    {
        error = lastError;
        started = true;
    }
    
    // adjust the terms
    double derivative = error - lastError;
    lastError = error;
    integral += error;
    if (integral < -5 && error > 0)
    {
        integral = 0;
    }
    if (integral > 5 && error < 0)
    {
        integral = 0;
    }
    
    // calculate output
    double out = (error * Kp + integral * Ki + derivative + Kd + 0.5);
    if (out > MAX_ERROR)
    {
        return (int)MAX_ERROR;
    }
    else if (out < -MAX_ERROR)
    {
        return -(int)MAX_ERROR;
    }
    else
    {
        return (int)out;
    }
}

/// Reset
void ControlArm::PIDControl::Reset()
{
    // initialise variables
    started = false;
    integral = 0;
    lastError = 0;
}

/// GetLastError
/// Out: last calculated error
double ControlArm::PIDControl::GetLastError()
{
    return lastError;
}

//--------------------------------------FuzzyRule-----------------------------------//

/// FuzzyRule basic constructor
ControlArm::FuzzyRule::FuzzyRule()
: min(Point3f())
, centre(Point3f())
, max(Point3f())
, weighting(Point3f())
{
    
}

/// FuzzyRule set values constructor
ControlArm::FuzzyRule::FuzzyRule(Point3f a, Point3f b, Point3f c, Point3f w)
: min(a)
, centre(b)
, max(c)
, weighting(w)
{
    
}

/// FuzzyRule copy constructor
ControlArm::FuzzyRule::FuzzyRule(FuzzyRule const& rule)
: min(rule.min)
, centre(rule.centre)
, max(rule.max)
, weighting(rule.weighting)
{
    
}

/// GetWeighting
/// In: error: position error
/// Out: weighting for the fuzzy rule
Point3f ControlArm::FuzzyRule::GetWeighting(Point3f error)
{
    Point3f k;
    
    // weighting in the X axis
    if(error.x < max.x && error.x >= centre.x)
    {
        k.x = weighting.x * (max.x - error.x) / (max.x - centre.x);
    }
    else if(error.x > min.x && error.x < centre.x)
    {
        k.x = weighting.x * (error.x - min.x) / (centre.x - min.x);
    }
    
    // weighting in the Y axis
    if(error.y < max.y && error.y >= centre.y)
    {
        k.y = weighting.y * (max.y - error.y) / (max.y - centre.y);
    }
    else if(error.y > min.y && error.y < centre.y)
    {
        k.y = weighting.y * (error.y - min.y) / (centre.y - min.y);
    }
    
    // weighting in the Z axis
    if(error.z < max.z && error.z >= centre.z)
    {
        k.z = weighting.z * (max.z - error.z) / (max.z - centre.z);
    }
    else if(error.z > min.z && error.z < centre.z)
    {
        k.z = weighting.z * (error.z - min.z) / (centre.z - min.z);
    }
    return k;
}


