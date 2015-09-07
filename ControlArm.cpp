#include "ControlArm.h"

ControlArm::ControlArm()
{
    startFuzzy = false;
}

ControlArm::ControlArm(double l0, double l1, double l2) :
link0(l0),
link1(l1),
link2(l2)
{
    startFuzzy = false;
}

ControlArm::PIDControl::PIDControl()
{
    integral = 0;
    lastError = 0;
    started = false;
    Kp=400;
    Ki=30;
    Kd=0;
}

int ControlArm::PIDControl::update(double value, double dest)
{
    double error = value - dest;
    
    if (abs(error-lastError) > 1 && started){
        error = lastError;
        started = true;
    }
    double derivative = error - lastError;
    
    lastError = error;
    
    double out = (error * Kp + integral * Ki + derivative + Kd + 0.5);
    
    //cerr << "Error: " << error << endl;
        
    if (out > MAX_ERROR){
        return (int)MAX_ERROR;
    }
    else if (out < -MAX_ERROR){
        return -(int)MAX_ERROR;
    }
    else{
        integral+= error;
        return (int)out;
    }
}

void ControlArm::PIDControl::reset()
{
    started = false;
    integral = 0;
    lastError = 0;
}

double ControlArm::PIDControl::GetLastError()
{
    return lastError;
}

void ControlArm::SetArmPose(vector<Point3f> joints)
{
    if(joints.size() == 4)
    {
        for(int i = 0; i < 4; i++)
        {
            jointPositions[i] = joints[i];
            //cerr << "Joint " << i << " : " << jointPositions[i] << endl;
        }
    }
}

void ControlArm::CalculateLinkLengths(double &l0, double &l1, double &l2)
{
    link0 = CalculateLength(CalculateVector(jointPositions[1], jointPositions[0]));
    link1 = CalculateLength(CalculateVector(jointPositions[2], jointPositions[1]));
    link2 = CalculateLength(CalculateVector(jointPositions[3], jointPositions[2]));
    l0 = link0;
    l1 = link1;
    l2 = link2;
    cerr << "Link 0: " << link0 << " Link 1: " << link1 << " Link 2: " << link2 << endl;
}

void ControlArm::SetLinkLengths(double l0, double l1, double l2)
{
    link0 = l0;
    link1 = l1;
    link2 = l2;
}


void ControlArm::GetArmPose(double *angles)
{
    angles[0] = jointAngles[0];
    angles[1] = jointAngles[1];
    angles[2] = jointAngles[2];
}

void ControlArm::GetCurrentPose(double *angles)
{
    Point3f vector1 = jointPositions[2] - jointPositions[0];
    vector1.y = 0;
    Point3f vector2 = jointPositions[3] - jointPositions[0];
    vector2.y = 0;
    Point3f baseVector;
    if(CalculateLength(vector1) > CalculateLength(vector2))
    {
        baseVector = vector1;
    }
    else
    {
        baseVector = vector2;
    }

    
    double x = baseVector.x;
    double z = baseVector.z;
    
    currentAngles[0] = atan2(z, x) + HALF_PI;
    if(currentAngles[0] > PI)
    {
        currentAngles[0] -= 2 * PI;
    }
    else if(currentAngles[0] < -PI)
    {
        currentAngles[0] += 2 * PI;
    }
    
    currentAngles[1] = CalculateAngle(CalculateVector(jointPositions[1], jointPositions[0]), CalculateVector(jointPositions[2], jointPositions[1]));
    
    currentAngles[2] = 3.14159 - CalculateAngle(CalculateVector(jointPositions[2], jointPositions[1]), CalculateVector(jointPositions[2], jointPositions[3]));
    
    //If joint 2 is further away from camera 1 than joint 1
    if (jointPositions[2].z > jointPositions[1].z){
        currentAngles[1] = -currentAngles[1];
        currentAngles[0] += PI;
        
        if(currentAngles[0] > PI)
        {
            currentAngles[0] -= 2 * PI;
        }
        else if(currentAngles[0] < -PI)
        {
            currentAngles[0] += 2 * PI;
        }
        
    }
    
//    if (currentAngles[2] < 0.3){
//        Point3f projected3D =CalculateVector(jointPositions[1], jointPositions[3]);
//        Point3f tip3D = jointPositions[3] - jointPositions[2];
//
//        Point2f projected2D, tip2D;
//        
//        projected2D = Convert3fTo2f(projected3D);
//        tip2D = Convert3fTo2f(tip3D);
//        
//        float angle = atan2(projected2D.x,projected2D.y);
//        
//        float tipY = cos(angle) * tip2D.y - sin(angle) * tip2D.x;
//        
//        if ((tip2D.x > 0 && tipY > 0) || (tipY > 0 && tip2D.x < 0 && currentAngles[1] < 0)){
//            currentAngles[2] = -currentAngles[2];
//        }
//    }
    
    //cerr << "Link 0: " << link0 << " Link 1: " << link1 << " Link 2: " << link2 << endl;
    //cerr << "Angle 0 " << currentAngles[0] << " Angle 1 " << currentAngles[1] << " Angle 2 " << currentAngles[2] << endl;
    
    int dir = FindAngleDirection(baseVector, jointPositions[1] - jointPositions[2], jointPositions[3] - jointPositions[2]);
    
    if ((dir > 0 && currentAngles[1] > 0) || (dir < 0 && currentAngles[1] < 0)){
        currentAngles[2] = -currentAngles[2];
    }

    
    
    angles[0] = currentAngles[0];
    angles[1] = currentAngles[1];
    angles[2] = currentAngles[2];
}

void ControlArm::SetTarget(Point3f target)
{
    targetPosition = target;
    //cerr << "TARGET: " << targetPosition.x << " " << targetPosition.y << " " << targetPosition.z << endl;
    
    if(CalculateLength(CalculateVector(targetPosition, jointPositions[1])) > link1 + link2)
    {
        Point3f pointDiff = targetPosition - jointPositions[1];
        double absDiff = CalculateLength(pointDiff);
        
        targetPosition.x = (int)(pointDiff.x / absDiff) * (link1 + link2 - 1);
        targetPosition.y = (int)(pointDiff.y / absDiff) * (link1 + link2 - 1);
        targetPosition.z = (int)(pointDiff.z / absDiff) * (link1 + link2 - 1);
        targetPosition = targetPosition + jointPositions[1];
        
        //cerr << "OUT OF REACH. NEW TARGET: " << targetPosition.x << " " << targetPosition.y << " " << targetPosition.z << endl;
    }
    FindInverseKinematics();
    //cerr << "NEW ANGLE0: " << jointAngles[0] << " ANGLE1: " << jointAngles[1] << " ANGLE2: " << jointAngles[2] << endl;
}

void ControlArm::SetFuzzyTarget(Point3f target)
{
    fuzzyTarget = target;
    //cerr << "TARGET: " << targetPosition.x << " " << targetPosition.y << " " << targetPosition.z << endl;
    
    if(CalculateLength(CalculateVector(fuzzyTarget, jointPositions[1])) > link1 + link2)
    {
        Point3f pointDiff = fuzzyTarget - jointPositions[1];
        double absDiff = CalculateLength(pointDiff);
        
        fuzzyTarget.x = (int)(pointDiff.x / absDiff) * (link1 + link2 - 1);
        fuzzyTarget.y = (int)(pointDiff.y / absDiff) * (link1 + link2 - 1);
        fuzzyTarget.z = (int)(pointDiff.z / absDiff) * (link1 + link2 - 1);
        fuzzyTarget = fuzzyTarget + jointPositions[1];
        
        //cerr << "OUT OF REACH. NEW TARGET: " << targetPosition.x << " " << targetPosition.y << " " << targetPosition.z << endl;
    }
    FindInverseKinematics();
    //cerr << "NEW ANGLE0: " << jointAngles[0] << " ANGLE1: " << jointAngles[1] << " ANGLE2: " << jointAngles[2] << endl;
}

Point3f ControlArm::GetTarget()
{
    return targetPosition;
}

Point3f ControlArm::GetFuzzyTarget()
{
    return fuzzyTarget;
}

bool ControlArm::UpdateArmPose(Point3f detected, double error0, double error1, double error2)
{
    if(CalculateLength(jointPositions[3] - targetPosition) < 10)
    {
        cerr << "Reached distance of " << CalculateLength(jointPositions[3] - targetPosition) << " from target" << endl;
        cerr << "Target reached" << endl;
        return false;
    }
    else if(it > 10)
    {
        cerr << "Reached distance of " << CalculateLength(jointPositions[3] - targetPosition) << " from target" << endl;
        cerr << "Terminating after 10 iterations" << endl;
        return false;
    }
    else if(abs(error0) <= 0.1 && abs(error1) <= 0.1 && abs(error2) <= 0.1)
    {
        SetFuzzyTarget(fuzzyTarget - CalculateCompensationStep(detected));
        cerr << "Reached distance of " << CalculateLength(jointPositions[3] - targetPosition) << " from target" << endl;
        cerr << "New target is " << fuzzyTarget << endl;
        return true;
    }
    else
    {
        cerr << "Reached distance of " << CalculateLength(jointPositions[3] - targetPosition) << " from target" << endl;
        cerr << "Error0: " << error0 << " Error1: " << error1 << " Error2: " << error2 << endl;
        return true;
    }
}

void ControlArm::IncrementIteration()
{
    it++;
}

void ControlArm::SendJointActuators(int diff0, int diff1, int diff2){
    cout << diff0 << " " << diff1 << " " << diff2 << endl;
}

double ControlArm::GetError()
{
    cerr << "From Target: " << CalculateLength(CalculateVector(targetPosition, jointPositions[3])) << endl;
    cerr << "From Fuzzy Target: " << CalculateLength(CalculateVector(fuzzyTarget, jointPositions[3])) << endl;
    return CalculateLength(CalculateVector(targetPosition, jointPositions[3]));
}

void ControlArm::FindInverseKinematics()
{
    Point3f temp;
    if(startFuzzy)
    {
        temp = fuzzyTarget;
    }
    else
    {
        temp = targetPosition;
    }
    
    double r, s, d;
    
    double a = temp.x - jointPositions[0].x;
    double b = temp.z - jointPositions[0].z;
    
    jointAngles[0] = atan2(b, a) + HALF_PI;
    if(jointAngles[0] > PI)
    {
        jointAngles[0] -= 2 * PI;
    }
    else if(jointAngles[0] < -PI)
    {
        jointAngles[0] += 2 * PI;
    }

    double x = temp.x - jointPositions[1].x;
    double z = temp.z - jointPositions[1].z;
    
    r = sqrt(x * x + z * z);
    s = temp.y - jointPositions[1].y;
    
    d = (r * r + s * s - link2 * link2 - link1 * link1) / (2 * link1 * link2);
    
    jointAngles[2] = atan2(-sqrt(1 - d * d), d);
    //jointAngles[1] = atan2(sqrt(s), sqrt(r)) - atan2(sin(jointAngles[2]), link1 + link2 * cos(jointAngles[2]));
    
    double k1 = link1 + link2 * cos(jointAngles[2]);
    double k2 = link2 * sin(jointAngles[2]);

    jointAngles[1] = atan2(s, r) - atan2(k2, k1);
    
    jointAngles[2] = -jointAngles[2];
    jointAngles[1] = HALF_PI - jointAngles[1];
    
    
    if (temp.z > jointPositions[1].z){
        jointAngles[0] += PI;
        
        if(jointAngles[0] > PI)
        {
            jointAngles[0] -= 2 * PI;
        }
        else if(jointAngles[0] < -PI)
        {
            jointAngles[0] += 2 * PI;
        }
        
        jointAngles[1] = -jointAngles[1];
        jointAngles[2] = -jointAngles[2];
        cerr << "FLIPPED" << endl;
    }
    
    /*
    if(jointAngles[1] > 1.7)
    {
        jointAngles[1] = 1.7;
    }
    if(jointAngles[1] < -1.7)
    {
        jointAngles[1] = -1.7;
    }
    
    if(jointAngles[2] > 1.7)
    {
        jointAngles[2] = 1.7;
    }
    if(jointAngles[2] < -1.7)
    {
        jointAngles[2] = -1.7;
    }*/
    
        
}

void ControlArm::InitFuzzyController()
{
    it = 0;
    startFuzzy = true;
    fuzzySet.clear();
    fuzzySet.push_back(FuzzyRule(Point3f(-10000, -10000, -10000), Point3f(-5, -5, -5), Point3f(-2.5, -2.5, -2.5), Point3f(-0.75, -0.75, -0.75)));
    fuzzySet.push_back(FuzzyRule(Point3f(-5, -5, -5), Point3f(-2.5, -2.5, -2.5), Point3f(0, 0, 0), Point3f(-0.4, -0.4, -0.4)));
    fuzzySet.push_back(FuzzyRule(Point3f(-2.5, -2.5, -2.5), Point3f(0, 0, 0), Point3f(2.5, 2.5, 2.5), Point3f(0, 0, 0)));
    fuzzySet.push_back(FuzzyRule(Point3f(0, 0, 0), Point3f(2.5, 2.5, 2.5), Point3f(5, 5, 5), Point3f(0.4, 0.4, 0.4)));
    fuzzySet.push_back(FuzzyRule(Point3f(2.5, 2.5, 2.5), Point3f(5, 5, 5), Point3f(10000, 10000, 10000), Point3f(0.75, 0.75, 0.75)));
    SetFuzzyTarget(targetPosition);
}

Point3f ControlArm::CalculateCompensationStep(Point3f detected)
{
    Point3f error = detected - targetPosition;
    Point3f kSum;
    for(int i = 0; i < 5; i++)
    {
        Point3f k = fuzzySet[i].GetWeighting(error);
        kSum = kSum + k;
    }
    cerr << "KSUM: " << kSum << endl;
    return kSum;
}

ControlArm::FuzzyRule::FuzzyRule()
: min(Point3f())
, centre(Point3f())
, max(Point3f())
, weighting(Point3f())
{
    
}

ControlArm::FuzzyRule::FuzzyRule(Point3f a, Point3f b, Point3f c, Point3f w)
: min(a)
, centre(b)
, max(c)
, weighting(w)
{
    
}

ControlArm::FuzzyRule::FuzzyRule(FuzzyRule const& rule)
: min(rule.min)
, centre(rule.centre)
, max(rule.max)
, weighting(rule.weighting)
{
    
}

void ControlArm::FuzzyRule::SetValues(Point3f a, Point3f b, Point3f c, Point3f w)
{
    min = a;
    centre = b;
    max = c;
    weighting = w;
}

Point3f ControlArm::FuzzyRule::GetWeighting(Point3f error)
{
    Point3f k;
    
    if(error.x < max.x && error.x >= centre.x)
    {
        k.x = weighting.x * (max.x - error.x) / (max.x - centre.x);
    }
    else if(error.x > min.x && error.x < centre.x)
    {
        k.x = weighting.x * (error.x - min.x) / (centre.x - min.x);
    }
    
    if(error.y < max.y && error.y >= centre.y)
    {
        k.y = weighting.y * (max.y - error.y) / (max.y - centre.y);
    }
    else if(error.y > min.y && error.y < centre.y)
    {
        k.y = weighting.y * (error.y - min.y) / (centre.y - min.y);
    }
    
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

