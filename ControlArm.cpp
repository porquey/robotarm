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
    Kp=500;
    Ki=25;
    Kd=0;
}

int ControlArm::PIDControl::update(double value, double dest)
{
    double error = value - dest;
    
    if (abs(error) > 100000){
        return 0;
    }
    
    if (abs(error-lastError) > 1 && started){
        error = lastError;
        started = true;
    }
    double derivative = error - lastError;
    
    lastError = error;
    integral+= error;
    
    if (integral < -5 && error > 0){
        integral = 0;
    }
    
    if (integral > 5 && error < 0){
        integral = 0;
    }
    
    double out = (error * Kp + integral * Ki + derivative + Kd + 0.5);
    
    if (out > MAX_ERROR){
        return (int)MAX_ERROR;
    }
    else if (out < -MAX_ERROR){
        return -(int)MAX_ERROR;
    }
    else{
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
    Point3f baseVector = jointPositions[0] - jointPositions[2];
    Point3f vector1 = jointPositions[1] - jointPositions[2];
    Point3f vector2 = jointPositions[3] - jointPositions[2];
    
    double x = baseVector.x;
    double z = baseVector.z;
    
    currentAngles[0] = abs(atan2(z, x));
    
    currentAngles[1] = CalculateAngle(CalculateVector(jointPositions[1], jointPositions[0]), CalculateVector(jointPositions[2], jointPositions[1]));
    
    currentAngles[2] = CalculateAngle(CalculateVector(jointPositions[2], jointPositions[1]), CalculateVector(jointPositions[3], jointPositions[2]));
    
    //If joint 2 is further away from camera 1 than joint 1
    if (jointPositions[2].z > jointPositions[1].z){
        currentAngles[1] = -currentAngles[1];
        currentAngles[0] = -currentAngles[0];
    }
    
    
    Point3f cross = CalculateCrossProduct(vector1, vector2);
    if(cross.x < 0){
        currentAngles[2] = -currentAngles[2];
    }
    
    //Calculate joint angle offset if it crosses the axis
    if (lastJointAngles[0] != 0 && (currentAngles[0] * lastJointAngles[0]) < 0)
        jointOffsets[0] = abs(lastJointAngles[0]) < 0.4 ? abs(lastJointAngles[0]) : 0;
    if (lastJointAngles[1] != 0 && (currentAngles[1] * lastJointAngles[1]) < 0)
        jointOffsets[1] = abs(lastJointAngles[1]) < 0.4 ? abs(lastJointAngles[1]) : 0;
    if (lastJointAngles[2] != 0 && (currentAngles[2] * lastJointAngles[2]) < 0)
        jointOffsets[2] = abs(lastJointAngles[2]) < 0.4 ? abs(lastJointAngles[2]) : 0;
    
    lastJointAngles[0] = currentAngles[0];
    lastJointAngles[1] = currentAngles[1];
    lastJointAngles[2] = currentAngles[2];
    
    angles[0] = currentAngles[0];
    angles[1] = currentAngles[1];
    angles[2] = currentAngles[2];

//
//    //Reduce angle with the offset
//    if (currentAngles[0] > 0)
//        angles[0] = currentAngles[0] - jointOffsets[0];
//    else
//        angles[0] = currentAngles[0] + jointOffsets[0];
//    if (currentAngles[1] > 0)
//        angles[1] = currentAngles[1] - jointOffsets[1];
//    else
//        angles[1] = currentAngles[1] + jointOffsets[1];
//    if (currentAngles[2] > 0)
//        angles[2] = currentAngles[2] - jointOffsets[2];
//    else
//        angles[2] = currentAngles[2] + jointOffsets[2];
}

void ControlArm::SetTarget(Point3f target)
{
    targetPosition = target;
    //cerr << "TARGET: " << targetPosition.x << " " << targetPosition.y << " " << targetPosition.z << endl;
    
    if(CalculateLength(CalculateVector(targetPosition, jointPositions[1])) > link1 + link2)
    {
        Point3f pointDiff = targetPosition - jointPositions[1];
        double absDiff = CalculateLength(pointDiff);
        cerr << " link1: " << link1 << " link2: " << link2 << " absDiff: " << absDiff << endl;
        targetPosition.x = (int)((pointDiff.x / absDiff) * (link1 + link2));
        targetPosition.y = (int)((pointDiff.y / absDiff) * (link1 + link2));
        targetPosition.z = (int)((pointDiff.z / absDiff) * (link1 + link2));
        cerr << "Final distance: " << CalculateLength(targetPosition) << endl;
        targetPosition = targetPosition + jointPositions[1];
        
        //cerr << "OUT OF REACH. NEW TARGET: " << targetPosition.x << " " << targetPosition.y << " " << targetPosition.z << endl;
    }
    FindInverseKinematics();
    //fuzzyTarget = targetPosition;
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
        
        fuzzyTarget.x = (int)((pointDiff.x / absDiff) * (link1 + link2));
        fuzzyTarget.y = (int)((pointDiff.y / absDiff) * (link1 + link2));
        fuzzyTarget.z = (int)((pointDiff.z / absDiff) * (link1 + link2));
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
    if(startFuzzy)
    {
        return fuzzyTarget;
    }
    else
    {
        return targetPosition;
    }
}

bool ControlArm::UpdateArmPose(Point3f detected, double error0, double error1, double error2)
{
    
    if(startFuzzy)
    {
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
    if(CalculateLength(fuzzyTarget - targetPosition) > 10 && CalculateLength(targetLast - targetPosition) > 1)
    {
        fuzzyTarget = targetPosition;
        targetLast = targetPosition;
        return false;
    }
    if(CalculateLength(targetLast - targetPosition) > 1)
    {
        targetLast = targetPosition;
    }
    
    if(startFuzzy)
    {
        if(CalculateLength(jointPositions[3] - targetPosition) < 5)
        {
            cerr << "Reached distance of " << CalculateLength(jointPositions[3] - targetPosition) << " from target" << endl;
            cerr << "Target reached" << endl;
            //startFuzzy = false;
            return false;
        }
        else if(it > 10)
        {
            cerr << "Reached distance of " << CalculateLength(jointPositions[3] - targetPosition) << " from target" << endl;
            cerr << "Terminating after 10 iterations" << endl;
            startFuzzy = false;
            return false;
        }
        else if(abs(error0) <= 0.2 && abs(error1) <= 0.2 && abs(error2) <= 0.2)
        {
            SetFuzzyTarget(fuzzyTarget - CalculateCompensationStep(detected));
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
    else
    {
        return false;
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
    //cerr << "From Target: " << CalculateLength(CalculateVector(targetPosition, jointPositions[3])) << endl;
    //cerr << "From Fuzzy Target: " << CalculateLength(CalculateVector(fuzzyTarget, jointPositions[3])) << endl;
    return CalculateLength(CalculateVector(targetPosition, jointPositions[3]));
}

void ControlArm::FindInverseKinematics()
{
    double tempAngles[3];
    
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
    
    tempAngles[2] = atan2(-sqrt(1 - d * d), d);
    //jointAngles[1] = atan2(sqrt(s), sqrt(r)) - atan2(sin(jointAngles[2]), link1 + link2 * cos(jointAngles[2]));
    
    double k1 = link1 + link2 * cos(tempAngles[2]);
    double k2 = link2 * sin(tempAngles[2]);

    tempAngles[1] = atan2(s, r) - atan2(k2, k1);
    
    tempAngles[2] = -tempAngles[2];
    tempAngles[1] = HALF_PI - tempAngles[1];
    
    
    if (tempAngles[0] <= 0){
        /*jointAngles[0] += PI;
        
        if(jointAngles[0] > PI)
        {
            jointAngles[0] -= 2 * PI;
        }
        else if(jointAngles[0] < -PI)
        {
            jointAngles[0] += 2 * PI;
        }
        */
        tempAngles[1] = -tempAngles[1];
        tempAngles[2] = -tempAngles[2];
    }

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

void ControlArm::InitFuzzyController()
{
    it = 0;
    startFuzzy = true;
    fuzzySet.clear();
    fuzzySet.push_back(FuzzyRule(Point3f(-100000, -100000, -100000), Point3f(-5, -5, -5), Point3f(-2.5, -2.5, -2.5), Point3f(-1.5, -1.5, -1.5)));
    fuzzySet.push_back(FuzzyRule(Point3f(-10, -10, -10), Point3f(-5, -5, -5), Point3f(0, 0, 0), Point3f(-0.4, -0.4, -0.4)));
    fuzzySet.push_back(FuzzyRule(Point3f(-5, -5, -5), Point3f(0, 0, 0), Point3f(5, 5, 5), Point3f(0, 0, 0)));
    fuzzySet.push_back(FuzzyRule(Point3f(0, 0, 0), Point3f(5, 5, 5), Point3f(10, 10, 10), Point3f(0.4, 0.4, 0.4)));
    fuzzySet.push_back(FuzzyRule(Point3f(5, 5, 5), Point3f(10, 10, 10), Point3f(100000, 100000, 100000), Point3f(1.5, 1.5, 1.5)));
    SetFuzzyTarget(targetPosition);
}

void ControlArm::TerminateFuzzyController()
{
    startFuzzy = false;
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

