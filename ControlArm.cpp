#include "ControlArm.h"

ControlArm::ControlArm()
{
}

ControlArm::ControlArm(double l0, double l1, double l2) :
link0(l0),
link1(l1),
link2(l2)
{
}

ControlArm::PIDControl::PIDControl()
{
    integral = 0;
    lastError = 0;
    Kp=400;
    Ki=40;
    Kd=0;
}

int ControlArm::PIDControl::update(double value, double dest)
{
    double error = value - dest;
    double derivative = error - lastError;
    
    lastError = error;
    
    double out = (error * Kp + integral * Ki + derivative + Kd + 0.5);
    
    cerr << error << endl;
        
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
    integral = 0;
    lastError = 0;
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

void ControlArm::CalculateLinkLengths()
{
    link0 = CalculateLength(CalculateVector(jointPositions[1], jointPositions[0]));
    link1 = CalculateLength(CalculateVector(jointPositions[2], jointPositions[1]));
    link2 = CalculateLength(CalculateVector(jointPositions[3], jointPositions[2]));
    //cerr << "Link 0: " << link0 << " Link 1: " << link1 << " Link 2: " << link2 << endl;
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
    
    currentAngles[0] = atan2(z, x);
    
    currentAngles[1] = CalculateAngle(CalculateVector(jointPositions[1], jointPositions[0]), CalculateVector(jointPositions[2], jointPositions[1]));
    
    currentAngles[2] = 3.14159 - CalculateAngle(CalculateVector(jointPositions[2], jointPositions[1]), CalculateVector(jointPositions[2], jointPositions[3]));
    
    //If joint 2 is further away from camera 1 than joint 1
    if (jointPositions[2].z > jointPositions[1].z){
        currentAngles[1] = -currentAngles[1];
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
        
        targetPosition.x = (pointDiff.x / absDiff) * (link1 + link2);
        targetPosition.y = (pointDiff.y / absDiff) * (link1 + link2);
        targetPosition.z = (pointDiff.z / absDiff) * (link1 + link2);
        targetPosition = targetPosition + jointPositions[1];
        
        //cerr << "OUT OF REACH. NEW TARGET: " << targetPosition.x << " " << targetPosition.y << " " << targetPosition.z << endl;
    }
    FindInverseKinematics();
    cerr << "NEW ANGLE0: " << jointAngles[0] << " ANGLE1: " << jointAngles[1] << " ANGLE2: " << jointAngles[2] << endl;
}

void ControlArm::UpdateArmPose(Point3f detected)
{
    SetTarget(CalculateCompensationStep(detected) + targetPosition);
    
}

void ControlArm::SendJointActuators(int diff0, int diff1, int diff2){
    cout << diff0 << " " << diff1 << " " << diff2 << endl;
}

void ControlArm::FindInverseKinematics()
{
    double x, z;
    double r, s, d;
    
    x = targetPosition.x - jointPositions[0].x;
    z = targetPosition.z - jointPositions[0].z;
    //cerr << "X: " << x << " Z: " << z << endl;
    
    jointAngles[0] = atan2(z, x);
    
    r = x * x + z * z;
    s = (targetPosition.y - jointPositions[1].y) * (targetPosition.y - jointPositions[1].y);
    
    d = (r + s - link2 * link2 - link1 * link1) / (2 * link1 * link2);
    
    jointAngles[2] = atan2(sqrt(1 - d * d), d);
    jointAngles[1] = atan2(sqrt(s), sqrt(r)) - atan2(sin(jointAngles[2]), link1 + link2 * cos(jointAngles[2]));
    
}

void ControlArm::InitFuzzyController()
{
    fuzzySet.clear();
    fuzzySet.push_back(FuzzyRule(Point3f(-10000, -10000, -10000), Point3f(-5, -5.5, -5), Point3f(-3, -1.5, -2), Point3f(-0.6, -0.6, -0.6)));
    fuzzySet.push_back(FuzzyRule(Point3f(-5, -5.5, -5), Point3f(-3, -1.5, -2), Point3f(0, 0, 0), Point3f(-0.4, -0.4, -0.4)));
    fuzzySet.push_back(FuzzyRule(Point3f(-3, -1.5, -2), Point3f(0, 0, 0), Point3f(3, 1.5, 2), Point3f(0, 0, 0)));
    fuzzySet.push_back(FuzzyRule(Point3f(0, 0, 0), Point3f(3, 1.5, 2), Point3f(5, 5.5, 5), Point3f(0.4, 0.4, 0.4)));
    fuzzySet.push_back(FuzzyRule(Point3f(3, 1.5, 2), Point3f(5, 5.5, 5), Point3f(10000, 10000, 10000), Point3f(0.6, 0.6, 0.6)));
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

