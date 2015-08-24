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

void ControlArm::SetArmPose(vector<Point3D> joints)
{
    if(joints.size() == 5)
    {
        for(int i = 0; i < 5; i++)
        {
            jointPositions[i] = joints[i];
        }
    }
}

void ControlArm::CalculateLinkLengths()
{
    link0 = CalculateLength(CalculateVector(jointPositions[1], jointPositions[0]));
    link1 = CalculateLength(CalculateVector(jointPositions[2], jointPositions[1]));
    link2 = CalculateLength(CalculateVector(jointPositions[3], jointPositions[2]));
    
}

void ControlArm::GetArmPose(double angles[3])
{
    angles = jointAngles;
}

void ControlArm::SetTarget(Point3D target)
{
    targetPosition = target;
    if(CalculateLength(CalculateVector(targetPosition, jointPositions[1])) > link1 + link2)
    {
        Point3D pointDiff = targetPosition - jointPositions[1];
        double absDiff = CalculateLength(PointToVec(pointDiff));
        
        targetPosition.x = (pointDiff.x / absDiff) * (link1 + link2);
        targetPosition.y = (pointDiff.y / absDiff) * (link1 + link2);
        targetPosition.z = (pointDiff.z / absDiff) * (link1 + link2);
        targetPosition = targetPosition + jointPositions[1];
        
        cout << "OUT OF REACH. NEW TARGET: " << targetPosition.x << " " << targetPosition.y << " " << targetPosition.z << endl;
    }
    FindInverseKinematics();
    cout << "LINK0: " << link0 << " LINK1: " << link1 << " LINK2: " << endl;
    cout << "ANGLE0: " << jointAngles[0] << " ANGLE1: " << jointAngles[1] << " ANGLE2: " << jointAngles[2] << endl;
}

void ControlArm::UpdateArmPose(Point3D detected)
{
    SetTarget(CalculateCompensationStep(detected) + targetPosition);
    
}

void ControlArm::FindInverseKinematics()
{
    double x, z;
    double r, s, d;
    
    x = targetPosition.x - jointPositions[0].x;
    z = targetPosition.z - jointPositions[0].z;
    cout << "X: " << x << " Z: " << z << endl;
    
    jointAngles[0] = atan2(z, x);
    
    r = x * x + z * z;
    s = (targetPosition.y - jointPositions[1].y) * (targetPosition.y - jointPositions[1].y);
    
    d = (r + s - link2 * link2 - link1 * link1) / (2 * link1 * link2);
    
    jointAngles[2] = atan2(sqrt(1 - d * d), d);
    jointAngles[1] = atan2(sqrt(s), sqrt(r)) - atan2(sin(jointAngles[2]), link1 + link2 * cos(jointAngles[2]));
    
}

double ControlArm::CalculateAngle(Vector3D a, Vector3D b)
{
    double mag1 = CalculateLength(a);
    double mag2 = CalculateLength(b);
    
    if(mag1 == 0 || mag2 == 0)
    {
        cout << "Vector magnitude = 0" << endl;
        return 0;
    }
    double temp = (a.x * b.x + a.y * b.y + a.z * b.z)/(mag1 * mag2);
    cout << "M1 " << mag1 << " M2 " << mag2 << endl;
    cout << "Val: " << temp << endl;
    
    //return 3.14159265 - acos(temp) - 0.58;
    return 3.14159265 + acos(temp) - 0.58;
}

Vector3D ControlArm::CalculateVector(Point3D a, Point3D b)
{
    Vector3D c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    c.z = a.z - b.z;
    return c;
}

double ControlArm::CalculateLength(Vector3D a)
{
     return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

Vector3D ControlArm::PointToVec(const Point3D a)
{
    Vector3D b;
    b.x = a.x;
    b.y = a.y;
    b.z = a.z;
    
    return b;
}

void ControlArm::InitFuzzyController()
{
    fuzzySet.clear();
    fuzzySet.push_back(FuzzyRule(Point3D(-10000, -10000, -10000), Point3D(-5, -5.5, -5), Point3D(-3, -1.5, -2), Point3D(-0.6, -0.6, -0.6)));
    fuzzySet.push_back(FuzzyRule(Point3D(-5, -5.5, -5), Point3D(-3, -1.5, -2), Point3D(0, 0, 0), Point3D(-0.4, -0.4, -0.4)));
    fuzzySet.push_back(FuzzyRule(Point3D(-3, -1.5, -2), Point3D(0, 0, 0), Point3D(3, 1.5, 2), Point3D(0, 0, 0)));
    fuzzySet.push_back(FuzzyRule(Point3D(0, 0, 0), Point3D(3, 1.5, 2), Point3D(5, 5.5, 5), Point3D(0.4, 0.4, 0.4)));
    fuzzySet.push_back(FuzzyRule(Point3D(3, 1.5, 2), Point3D(5, 5.5, 5), Point3D(10000, 10000, 10000), Point3D(0.6, 0.6, 0.6)));
}

Point3D ControlArm::CalculateCompensationStep(Point3D detected)
{
    Point3D error = detected - targetPosition;
    Point3D kSum;
    for(int i = 0; i < 5; i++)
    {
        Point3D k = fuzzySet[i].GetWeighting(error);
        kSum = kSum + k;
    }
    return kSum;
}

ControlArm::FuzzyRule::FuzzyRule()
: min(Point3D())
, centre(Point3D())
, max(Point3D())
, weighting(Point3D())
{
    
}

ControlArm::FuzzyRule::FuzzyRule(Point3D a, Point3D b, Point3D c, Point3D w)
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

void ControlArm::FuzzyRule::SetValues(Point3D a, Point3D b, Point3D c, Point3D w)
{
    min = a;
    centre = b;
    max = c;
    weighting = w;
}

Point3D ControlArm::FuzzyRule::GetWeighting(Point3D error)
{
    Point3D k;
    
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
