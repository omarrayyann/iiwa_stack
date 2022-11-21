#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>
#include <ros/ros.h>
#include <ros/service.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>


using namespace std;
using namespace Eigen;

//Utilities class
class Utils
{
public:
    static Matrix4d rotx(double theta)
    {
        double cost = cos(theta);
        double sint = sin(theta);

        Matrix4d rot;
        rot << 1, 0, 0, 0,
            0, cost, -sint, 0,
            0, sint, cost, 0,
            0, 0, 0, 1;

        return rot;
    }
    static Matrix4d roty(double theta)
    {
        double cost = cos(theta);
        double sint = sin(theta);

        Matrix4d rot;
        rot << cost, 0, sint, 0,
            0, 1, 0, 0,
            -sint, 0, cost, 0,
            0, 0, 0, 1;

        return rot;
    }
    static Matrix4d rotz(double theta)
    {
        double cost = cos(theta);
        double sint = sin(theta);

        Matrix4d rot;
        rot << cost, -sint, 0, 0,
            sint, cost, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        return rot;
    }
    static Matrix4d trn(Vector3d v)
    {

        Matrix4d trn;
        trn << 1, 0, 0, v(0),
            0, 1, 0, v(1),
            0, 0, 1, v(2),
            0, 0, 0, 1;

        return trn;
    }
    static Matrix4d trn(double x, double y, double z)
    {

        Matrix4d trn;
        trn << 1, 0, 0, x,
            0, 1, 0, y,
            0, 0, 1, z,
            0, 0, 0, 1;

        return trn;
    }
    static Matrix3d S(Vector3d v)
    {
        Matrix3d res = Matrix3d::Zero();

        res(0, 1) = -v(2);
        res(1, 0) = v(2);
        res(0, 2) = v(1);
        res(2, 0) = -v(1);
        res(1, 2) = -v(0);
        res(2, 1) = v(0);

        return res;
    }
    static Vector3d cross(Vector3d a, Vector3d b)
    {
        return Utils::S(a) * b;
    }
    static MatrixXd pinv(MatrixXd M, double eps)
    {
        int m = M.cols();
        MatrixXd MT = M.transpose();
        return (eps * MatrixXd::Identity(m, m) + MT * M).inverse() * MT;
    }
    static VectorXd vecPow(VectorXd v, int indStart, int indEnd, double h)
    {
        //int n = v.rows();
        VectorXd w = v;

        for (int i = indStart; i < indEnd; i++)
        {
            w[i] = (v[i] > 0 ? 1 : -1) * pow(abs(v[i]), h);
        }

        return w;
    }
    static double rand(double vMin = 0, double vMax = 1)
    {
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_real_distribution<double> dist(vMin, vMax);

        return dist(mt);
    }
    static VectorXd randVec(int n, double vMin = 0, double vMax = 0)
    {
        VectorXd v(n);

        for (int i = 0; i < n; i++)
        {
            v[i] = rand(vMin, vMax);
        }

        return v;
    }
    static Vector3d rpy(Matrix4d rot)
    {
        double cos_pitch = sqrt( rot(2,1) * rot(2,1) + rot(2,2) * rot(2,2) );
        double sin_pitch = -rot(2,0);
        Vector3d angles = Vector3d::Zero();

        if(cos_pitch > 0.001)
        {
            double roll = atan2(rot(2,1), rot(2,2));
            double pitch = atan2(-rot(2,0), cos_pitch);
            double yaw = atan2(rot(1,0), rot(0,0));
            angles<<roll, pitch, yaw;
            return angles;
        }
        else
        {
            if(sin_pitch>0)
            {
                double pitch = M_PI/2;
                double yaw  = 0;
                double roll = atan2(rot(0,1), rot(0,2));
                angles<<roll, pitch, yaw;
                return angles;
            }
            else
            {
                double pitch = -M_PI/2;
                double yaw  = 0;
                double roll = atan2(-rot(0,1), -rot(0,2));
                angles<<roll, pitch, yaw;
                return angles;
            }
        }
    }
    static double distanceAABB(GeometricPrimitives *objA, GeometricPrimitives *objB)
    {
        AxisAlignedBoundingBox AABB_A = (*objA).getAABB();
        AxisAlignedBoundingBox AABB_B = (*objB).getAABB();

        
        Vector3d delta = (*objA).htm.block<3,1>(0,3) - (*objB).htm.block<3,1>(0,3);

        double dx = max(abs(delta[0]) - (AABB_A.width + AABB_B.width) / 2.0, 0,0);
        double dy = max(abs(delta[1]) - (AABB_A.depth + AABB_B.depth) / 2.0, 0.0);
        double dz = max(abs(delta[2]) - (AABB_A.height + AABB_B.height) / 2.0, 0.0);

        return sqrt(dx * dx + dy * dy + dz * dz)
    }

};

// Implement geometric primitives and distance computation
struct AxisAlignedBoundingBox
{
    double width;
    double depth;
    double height;
};

struct DistanceStruct
{
    double distance;
    Vector3d witnessA;
    Vector3d witnessB;
};

class GeometricPrimitives
{
public:
    Matrix4d htm;
    virtual AxisAlignedBoundingBox getAABB() = 0;
    virtual Vector3d projection(Vector3d point) = 0;
    virtual GeometricPrimitives* copy() = 0;

    static DistanceStruct computeDist(GeometricPrimitives* objA, GeometricPrimitives* objB, Vector3d pointA0 = Vector3d::Random(), 
    double tol=0.001)
    {
        Vector3d pointA = pointA0;
        Vector3d pointB;
        Vector3d pointAOld;

        do
        {
            pointAOld = pointA;
            pointB = objB->projection(pointA);
            pointA = objA->projection(pointB);
        }while( (pointA-pointAOld).norm() >= tol );

        DistanceStruct ds;
        ds.witnessA = pointA;
        ds.witnessB = pointB;
        ds.distance = (pointA-pointB).norm();

        return ds;
    }
};

class Box : public GeometricPrimitives
{
public:
    double width;
    double depth;
    double height;

    Box(Matrix4d htm0, double widthVal, double depthVal, double heightVal)
    {
        htm = htm0;
        width = widthVal;
        depth = depthVal;
        height = heightVal;
    }

    AxisAlignedBoundingBox getAABB()
    {
        Vector3d p1 =  this->width * this->htm.block<3,1>(0, 0) + this->depth * this->htm.block<3,1>(0, 1) + this->height * this->htm.block<3,1>(0, 2);
        Vector3d p2 = -this->width * this->htm.block<3,1>(0, 0) + this->depth * this->htm.block<3,1>(0, 1) + this->height * this->htm.block<3,1>(0, 2);
        Vector3d p3 =  this->width * this->htm.block<3,1>(0, 0) - this->depth * this->htm.block<3,1>(0, 1) + this->height * this->htm.block<3,1>(0, 2);
        Vector3d p4 =  this->width * this->htm.block<3,1>(0, 0) + this->depth * this->htm.block<3,1>(0, 1) - this->height * this->htm.block<3,1>(0, 2);

        AxisAlignedBoundingBox aabb;
        aabb.width  = max(p1[0],max(p2[0],max(p3[0], p4[0])));
        aabb.depth  = max(p1[1],max(p2[1],max(p3[1], p4[1])));
        aabb.height = max(p1[2],max(p2[2],max(p3[2], p4[2])));

        return aabb;
    }

    Vector3d projection(Vector3d point)
    {
        Matrix3d Q = this->htm.block<3,3>(0,0);
        Vector3d p = this->htm.block<3,1>(0,3);
        Vector3d point_mod = Q.transpose() * (point - p);

        point_mod[0] = min(max(point_mod[0], -this->width/2), this->width/2);
        point_mod[1] = min(max(point_mod[1], -this->depth/2), this->depth/2);
        point_mod[2] = min(max(point_mod[2], -this->height/2), this->height/2);

        return Q * point_mod + p;
    }

    GeometricPrimitives* copy()
    {
        Box *b = new Box(this->htm, this->width, this->depth, this->height);
        return b;
    }
};

class Cylinder : public GeometricPrimitives
{
public:
    double radius;
    double height;

    Cylinder(Matrix4d htm0, double radiusVal, double heightVal)
    {
        htm = htm0;
        radius = radiusVal;
        height = heightVal;
    }

    AxisAlignedBoundingBox getAABB()
    {
        double dradius = 2*this->radius;

        Vector3d p1 =  dradius * this->htm.block<3,1>(0, 0) + dradius * this->htm.block<3,1>(0, 1) + this->height * this->htm.block<3,1>(0, 2);
        Vector3d p2 = -dradius * this->htm.block<3,1>(0, 0) + dradius * this->htm.block<3,1>(0, 1) + this->height * this->htm.block<3,1>(0, 2);
        Vector3d p3 =  dradius * this->htm.block<3,1>(0, 0) - dradius * this->htm.block<3,1>(0, 1) + this->height * this->htm.block<3,1>(0, 2);
        Vector3d p4 =  dradius * this->htm.block<3,1>(0, 0) + dradius * this->htm.block<3,1>(0, 1) - this->height * this->htm.block<3,1>(0, 2);

        AxisAlignedBoundingBox aabb;
        aabb.width  = max(p1[0],max(p2[0],max(p3[0], p4[0])));
        aabb.depth  = max(p1[1],max(p2[1],max(p3[1], p4[1])));
        aabb.height = max(p1[2],max(p2[2],max(p3[2], p4[2])));

        return aabb;
    }

    Vector3d projection(Vector3d point)
    {
        Matrix3d Q = this->htm.block<3,3>(0,0);
        Vector3d p = this->htm.block<3,1>(0,3);
        Vector3d point_mod = Q.transpose() * (point - p);

        double dist = sqrt( pow(point_mod[0],2) + pow(point_mod[1], 2));

        if( dist > this->radius)
        {
            point_mod[0] = this->radius * point_mod[0]/dist;
            point_mod[1] = this->radius * point_mod[1]/dist; 
        }

        point_mod[2] = min(max(point_mod[2], -this->height/2), this->height/2);

        return Q * point_mod + p;
    }

    GeometricPrimitives* copy()
    {
        Cylinder *c = new Cylinder(this->htm, this->radius, this->height);
        return c;
    }
};

class Sphere : public GeometricPrimitives
{
public:
    double radius;

    Sphere(Matrix4d htm0, double radiusVal)
    {
        htm = htm0;
        radius = radiusVal;
    }

    AxisAlignedBoundingBox getAABB()
    {

        AxisAlignedBoundingBox aabb;
        aabb.width  = 2 * this-> radius;
        aabb.depth  = 2 * this-> radius;
        aabb.height = 2 * this-> radius;

        return aabb;
    }

    Vector3d projection(Vector3d point)
    {

        Vector3d p = this->htm.block<3,1>(0,3);
        Vector3d point_mod = point - p;

        double dist = point_mod.norm();

        if( dist > this->radius)
        {
            point_mod = this->radius * point_mod/dist;
        }

        return point_mod + p;
    }

    GeometricPrimitives* copy()
    {
        Sphere *e = new Sphere(this->htm, this->radius);
        return e;
    }
};

// Implement robot routines

class Link
{
private:
    double _cost;
    double _sint;
    double _cosa;
    double _sina;

public:
    double dHtheta;
    double dHd;
    double dHalpha;
    double dHa;
    int jointType;
    int linkNumber;
    vector<GeometricPrimitives *> colObjs;
    vector<Matrix4d> htmCols;
    string name;

    Matrix4d dhMatrix(double q)
    {
        Matrix4d dhMat = Matrix4d::Zero();

        double cost = this->jointType == 0 ? cos(q) : this->_cost;
        double sint = this->jointType == 0 ? sin(q) : this->_sint;
        double cosa = _cosa;
        double sina = _sina;
        double d = this->jointType == 1 ? q : this->dHd;
        double a = this->dHa;

        dhMat(0, 0) = cost;
        dhMat(0, 1) = -sint * cosa;
        dhMat(0, 2) = sint * sina;
        dhMat(0, 3) = a * cost;

        dhMat(1, 0) = sint;
        dhMat(1, 1) = cost * cosa;
        dhMat(1, 2) = -cost * sina;
        dhMat(1, 3) = a * sint;

        dhMat(2, 1) = sina;
        dhMat(2, 2) = cosa;
        dhMat(2, 3) = d;

        dhMat(3, 3) = 1;

        return dhMat;
    }

    Link(int number, double theta, double d, double alpha, double a, int type, string strName)
    {
        linkNumber = number;
        dHtheta = theta;
        dHd = d;
        dHalpha = alpha;
        dHa = a;
        jointType = type;
        name = strName;

        _cost = cos(theta);
        _sint = sin(theta);
        _cosa = cos(alpha);
        _sina = sin(alpha);
    }
};

struct FKResult
{
    vector<Matrix4d> htmDH;
    Matrix4d htmTool;
    vector<MatrixXd> jacDH;
    MatrixXd jacTool;
};

struct TaskResult
{
    VectorXd task;
    MatrixXd jacTask;
};

class DistanceLinkObj
{
    public:
        int linkNumber;
        int colObjLinkNumber;
        Vector3d witnessColObjLink;
        Vector3d witnessObj;
        double distance;
        VectorXd jacDist;

        DistanceLinkObj();
};
class DistanceRobotObj
{
    private:
        vector<DistanceLinkObj> _structDistances;
        int _noJoints;

    public:
        int noElements;
        GeometricPrimitives *obj;
        Manipulator robot;

        DistanceRobotObj();

        DistanceLinkObj operator [](int i) const
        {
            return _structDistances[i];
        }

        DistanceLinkObj getClosest()
        {
            DistanceLinkObj ds = this->_structDistances[0];
            double minDist = ds.distance;

            for(int i=0; i < this->noElements; i++)
            {
                if(this->_structDistances[i].distance < minDist)
                {
                    minDist = this->_structDistances[i].distance;
                    ds = this->_structDistances[i];
                }
            }

            return ds;
        }

        DistanceLinkObj* getItem(int indLink, int indColObjLink)
        {
            for(int i=0; i < this->noElements; i++)
            {
                if(this->_structDistances[i].linkNumber == indLink && this->_structDistances[i].colObjLinkNumber == indColObjLink)
                {
                    return &this->_structDistances[i];
                }
            }
            return NULL;
        }

        MatrixXd getAllJacobians()
        {
            MatrixXd jac(this->noElements, this->_noJoints);

            for(int i=0; i < this->noElements; i++)
            {
                for(int j=0; j < this->_noJoints; j++)
                {
                    jac(i,j) = this->_structDistances[i].jacDist[j];
                }
            }

            return jac;

        }

        VectorXd getDistances()
        {
            VectorXd dist(this->noElements);

            for(int i=0; i < this->noElements; i++)
            {
                dist[i] = this->_structDistances[i].distance;
            }

            return dist;            
        }
};

class Manipulator
{
public:
    int noJoints;
    vector<Link> links;
    VectorXd q;
    VectorXd qBase;
    Matrix4d htmWorldToBase;
    Matrix4d htmBaseToDH0;
    Matrix4d htmDHnToTool;

    Manipulator(int no)
    {
        noJoints = no;
        q = VectorXd::Zero(no);
        qBase = VectorXd::Zero(no);
        htmWorldToBase = Matrix4d::Identity();
        htmBaseToDH0 = Matrix4d::Identity();
        htmDHnToTool = Matrix4d::Identity();
    }

    static Manipulator createKukaKR5()
    {
        // Create empty manipulator
        Manipulator manip(6);

        // Create links
        double d_bs = 0.340;
        double d_se = 0.400;
        double d_ew = 0.400;
        double d_wf = 0.126;

        vector<Link> linksKuka;

        Link link1(1, 0, 0.335, -M_PI / 2, 0.075, 0,"j1");
        Link link2(2, 0, 0, 0, 0.365, 0,"j2");
        Link link3(3, 0, 0, M_PI / 2, 0.090, 0,"j3");
        Link link4(4, 0, -0.405, -M_PI / 2, 0, 0,"j4");
        Link link5(5, 0, 0, -M_PI / 2, 0, 0,"j5");
        Link link6(6, 0, 0.080, 0, 0, 0,"j6");

        linksKuka.push_back(link1);
        linksKuka.push_back(link2);
        linksKuka.push_back(link3);
        linksKuka.push_back(link4);
        linksKuka.push_back(link5);
        linksKuka.push_back(link6);

        manip.links = linksKuka;

        return manip;
    }

    static Manipulator createKukaIIWA()
    {
        // Create empty manipulator
        Manipulator manip(7);

        // Create links
        double d_bs = 0.340;
        double d_se = 0.400;
        double d_ew = 0.400;
        double d_wf = 0.126;

        vector<Link> linksKuka;

        Link link1(1, 0, d_bs, -M_PI / 2, 0, 0, "iiwa_joint_1");
        Link link2(2, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_2");
        Link link3(3, 0, d_se, M_PI / 2, 0, 0, "iiwa_joint_3");
        Link link4(4, 0, 0, -M_PI / 2, 0, 0, "iiwa_joint_4");
        Link link5(5, 0, d_ew, -M_PI / 2, 0, 0, "iiwa_joint_5");
        Link link6(6, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_6");
        Link link7(7, 0, d_wf, 0, 0, 0, "iiwa_joint_7");

        linksKuka.push_back(link1);
        linksKuka.push_back(link2);
        linksKuka.push_back(link3);
        linksKuka.push_back(link4);
        linksKuka.push_back(link5);
        linksKuka.push_back(link6);
        linksKuka.push_back(link7);

        manip.links = linksKuka;

        //Create collision objects
        FKResult fkres0 = manip.fk();

   
        //For the first link
        Cylinder *iiwaC0_0 = new Cylinder(Utils::trn(0,-0.14,0) * Utils::rotx(M_PI/2), 0.1, 0.12);
        manip.links[0].colObjs.push_back(iiwaC0_0);
        manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_0).htm);

        Sphere *iiwaC0_1 = new Sphere(Utils::trn(0,-0.03, 0) * Utils::rotx(M_PI/2), 0.12);
        manip.links[0].colObjs.push_back(iiwaC0_1);
        manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_1).htm);

        //For the second link
        Sphere *iiwaC1_0 = new Sphere(Utils::trn(0,-0.035, 0.01), 0.1);
        manip.links[1].colObjs.push_back(iiwaC1_0);
        manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_0).htm);

        Cylinder *iiwaC1_1 = new Cylinder(Utils::trn(0,0, 0.14), 0.075, 0.125);
        manip.links[1].colObjs.push_back(iiwaC1_1);
        manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_1).htm);

        //For the third link
        Cylinder *iiwaC2_0 = new Cylinder(Utils::trn(0, 0.17, 0) * Utils::rotx(M_PI/2), 0.08, 0.1);
        manip.links[2].colObjs.push_back(iiwaC2_0);
        manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_0).htm);

        Sphere *iiwaC2_1 = new Sphere(Utils::trn(0,0.035, -0.01), 0.11);
        manip.links[2].colObjs.push_back(iiwaC2_1);
        manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_1).htm);

        //For the fourth link
        Sphere *iiwaC3_0 = new Sphere(Utils::trn(0,0.035, 0.01), 0.08);
        manip.links[3].colObjs.push_back(iiwaC3_0);
        manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_0).htm);

        Cylinder *iiwaC3_1 = new Cylinder(Utils::trn(0,0, 0.12), 0.075, 0.125);
        manip.links[3].colObjs.push_back(iiwaC3_1);
        manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_1).htm);

        //For the fifth link
        Cylinder *iiwaC4_0 = new Cylinder(Utils::trn(0, -0.165 + 0.005, 0) * Utils::rotx(M_PI/2), 0.075, 0.11);
        manip.links[4].colObjs.push_back(iiwaC4_0);
        manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_0).htm);

        Box *iiwaC4_1 = new Box(Utils::trn(0, -0.03, 0.08), 0.12, 0.17, 0.04);
        manip.links[4].colObjs.push_back(iiwaC4_1);
        manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_1).htm);

        //For the sixth link
        Cylinder *iiwaC5_0 = new Cylinder(Utils::trn(0, 0, 0.023), 0.075, 0.21);
        manip.links[5].colObjs.push_back(iiwaC5_0);
        manip.links[5].htmCols.push_back(fkres0.htmDH[5].inverse() * (*iiwaC5_0).htm);    


        return manip;
    }

    void setConfig(VectorXd q, Matrix4d customHtmWorldToBase = Matrix4d::Zero())
    {
        // Set the configuration
        this->q = q;

        FKResult fkres = this->fk(q, customHtmWorldToBase);

        // Update all primitive collision objects to the new pose
        for(int i=0; i < this->noJoints; i++)
        {
            for(int j=0; j < this->links[i].colObjs.size(); j++)
            {
                (*this->links[i].colObjs[j]).htm = fkres.htmDH[i] * this->links[i].htmCols[j];
            }
        }
    }

    FKResult fk(VectorXd q = VectorXd(0), Matrix4d customHtmWorldToBase = Matrix4d::Zero())
    {

        VectorXd q_int = q.rows() == 0 ? this->q : q;
        Matrix4d htmNow = customHtmWorldToBase(3, 3) == 0 ? this->htmWorldToBase : customHtmWorldToBase;
        htmNow = htmNow * this->htmBaseToDH0;

        vector<Matrix4d> listOfHtms;
        double theta, d, alpha, a;

        for (int i = 0; i < this->noJoints; i++)
        {
            htmNow = htmNow * links[i].dhMatrix(q_int[i]);
            listOfHtms.push_back(htmNow);
        }

        FKResult fkres;
        fkres.htmDH = listOfHtms;
        fkres.htmTool = htmNow * this->htmDHnToTool;

        return fkres;
    }

    FKResult jacGeo(VectorXd q = VectorXd(0), Matrix4d customHtmWorldToBase = Matrix4d::Zero())
    {
        // Compute the forward kinematics
        FKResult fkres = this->fk(q, customHtmWorldToBase);

        // Now compute the differential kinematics
        vector<MatrixXd> listOfJacs;

        int n = this->noJoints;

        for (int i = 0; i < n; i++)
        {
            MatrixXd newJac = MatrixXd::Zero(6, this->noJoints);
            listOfJacs.push_back(newJac);
        }

        Vector3d p_i, p_j_ant, z_j_ant, v, w;
        Matrix4d htmWorldToDH0 = customHtmWorldToBase(3, 3) == 0 ? this->htmWorldToBase : customHtmWorldToBase;
        htmWorldToDH0 = htmWorldToDH0 * this->htmBaseToDH0;

        for (int i = 0; i < n; i++)
        {
            p_i = fkres.htmDH[i].block<3, 1>(0, 3);
            for (int j = 0; j <= i; j++)
            {

                p_j_ant = (j == 0) ? htmWorldToDH0.block<3, 1>(0, 3) : fkres.htmDH[j - 1].block<3, 1>(0, 3);
                z_j_ant = (j == 0) ? htmWorldToDH0.block<3, 1>(0, 2) : fkres.htmDH[j - 1].block<3, 1>(0, 2);

                v = (this->links[j].jointType == 0) ? Utils::cross(z_j_ant, p_i - p_j_ant) : z_j_ant;
                w = (this->links[j].jointType == 0) ? z_j_ant : Vector3d::Zero();

                listOfJacs[i](0, j) = v(0);
                listOfJacs[i](1, j) = v(1);
                listOfJacs[i](2, j) = v(2);
                listOfJacs[i](3, j) = w(0);
                listOfJacs[i](4, j) = w(1);
                listOfJacs[i](5, j) = w(2);
            }
        }

        fkres.jacDH = listOfJacs;

        Vector3d p_ef = fkres.htmTool.block<3, 1>(0, 3);
        Vector3d p_n = fkres.htmDH[n - 1].block<3, 1>(0, 3);
        MatrixXd Jv = fkres.jacDH[n - 1].block(0, 0, 3, n);
        MatrixXd Jw = fkres.jacDH[n - 1].block(3, 0, 3, n);
        MatrixXd JvTool = Jv - Utils::S(p_ef - p_n) * Jw;
        MatrixXd jacGeoTool(6, n);
        jacGeoTool << JvTool, Jw;

        fkres.jacTool = jacGeoTool;

        return fkres;
    }

    TaskResult taskFunction(Matrix4d taskHtm, VectorXd q = VectorXd(0))
    {
        int n = this->noJoints;
        FKResult fkres = this->jacGeo(q);

        Vector3d p_ef = fkres.htmTool.block<3, 1>(0, 3);
        Vector3d x_ef = fkres.htmTool.block<3, 1>(0, 0);
        Vector3d y_ef = fkres.htmTool.block<3, 1>(0, 1);
        Vector3d z_ef = fkres.htmTool.block<3, 1>(0, 2);

        Vector3d p_des = taskHtm.block<3, 1>(0, 3);
        Vector3d x_des = taskHtm.block<3, 1>(0, 0);
        Vector3d y_des = taskHtm.block<3, 1>(0, 1);
        Vector3d z_des = taskHtm.block<3, 1>(0, 2);

        Vector3d r_p = p_ef - p_des;
        Vector3d r_o;
        double e_x = 1 - x_des.transpose() * x_ef;
        double e_y = 1 - y_des.transpose() * y_ef;
        double e_z = 1 - z_des.transpose() * z_ef;
        r_o << e_x, e_y, e_z;

        VectorXd r(6, 1);
        r << r_p, r_o;

        MatrixXd jacr_p = fkres.jacTool.block(0, 0, 3, n);
        MatrixXd Jw = fkres.jacDH[n - 1].block(3, 0, 3, n);
        MatrixXd jacr_o_x = x_des.transpose() * Utils::S(x_ef) * Jw;
        MatrixXd jacr_o_y = y_des.transpose() * Utils::S(y_ef) * Jw;
        MatrixXd jacr_o_z = z_des.transpose() * Utils::S(z_ef) * Jw;

        MatrixXd jacr(6, n);
        jacr << jacr_p, jacr_o_x, jacr_o_y, jacr_o_z;

        TaskResult taskResult;

        taskResult.task = r;
        taskResult.jacTask = jacr;

        return taskResult;
    }

    VectorXd ik(Matrix4d desHtm, VectorXd q0 = VectorXd(0), double pTol = 0.001, double aTol = 2, int noIterMax = 2000)
    {
        int n = this->noJoints;

        VectorXd q = q0.rows() == 0 ? Utils::randVec(n, -M_PI, M_PI) : q0;

        bool continueLoop = true;
        TaskResult tr;
        double dt = 0.05;
        double error_pos, error_ori;

        int iter = 0;
        int localIter = 0;

        while (continueLoop)
        {
            tr = taskFunction(desHtm, q);
            MatrixXd H = Utils::pinv(MatrixXd::Ones(3, 6), 0.005);
            q = q - dt * (Utils::pinv(tr.jacTask, 0.005) * Utils::vecPow(tr.task, 0, 6, 0.5));

            error_pos = tr.task.block(0, 0, 3, 1).norm();
            error_ori = 0;
            for (int i = 0; i < 3; i++)
            {
                error_ori = max(error_ori, (180 / M_PI) * acos(1 - tr.task[i + 3]));
            }

            continueLoop = !(error_pos <= pTol && error_ori <= aTol) && (iter <= noIterMax);
            iter++;

            if (!(error_pos <= pTol && error_ori <= aTol) && localIter >= 500)
            {
                q = Utils::randVec(n, -M_PI, M_PI);
                localIter = 0;
            }
        }

        cout << iter << std::endl;
        return q;
    }

    DistanceRobotObj computeDist(GeometricPrimitives* obj, VectorXd q = VectorXd(0), Matrix4d customHtmWorldToBase = Matrix4d::Zero(),
    DistanceRobotObj *oldDistStruct = NULL, double tol=0.0005, double maxDist = INFINITY)
    {
        DistanceRobotObj ds;
        FKResult fkres = this->jacGeo(q, customHtmWorldToBase);

        //Prepare the list of objects with the appropriate pose
        vector<vector<GeometricPrimitives *>> listObjsCopy;

        for(int i=0; i < this->noJoints; i++)
        {
            for(int j=0; j < this->links[i].colObjs.size(); j++)
            {
                GeometricPrimitives *colObjCopied = (*this->links[i].colObjs[j]).copy();
                (*colObjCopied).htm = fkres.htmDH[i] * this->links[i].htmCols[j];
                listObjsCopy[i].push_back(colObjCopied);
            }
        }

        //Compute all the distances
    }
};

//Main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "iiwa_control_vinicius");
    ros::NodeHandle n;
    ros::Publisher arm_pub = n.advertise<trajectory_msgs::JointTrajectory>("/iiwa/PositionJointInterface_trajectory_controller/command",1000);
    ros::Rate loop_rate(100);
    

    ROS_INFO("Starting controller!");

    //Generate trajectory
    Manipulator iiwa = Manipulator::createKukaIIWA();

    double dt = 0.01;
    FKResult fkres = iiwa.jacGeo();
    Matrix4d htmTg = Utils::trn(0.5, 0, -0.5) * fkres.htmTool;
    TaskResult taskres;
    VectorXd qdot;
    trajectory_msgs::JointTrajectory traj;

   
    traj.joint_names.resize(7);
    //traj.points.resize(1);

    traj.joint_names[0] = iiwa.links[0].name;
    traj.joint_names[1] = iiwa.links[1].name;
    traj.joint_names[2] = iiwa.links[2].name;
    traj.joint_names[3] = iiwa.links[3].name;
    traj.joint_names[4] = iiwa.links[4].name;
    traj.joint_names[5] = iiwa.links[5].name;
    traj.joint_names[6] = iiwa.links[6].name;

    //traj.points.resize(1000);

    int i=0;
    traj.header.stamp = ros::Time::now();
    double kp=0.5;

    while(i < 1000)
    {
        taskres = iiwa.taskFunction(htmTg);
        qdot = -kp * Utils::pinv(taskres.jacTask, 0.001)*(Utils::vecPow(taskres.task, 0, 6, 0.5));
        //qdot = -Utils::pinv(taskres.jacTask, 0.005)*(taskres.task);
        iiwa.setConfig(iiwa.q + dt*qdot);

        //Publish
        //traj.header.stamp = ros::Time::now();

        trajectory_msgs::JointTrajectoryPoint joint;

        for(int j=0; j<7; j++)
        {
            joint.positions.push_back(iiwa.q[j]);
        }


        traj.points.push_back(joint);
        traj.points[i].time_from_start = ros::Duration(i*dt);

        //arm_pub.publish(traj);
        //ros::spinOnce();

        //ROS_INFO_STREAM("Step now: "<<i);
        //ROS_INFO_STREAM("Time now: "<<ros::Time::now());
        ROS_INFO_STREAM("Time stamp: "<<traj.points[i].time_from_start);
        ROS_INFO_STREAM("Task: "<<taskres.task.transpose());

        //loop_rate.sleep();
        i++;
        
    }

    //Come back to base configuration
    trajectory_msgs::JointTrajectoryPoint joint;

    for(int j=0; j<7; j++)
    {
        joint.positions.push_back(0);
    }
    traj.points.push_back(joint);
    traj.points[i].time_from_start = ros::Duration(i*dt+5);   

    //Publish
    arm_pub.publish(traj);
    //ros::spinOnce();
    //end trajectory generation


}