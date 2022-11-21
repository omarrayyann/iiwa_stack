#pragma once

#include "utils.h"
#include "distance.h"
#include "robot.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>
#include <string.h>


using namespace std;
using namespace Eigen;

class Manipulator;

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
    vector<string> nameObjs;
    string name;

    Matrix4d dhMatrix(double q);
    Link(int number, double theta, double d, double alpha, double a, int type, string strName);
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
        Manipulator *robot;

        DistanceRobotObj(int noJoints);
        DistanceRobotObj();
        DistanceLinkObj operator [](int i) const;
        DistanceLinkObj getClosest();        
        DistanceLinkObj* getItem(int indLink, int indColObjLink);
        MatrixXd getAllJacobians();
        VectorXd getDistances();
        void addDistanceLinkObj(DistanceLinkObj dlo);
};

class Manipulator
{
public:
    int noJoints;
    vector<Link> links;
    VectorXd q;
    VectorXd qBase;
    VectorXd qMin;
    VectorXd qMax;
    VectorXd qDotMin;
    VectorXd qDotMax;
    Matrix4d htmWorldToBase;
    Matrix4d htmBaseToDH0;
    Matrix4d htmDHnToTool;

    Manipulator(int no);
    static Manipulator createKukaKR5();
    static Manipulator createKukaIIWA();
    void setConfig(VectorXd q, Matrix4d customHtmWorldToBase = Matrix4d::Zero());
    FKResult fk(VectorXd q = VectorXd(0), Matrix4d customHtmWorldToBase = Matrix4d::Zero());
    FKResult jacGeo(VectorXd q = VectorXd(0), Matrix4d customHtmWorldToBase = Matrix4d::Zero());
    TaskResult taskFunction(Matrix4d taskHtm, VectorXd q = VectorXd(0));
    VectorXd ik(Matrix4d desHtm, VectorXd q0 = VectorXd(0), double pTol = 0.001, double aTol = 2, int noIterMax = 2000);
    DistanceRobotObj computeDist(GeometricPrimitives* obj, VectorXd q = VectorXd(0), Matrix4d customHtmWorldToBase = Matrix4d::Zero(),
    DistanceRobotObj *oldDistStruct = NULL, double tol=0.0005, double maxDist = 10000);
};
