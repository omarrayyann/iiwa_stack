#pragma once

#include "distance.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>


using namespace std;
using namespace Eigen;

struct VectorFieldResult
{
    VectorXd vector;
    double distance;
    int index;
};

class Utils
{
public:
    static Matrix4d rotx(double theta);

    static Matrix4d roty(double theta);

    static Matrix4d rotz(double theta);

    static Matrix4d trn(Vector3d v);

    static Matrix4d trn(double x, double y, double z);

    static Matrix3d S(Vector3d v);

    static Vector3d cross(Vector3d a, Vector3d b);

    static MatrixXd pinv(MatrixXd M, double eps);

    static VectorXd vecPow(VectorXd v, int indStart, int indEnd, double h);

    static double rand(double vMin = 0, double vMax = 1);

    static VectorXd randVec(int n, double vMin = 0, double vMax = 0);

    static Vector3d rpy(Matrix4d rot);

    static double distanceAABB(GeometricPrimitives* objA, GeometricPrimitives* objB);

    static vector<VectorXd> upsample(vector<VectorXd> points, double dist);

    static VectorFieldResult vectorField(VectorXd q, vector<VectorXd> points, double alpha = 3, bool openCurve = true, double percentLengthStop=0.8);

    static VectorXd solveQP(MatrixXd H, VectorXd f, MatrixXd A = MatrixXd(0,0), VectorXd b = VectorXd(0), MatrixXd Aeq = MatrixXd(0,0), VectorXd beq = VectorXd(0));

    static MatrixXd matrixVertStack(MatrixXd A1, MatrixXd A2);

    static VectorXd vectorVertStack(VectorXd v1, VectorXd v2);
};

