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
};

