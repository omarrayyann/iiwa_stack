
#include "utils.h"
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


Matrix4d Utils::rotx(double theta)
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
Matrix4d Utils::roty(double theta)
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
Matrix4d Utils::rotz(double theta)
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
Matrix4d Utils::trn(Vector3d v)
{

    Matrix4d trn;
    trn << 1, 0, 0, v(0),
        0, 1, 0, v(1),
        0, 0, 1, v(2),
        0, 0, 0, 1;

    return trn;
}
Matrix4d Utils::trn(double x, double y, double z)
{

    Matrix4d trn;
    trn << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;

    return trn;
}
Matrix3d Utils::S(Vector3d v)
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
Vector3d Utils::cross(Vector3d a, Vector3d b)
{
    return Utils::S(a) * b;
}
MatrixXd Utils::pinv(MatrixXd M, double eps)
{
    int m = M.cols();
    MatrixXd MT = M.transpose();
    return (eps * MatrixXd::Identity(m, m) + MT * M).inverse() * MT;
}
VectorXd Utils::vecPow(VectorXd v, int indStart, int indEnd, double h)
{
    //int n = v.rows();
    VectorXd w = v;

    for (int i = indStart; i < indEnd; i++)
    {
        w[i] = (v[i] > 0 ? 1 : -1) * pow(abs(v[i]), h);
    }

    return w;
}
double Utils::rand(double vMin , double vMax)
{
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(vMin, vMax);

    return dist(mt);
}
VectorXd Utils::randVec(int n, double vMin, double vMax)
{
    VectorXd v(n);

    for (int i = 0; i < n; i++)
    {
        v[i] = rand(vMin, vMax);
    }

    return v;
}
Vector3d Utils::rpy(Matrix4d rot)
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

double Utils::distanceAABB(GeometricPrimitives* objA, GeometricPrimitives* objB)
{
    AxisAlignedBoundingBox AABB_A = (*objA).getAABB();
    AxisAlignedBoundingBox AABB_B = (*objB).getAABB();

    
    Vector3d delta = (*objA).htm.block<3,1>(0,3) - (*objB).htm.block<3,1>(0,3);

    double dx = max(abs(delta[0]) - (AABB_A.width + AABB_B.width) / 2.0, 0.0);
    double dy = max(abs(delta[1]) - (AABB_A.depth + AABB_B.depth) / 2.0, 0.0);
    double dz = max(abs(delta[2]) - (AABB_A.height + AABB_B.height) / 2.0, 0.0);

    return sqrt(dx * dx + dy * dy + dz * dz);
}
