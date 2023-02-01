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

#include <ros/ros.h>
#include <ros/service.h>

using namespace std;
using namespace Eigen;

// Auxiliary functions
#define PI 3.1415926
#define SQRTHALFPI 1.2533141
#define SQRT2 1.4142135
#define CONSTJA 2.7889

double fun_J(double u)
{
    return CONSTJA / ((CONSTJA - 1) * sqrt(PI * u * u) + sqrt(PI * u * u + CONSTJA * CONSTJA));
}

double fun_I0_hat(double u)
{
    return pow(1 + 0.25 * u * u, -0.25) * (1 + 0.24273 * u * u) / (1 + 0.43023 * u * u);
}

double fun_f(double nu, double rho)
{
    double A1 = exp(-0.5 * (rho - nu) * (rho - nu));
    double A2 = exp(-0.5 * (rho + nu) * (rho + nu));
    return rho * (A1 + A2) * fun_I0_hat(rho * nu);
}

double fun_f_hat(double nu, double rho, double rhobar)
{
    double A1 = exp(-0.5 * (rho - nu) * (rho - nu) + 0.5 * (rhobar - nu) * (rhobar - nu));
    double A2 = exp(-0.5 * (rho + nu) * (rho + nu) + 0.5 * (rhobar - nu) * (rhobar - nu));
    return rho * (A1 + A2) * fun_I0_hat(rho * nu);
}

double max(double a, double b)
{
    if (a >= b)
    {
        return a;
    }
    else
    {
        return b;
    }
}

double min(double a, double b)
{
    if (a >= b)
    {
        return b;
    }
    else
    {
        return a;
    }
}

double Int(double v, double h, double L)
{
    if (abs(v) <= L)
    {
        double A1 = exp(-(L - v) * (L - v) / (2 * h * h)) * fun_J((v - L) / (SQRT2 * h));
        double A2 = exp(-(L + v) * (L + v) / (2 * h * h)) * fun_J((v + L) / (SQRT2 * h));
        return -h * h * log(SQRTHALFPI * (h / (2 * L)) * (2 - A1 - A2));
    }
    else
    {
        // The function is even
        v = abs(v);

        double A1 = fun_J((v - L) / (SQRT2 * h));
        double A2 = exp(-2 * L * v / (h * h)) * fun_J((v + L) / (SQRT2 * h));
        return 0.5 * (v - L) * (v - L) - h * h * log(SQRTHALFPI * (h / (2 * L)) * (A1 - A2));
    }
}

double Cir(double v, double h, double R)
{
    // The function should be called only for v >= 0
    v = abs(v);

    // Change here the Gauss-Legendre quadrature
    int N = 7;
    double node[N] = {-0.94910, -0.74153, -0.40584, 0, 0.40584, 0.74153, 0.94910};
    double weight[N] = {0.12948, 0.27970, 0.38183, 0.4179, 0.38183, 0.27970, 0.12948};
    // end

    double F_low, F_up, delta, rhobar, y;

    if (v <= R)
    {
        F_low = max(0, sqrt((v / h) * (v / h) + 1) - 3);
        F_up = min(R / h, sqrt((v / h) * (v / h) + 1) + 3);
        delta = 0.5 * (F_up - F_low);

        y = 0;
        for (int i = 0; i < N; i++)
        {
            y = y + weight[i] * fun_f(v / h, F_low + delta * (node[i] + 1));
        }
        y = delta * y;
        return -h * h * log(y * (h / R) * (h / R));
    }
    else
    {
        F_low = 0;
        F_up = R / h;
        delta = 0.5 * (F_up - F_low);
        rhobar = F_low + delta * (node[N - 1] + 1);

        y = 0;
        for (int i = 0; i < N; i++)
        {
            y = y + weight[i] * fun_f_hat(v / h, F_low + delta * (node[i] + 1), rhobar);
        }
        y = delta * y;
        return 0.5 * (v - h * rhobar) * (v - h * rhobar) - h * h * log(y * (h / R) * (h / R));
    }
}

double Sph(double v, double h, double R)
{

    // The function should be called only for v >= 0
    v = abs(v);

    double C = 3 * (h * h) / (2 * R * R * R);
    double A1, A2;
    if (v <= R)
    {
        if (v == 0)
        {
            return -h * h * log(C * (-2 * R * exp(-(R * R) / (2 * h * h)) + 2 * R * exp(-Int(0, h, R) / (h * h))));
        }
        else
        {
            A1 = exp(-((R + v) * (R + v) / (2 * h * h)));
            A2 = exp(-((R - v) * (R - v) / (2 * h * h)));
            return -h * h * log(C * (h * h * (A1 - A2) / v + 2 * R * exp(-Int(v, h, R) / (h * h))));
        }
    }
    else
    {
        A1 = exp(-(2 * R * v / (h * h)));
        A2 = 1;
        return 0.5 * (v - R) * (v - R) - h * h * log(C * (h * h * (A1 - A2) / v + 2 * R * exp((0.5 * (v - R) * (v - R) - Int(v, h, R)) / (h * h))));
    }
}

//

DistanceStruct GeometricPrimitives::computeDist(GeometricPrimitives *objA, GeometricPrimitives *objB, Vector3d pointA0, double h, double tol)
{
    Vector3d pointA = pointA0;
    Vector3d pointB;
    Vector3d pointAOld;

    do
    {
        pointAOld = pointA;
        pointB = objB->projection(pointA, h);
        pointA = objA->projection(pointB, h);
    } while ((pointA - pointAOld).norm() >= tol);

    DistanceStruct ds;
    ds.witnessA = pointA;
    ds.witnessB = pointB;

    if (h == 0)
    {
        ds.distance = (pointA - pointB).norm();
    }
    else
    {
        ds.distance = sqrt(pow(objA->distanceToPoint(pointB, h), 2) + pow(objB->distanceToPoint(pointA, h), 2) - pow((pointA - pointB).norm(), 2));
    }

    return ds;
}



// Box

Box::Box(Matrix4d htm0, double widthVal, double depthVal, double heightVal)
{
    htm = htm0;
    width = widthVal;
    depth = depthVal;
    height = heightVal;
}

AxisAlignedBoundingBox Box::getAABB()
{
    Vector3d p1 = this->width * this->htm.block<3, 1>(0, 0) + this->depth * this->htm.block<3, 1>(0, 1) + this->height * this->htm.block<3, 1>(0, 2);
    Vector3d p2 = -this->width * this->htm.block<3, 1>(0, 0) + this->depth * this->htm.block<3, 1>(0, 1) + this->height * this->htm.block<3, 1>(0, 2);
    Vector3d p3 = this->width * this->htm.block<3, 1>(0, 0) - this->depth * this->htm.block<3, 1>(0, 1) + this->height * this->htm.block<3, 1>(0, 2);
    Vector3d p4 = this->width * this->htm.block<3, 1>(0, 0) + this->depth * this->htm.block<3, 1>(0, 1) - this->height * this->htm.block<3, 1>(0, 2);

    AxisAlignedBoundingBox aabb;
    aabb.width = max(p1[0], max(p2[0], max(p3[0], p4[0])));
    aabb.depth = max(p1[1], max(p2[1], max(p3[1], p4[1])));
    aabb.height = max(p1[2], max(p2[2], max(p3[2], p4[2])));

    return aabb;
}

double Box::distanceToPoint(Vector3d point, double h)
{
    Matrix3d Q = this->htm.block<3, 3>(0, 0);
    Vector3d p = this->htm.block<3, 1>(0, 3);
    Vector3d point_mod = Q.transpose() * (point - p);

    if (h == 0)
    {
        double dx = point_mod[0] - min(max(point_mod[0], -this->width / 2), this->width / 2);
        double dy = point_mod[1] - min(max(point_mod[1], -this->depth / 2), this->depth / 2);
        double dz = point_mod[2] - min(max(point_mod[2], -this->height / 2), this->height / 2);
        return sqrt(dx * dx + dy * dy + dz * dz);
    }
    else
    {
        double dx = Int(point_mod[0], h, this->width / 2);
        double dy = Int(point_mod[1], h, this->depth / 2);
        double dz = Int(point_mod[2], h, this->height / 2);

        return sqrt(2 * (dx + dy + dz));
    }
}

Vector3d Box::projection(Vector3d point, double h)
{
    Matrix3d Q = this->htm.block<3, 3>(0, 0);
    Vector3d p = this->htm.block<3, 1>(0, 3);
    Vector3d point_mod = Q.transpose() * (point - p);
    Vector3d point_mod_proj;

    if (h == 0)
    {
        point_mod_proj[0] = min(max(point_mod[0], -this->width / 2), this->width / 2);
        point_mod_proj[1] = min(max(point_mod[1], -this->depth / 2), this->depth / 2);
        point_mod_proj[2] = min(max(point_mod[2], -this->height / 2), this->height / 2);
    }
    else
    {
        double delta = 0.01;
        double Dxp = Int(point_mod[0] + delta, h, this->width / 2);
        double Dxm = Int(point_mod[0] - delta, h, this->width / 2);
        double Dyp = Int(point_mod[1] + delta, h, this->depth / 2);
        double Dym = Int(point_mod[1] - delta, h, this->depth / 2);
        double Dzp = Int(point_mod[2] + delta, h, this->height / 2);
        double Dzm = Int(point_mod[2] - delta, h, this->height / 2);

        point_mod_proj[0] = point_mod[0] - (Dxp - Dxm) / (2 * delta);
        point_mod_proj[1] = point_mod[1] - (Dyp - Dym) / (2 * delta);
        point_mod_proj[2] = point_mod[2] - (Dzp - Dzm) / (2 * delta);
    }

    return Q * point_mod_proj + p;
}

GeometricPrimitives *Box::copy()
{
    Box *b = new Box(this->htm, this->width, this->depth, this->height);
    return b;
}

// Cylinder

Cylinder::Cylinder(Matrix4d htm0, double radiusVal, double heightVal)
{
    htm = htm0;
    radius = radiusVal;
    height = heightVal;
}

AxisAlignedBoundingBox Cylinder::getAABB()
{
    double dradius = 2 * this->radius;

    Vector3d p1 = dradius * this->htm.block<3, 1>(0, 0) + dradius * this->htm.block<3, 1>(0, 1) + this->height * this->htm.block<3, 1>(0, 2);
    Vector3d p2 = -dradius * this->htm.block<3, 1>(0, 0) + dradius * this->htm.block<3, 1>(0, 1) + this->height * this->htm.block<3, 1>(0, 2);
    Vector3d p3 = dradius * this->htm.block<3, 1>(0, 0) - dradius * this->htm.block<3, 1>(0, 1) + this->height * this->htm.block<3, 1>(0, 2);
    Vector3d p4 = dradius * this->htm.block<3, 1>(0, 0) + dradius * this->htm.block<3, 1>(0, 1) - this->height * this->htm.block<3, 1>(0, 2);

    AxisAlignedBoundingBox aabb;
    aabb.width = max(p1[0], max(p2[0], max(p3[0], p4[0])));
    aabb.depth = max(p1[1], max(p2[1], max(p3[1], p4[1])));
    aabb.height = max(p1[2], max(p2[2], max(p3[2], p4[2])));

    return aabb;
}

double Cylinder::distanceToPoint(Vector3d point, double h)
{
    Matrix3d Q = this->htm.block<3, 3>(0, 0);
    Vector3d p = this->htm.block<3, 1>(0, 3);
    Vector3d point_mod = Q.transpose() * (point - p);
    double dist = sqrt(pow(point_mod[0], 2) + pow(point_mod[1], 2));

    if (h == 0)
    {
        double dr = max(dist - this->radius, 0);
        double dz = point_mod[2] - min(max(point_mod[2], -this->height / 2), this->height / 2);
        return sqrt(dr * dr + dz * dz);
    }
    else
    {
        double dr = Cir(dist, h, this->radius);
        double dz = Int(point_mod[2], h, this->height / 2);

        return sqrt(2 * (dr + dz));
    }
}

Vector3d Cylinder::projection(Vector3d point, double h)
{
    Matrix3d Q = this->htm.block<3, 3>(0, 0);
    Vector3d p = this->htm.block<3, 1>(0, 3);
    Vector3d point_mod = Q.transpose() * (point - p);
    Vector3d point_mod_proj;

    if (h == 0)
    {
        double dist = sqrt(pow(point_mod[0], 2) + pow(point_mod[1], 2));

        if (dist > this->radius)
        {
            point_mod_proj[0] = this->radius * point_mod[0] / dist;
            point_mod_proj[1] = this->radius * point_mod[1] / dist;
        }

        point_mod_proj[2] = min(max(point_mod[2], -this->height / 2), this->height / 2);
    }
    else
    {
        double delta = 0.01;

        double r = sqrt(point_mod[0] * point_mod[0] + point_mod[1] * point_mod[1]);

        double Drp = Cir(r + delta, h, this->radius);
        double Drm = Cir(r - delta, h, this->radius);
        double Dzp = Int(point_mod[2] + delta, h, this->height / 2);
        double Dzm = Int(point_mod[2] - delta, h, this->height / 2);

        point_mod_proj[0] = point_mod[0] - ((Drp - Drm) / (2 * delta)) * point_mod[0] / r;
        point_mod_proj[1] = point_mod[1] - ((Drp - Drm) / (2 * delta)) * point_mod[1] / r;
        point_mod_proj[2] = point_mod[2] - (Dzp - Dzm) / (2 * delta);
    }

    return Q * point_mod_proj + p;
}

GeometricPrimitives *Cylinder::copy()
{
    Cylinder *c = new Cylinder(this->htm, this->radius, this->height);
    return c;
}

// Sphere

Sphere::Sphere(Matrix4d htm0, double radiusVal)
{
    htm = htm0;
    radius = radiusVal;
}

AxisAlignedBoundingBox Sphere::getAABB()
{

    AxisAlignedBoundingBox aabb;
    aabb.width = 2 * this->radius;
    aabb.depth = 2 * this->radius;
    aabb.height = 2 * this->radius;

    return aabb;
}

double Sphere::distanceToPoint(Vector3d point, double h)
{
    Vector3d p = this->htm.block<3, 1>(0, 3);
    double dist = (p - point).norm();
    if (h == 0)
    {
        return max(dist - this->radius, 0);
    }
    else
    {
        return sqrt(2 * Sph(dist, h, this->radius));
    }
}

Vector3d Sphere::projection(Vector3d point, double h)
{

    Vector3d p = this->htm.block<3, 1>(0, 3);
    Vector3d point_mod = point - p;
    Vector3d point_mod_proj;
    double dist = point_mod.norm();

    if (h == 0)
    {
        if (dist > this->radius)
        {
            point_mod_proj = this->radius * point_mod / dist;
        }
    }
    else
    {
        double delta = 0.01;
        double Drp = Sph(dist + delta, h, this->radius);
        double Drm = Sph(dist - delta, h, this->radius);

        point_mod_proj = point_mod - ((Drp - Drm) / (2 * delta)) * point_mod / dist;
    }

    return point_mod_proj + p;
}

GeometricPrimitives *Sphere::copy()
{
    Sphere *e = new Sphere(this->htm, this->radius);

    return e;
}
