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



DistanceStruct GeometricPrimitives::computeDist(GeometricPrimitives* objA, GeometricPrimitives* objB, Vector3d pointA0, double tol)
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

//Box

Box::Box(Matrix4d htm0, double widthVal, double depthVal, double heightVal)
{
    htm = htm0;
    width = widthVal;
    depth = depthVal;
    height = heightVal;
}

AxisAlignedBoundingBox Box::getAABB()
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

Vector3d Box::projection(Vector3d point)
{
    Matrix3d Q = this->htm.block<3,3>(0,0);
    Vector3d p = this->htm.block<3,1>(0,3);
    Vector3d point_mod = Q.transpose() * (point - p);

    point_mod[0] = min(max(point_mod[0], -this->width/2), this->width/2);
    point_mod[1] = min(max(point_mod[1], -this->depth/2), this->depth/2);
    point_mod[2] = min(max(point_mod[2], -this->height/2), this->height/2);

    return Q * point_mod + p;
}

GeometricPrimitives* Box::copy()
{
    Box *b = new Box(this->htm, this->width, this->depth, this->height);
    return b;
}

//Cylinder

Cylinder::Cylinder(Matrix4d htm0, double radiusVal, double heightVal)
{
    htm = htm0;
    radius = radiusVal;
    height = heightVal;
}

AxisAlignedBoundingBox Cylinder::getAABB()
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

Vector3d Cylinder::projection(Vector3d point)
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

GeometricPrimitives* Cylinder::copy()
{
    Cylinder *c = new Cylinder(this->htm, this->radius, this->height);
    return c;
}


//Sphere

Sphere::Sphere(Matrix4d htm0, double radiusVal)
{
    htm = htm0;
    radius = radiusVal;
}

AxisAlignedBoundingBox Sphere::getAABB()
{

    AxisAlignedBoundingBox aabb;
    aabb.width  = 2 * this-> radius;
    aabb.depth  = 2 * this-> radius;
    aabb.height = 2 * this-> radius;

    return aabb;
}

Vector3d Sphere::projection(Vector3d point)
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

GeometricPrimitives* Sphere::copy()
{
    Sphere *e = new Sphere(this->htm, this->radius);
    
    return e;
}
