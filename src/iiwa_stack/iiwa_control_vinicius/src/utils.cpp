
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
  rot << 1, 0, 0, 0, 0, cost, -sint, 0, 0, sint, cost, 0, 0, 0, 0, 1;

  return rot;
}
Matrix4d Utils::roty(double theta)
{
  double cost = cos(theta);
  double sint = sin(theta);

  Matrix4d rot;
  rot << cost, 0, sint, 0, 0, 1, 0, 0, -sint, 0, cost, 0, 0, 0, 0, 1;

  return rot;
}
Matrix4d Utils::rotz(double theta)
{
  double cost = cos(theta);
  double sint = sin(theta);

  Matrix4d rot;
  rot << cost, -sint, 0, 0, sint, cost, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  return rot;
}
Matrix4d Utils::trn(Vector3d v)
{
  Matrix4d trn;
  trn << 1, 0, 0, v(0), 0, 1, 0, v(1), 0, 0, 1, v(2), 0, 0, 0, 1;

  return trn;
}
Matrix4d Utils::trn(double x, double y, double z)
{
  Matrix4d trn;
  trn << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1;

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
  // int n = v.rows();
  VectorXd w = v;

  for (int i = indStart; i < indEnd; i++)
  {
    w[i] = (v[i] > 0 ? 1 : -1) * pow(abs(v[i]), h);
  }

  return w;
}
double Utils::rand(double vMin, double vMax)
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
  double cos_pitch = sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2));
  double sin_pitch = -rot(2, 0);
  Vector3d angles = Vector3d::Zero();

  if (cos_pitch > 0.001)
  {
    double roll = atan2(rot(2, 1), rot(2, 2));
    double pitch = atan2(-rot(2, 0), cos_pitch);
    double yaw = atan2(rot(1, 0), rot(0, 0));
    angles << roll, pitch, yaw;
    return angles;
  }
  else
  {
    if (sin_pitch > 0)
    {
      double pitch = M_PI / 2;
      double yaw = 0;
      double roll = atan2(rot(0, 1), rot(0, 2));
      angles << roll, pitch, yaw;
      return angles;
    }
    else
    {
      double pitch = -M_PI / 2;
      double yaw = 0;
      double roll = atan2(-rot(0, 1), -rot(0, 2));
      angles << roll, pitch, yaw;
      return angles;
    }
  }
}

double Utils::distanceAABB(GeometricPrimitives* objA, GeometricPrimitives* objB)
{
  AxisAlignedBoundingBox AABB_A = (*objA).getAABB();
  AxisAlignedBoundingBox AABB_B = (*objB).getAABB();

  Vector3d delta = (*objA).htm.block<3, 1>(0, 3) - (*objB).htm.block<3, 1>(0, 3);

  double dx = max(abs(delta[0]) - (AABB_A.width + AABB_B.width) / 2.0, 0.0);
  double dy = max(abs(delta[1]) - (AABB_A.depth + AABB_B.depth) / 2.0, 0.0);
  double dz = max(abs(delta[2]) - (AABB_A.height + AABB_B.height) / 2.0, 0.0);

  return sqrt(dx * dx + dy * dy + dz * dz);
}

vector<VectorXd> Utils::upsample(vector<VectorXd> points, double dist)
{
  vector<VectorXd> upsampledPoints;
  VectorXd currentPoint;
  VectorXd nextPoint;
  VectorXd dir;
  double curDist;
  int jmax;

  for (int i = 1; i < points.size(); i++)
  {
    currentPoint = points[i - 1];
    nextPoint = points[i];
    curDist = (currentPoint - nextPoint).norm();

    jmax = (int)floor(curDist / dist) - 1;

    dir = (nextPoint - currentPoint) / (0.0000001 + curDist);

    for (int j = 0; j <= jmax; j++)
    {
      upsampledPoints.push_back(currentPoint + ((double)j) * dist * dir);
    }
  }
  upsampledPoints.push_back(nextPoint);

  return upsampledPoints;
}

VectorFieldResult Utils::vectorField(VectorXd q, vector<VectorXd> points, double alpha, bool openCurve,
                                     double percentLengthStop)
{
  // Find the closest point in the curve
  double dmin = (points[0] - q).norm();
  double dminTemp;
  int ind = 0;
  vector<double> s;
  s.push_back(0);

  for (int i = 0; i < points.size(); i++)
  {
    dminTemp = (points[i] - q).norm();
    if (dminTemp < dmin)
    {
      dmin = dminTemp;
      ind = i;
    }
    if (i > 0) s.push_back(s[s.size() - 1] + (points[i] - points[i - 1]).norm());
  }

  VectorXd pi = points[ind];

  // Compute the normal vector
  VectorXd N = (pi - q) / ((pi - q).norm() + 0.00001);

  // Compute the tangent vector
  VectorXd T;

  if (openCurve)
  {
    if (ind == 0)
    {
      T = (points[1] - points[0]);
      // cout << "NextPoint: " << points[1] << endl;
      // cout << "PreviousPoint: " << points[0] << endl;
    }
    else
    {
      T = (points[ind] - points[ind - 1]);
      // cout << "NextPoint: " << points[ind] << endl;
      // cout << "PreviousPoint: " << points[ind - 1] << endl;
    }
  }
  else
  {
    if (ind == 0)
      T = (points[0] - points[points.size() - 1]);
    else
      T = (points[ind] - points[ind - 1]);
  }

  // Compute the G and H gains
  double G = (2 / M_PI) * atan(alpha * sqrt(dmin));
  double H = sqrt(1 - 0.9999 * G * G);

  T = T.normalized();

  // Compute the final vector field:
  VectorXd v = G * N + H * T;

  // cout << "normN: " << N.norm() << endl;
  // cout << "T: " << T.norm() << endl;
  // cout << "G: " << G << endl;
  // cout << "H: " << H << endl;
  // cout << "V: " << v.norm() << endl;

  // cOMPUTE THE MAXIMUM ERROR:
  double maxe = 0;
  for (int i = 0; i < 7; i++) maxe = max(maxe, abs(pi[i] - q[i]));

  cout << "maxError: " << maxe * (180 / M_PI) << endl;

  // Scale if the curve is open
  double sCurrent = s[ind];
  double sMaximum = s[s.size() - 1];
  double mult;
  if (openCurve)
  {
    mult = sqrt(abs((1 - (sCurrent - percentLengthStop * sMaximum) / ((1 - percentLengthStop) * sMaximum))));
    if (sCurrent > percentLengthStop * sMaximum) v = mult * v;
  }

  // if(openCurve)
  //     cout<<dmin<<std::endl;

  VectorFieldResult vfr;
  vfr.v = v;
  vfr.dist = dmin;
  vfr.ind = ind;
  vfr.pi = pi;

  return vfr;
}