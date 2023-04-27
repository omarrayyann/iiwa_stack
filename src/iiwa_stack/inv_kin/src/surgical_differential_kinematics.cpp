#include <iostream>
#include <stdio.h>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <string>
#include <math.h>
#include <vector>
#include <fstream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <sstream>
#include <stack>
#include "iiwa_msgs/CartesianPose.h"
#include "iiwa_msgs/JointQuantity.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointPositionVelocity.h"
#include "iiwa_msgs/JointQuantity.h"
#include "std_msgs/Float32MultiArray.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>
#include <conio.h>
#include "robot.cpp"
#include "utils.cpp"
#include "distance.cpp"
#include "OmniButtonEvent.h"

using namespace std;

double FILTER_PARAM = 0.01;
double VELCONVERT = 1;

ros::Publisher geoTouch;

class TimeSeries
{
public:
  vector<VectorXd> data;
  vector<double> timeStamp;

  void add(VectorXd v, double t)
  {
    data.push_back(v);
    timeStamp.push_back(t);
  }
  void add(double v, double t)
  {
    VectorXd vv = VectorXd::Zero(1);
    vv << v;
    data.push_back(vv);
    timeStamp.push_back(t);
  }

  int size() { return data.size(); }
  VectorXd atTime(double t)
  {
    if (t < timeStamp[0])
    {
      ROS_INFO_STREAM("U");
      return data[0];
    }

    int k = 0;
    while (!(timeStamp[k] <= t && timeStamp[k + 1] > t) && k + 1 < timeStamp.size()) k++;

    if (k + 1 == timeStamp.size())
    {
      ROS_INFO_STREAM("V");
      return data[k - 2];
    }
    else
    {
      double alpha = (t - timeStamp[k]) / (timeStamp[k + 1] - timeStamp[k]);
      return (1 - alpha) * data[k] + alpha * data[k + 1];
    }
  }
  string print(int n) { return std::to_string(timeStamp[n]) + " " + Utils::printVectorOctave(data[n]) + " "; }
};

ofstream g_fileDebug;
TimeSeries g_qTimeSeries;
TimeSeries g_qdTimeSeries;
TimeSeries g_taskTimeSeries;
TimeSeries g_pTimeSeries;
TimeSeries g_pdTimeSeries;
TimeSeries g_fpeTimeSeries;

struct Data
{
  VectorXd data;
  double timeStamp;
};

vector<Data> touchJoints;
vector<Data> touchJointsVelocities;
vector<Data> touchEEfPosition;
vector<Data> touchEEfVelocitites;

VectorXd q_kuka;

ros::Time startingTime;
double lastTimeEEfPosition;
double lastTimeJointsPosition;
ros::Publisher KUKAJointsPublisher;
ros::Publisher KUKAJointsVelocityPublisher;
ros::Publisher KUKAFRIPublisher;
double g_dt = 0.01;

bool readedJoints = false;
bool act = true;

int g_count = 0;

double getTime()
{
  return (ros::Time::now() - startingTime).toSec();
}

bool safetyCheck(VectorXd q)
{
  vector<string> color = {"red", "green", "blue", "orange", "magenta", "cyan", "black"};
  Box* obs1 = new Box(Utils::trn(-0.5 + 0.12, 0, 0.17 - 0.17 / 2), 1, 1, 0.17);
  Box* obs2 = new Box(Utils::trn(0.5, 0, -0.05), 1, 1, 0.1);
  vector<GeometricPrimitives*> obstacles;

  // obstacles.push_back(obs1);
  // obstacles.push_back(obs2);
  FreeConfigParam param;
  param.obstacles = obstacles;
  param.h = 0.0005;
  IgnoreColLinkObstacle ig1;
  ig1.linkNo = 0;
  ig1.obsNo = 0;
  IgnoreColLinkObstacle ig2;
  ig2.linkNo = 0;
  ig2.obsNo = 1;

  param.distSafeObs = 0.04;
  param.distSafeAuto = 0.02;
  param.distSafeJoint = 0;
  param.considerJointLimits = true;

  param.ignoreCol = {ig1, ig2};

  Manipulator manip = Manipulator::createKukaIIWAWithTool();

  FreeConfigResult fcr = manip.checkFreeConfig(q, param);

  string warntext;

  if (!fcr.isFree)
  {
    if (fcr.errorType == FreeConfigResult::ErrorType::autoCollision)
    {
      warntext = "Autocollision between link " + color[fcr.linkNumber1] + " and link " + color[fcr.linkNumber2];
      ROS_INFO_STREAM(warntext);
    }
    if (fcr.errorType == FreeConfigResult::ErrorType::obstacleCollision)
    {
      warntext =
          "Collision between link " + color[fcr.linkNumber1] + " and obstacle " + std::to_string(fcr.obstacleNumber);
      ROS_INFO_STREAM(warntext);
    }
    if (fcr.errorType == FreeConfigResult::ErrorType::lowerJointLimit)
    {
      cout << "#Lower joint violation for joint: " << fcr.jointNumber + 1 << " ( value: " << q[fcr.jointNumber]
           << ", allowed: " << manip.qMin[fcr.jointNumber] << ")" << std::endl;
    }
    if (fcr.errorType == FreeConfigResult::ErrorType::upperJointLimit)
    {
      cout << "#Upper joint violation for joint: " << fcr.jointNumber + 1 << " ( value: " << q[fcr.jointNumber]
           << ", allowed: " << manip.qMax[fcr.jointNumber] << ")" << std::endl;
    }
  }

  return fcr.isFree;
}

bool publishNewJointPosition(Manipulator iiwa, VectorXd q)
{
  // if (iiwa.fk(q).htmTool(2, 3) < 0.45 - 0.4)  //! safetyCheck(q)
  // {
  //   ROS_INFO("DID NOT SEND ANGLES - COLLISION");
  //   return false;
  // }

  std_msgs::Float64MultiArray messageArray;
  iiwa_msgs::JointPosition jointPosition;
  iiwa_msgs::JointQuantity quantity;

  messageArray.data = {q[0], q[1], q[2], q[3], q[4], q[5], q[6]};

  quantity.a1 = q[0];
  quantity.a2 = q[1];
  quantity.a3 = q[2];
  quantity.a4 = q[3];
  quantity.a5 = q[4];
  quantity.a6 = q[5];
  quantity.a7 = q[6];

  jointPosition.position = quantity;
  KUKAFRIPublisher.publish(messageArray);

  return true;
}

bool publishNewJointPositionVelocity(Manipulator iiwa, VectorXd q, VectorXd qdot)
{
  if (iiwa.fk(q).htmTool(2, 3) < 0.45 - 0.2)  //! safetyCheck(q)
  {
    ROS_INFO("DID NOT SEND ANGLES - COLLISION");
    return false;
  }

  iiwa_msgs::JointPositionVelocity jointPositionVelocity;

  jointPositionVelocity.position.a1 = q[0];
  jointPositionVelocity.position.a2 = q[1];
  jointPositionVelocity.position.a3 = q[2];
  jointPositionVelocity.position.a4 = q[3];
  jointPositionVelocity.position.a5 = q[4];
  jointPositionVelocity.position.a6 = q[5];
  jointPositionVelocity.position.a7 = q[6];

  jointPositionVelocity.velocity.a1 = qdot[0];
  jointPositionVelocity.velocity.a2 = qdot[1];
  jointPositionVelocity.velocity.a3 = qdot[2];
  jointPositionVelocity.velocity.a4 = qdot[3];
  jointPositionVelocity.velocity.a5 = qdot[4];
  jointPositionVelocity.velocity.a6 = qdot[5];
  jointPositionVelocity.velocity.a7 = qdot[6];

  KUKAJointsVelocityPublisher.publish(jointPositionVelocity);

  return true;
}

double sqrtsgn(double x)
{
  return ((x > 0) ? 1 : -1) * sqrt(abs(x));
}
VectorXd computeJointVelocitiesKuka3b(Manipulator iiwa, VectorXd q_kuka, Vector3d vlin_des, Vector3d pf)
{
  FKResult fkr = iiwa.jacGeo(q_kuka);

  double K = 2.0;

  Vector3d xe = fkr.htmTool.block<3, 1>(0, 0);
  Vector3d ye = fkr.htmTool.block<3, 1>(0, 1);
  Vector3d ze = fkr.htmTool.block<3, 1>(0, 2);
  Vector3d pe = fkr.htmTool.block<3, 1>(0, 3);

  MatrixXd Jv = fkr.jacTool.block<3, 7>(0, 0);
  MatrixXd Jw = fkr.jacTool.block<3, 7>(3, 0);

  double f1 = (xe.transpose() * (pe - pf))[0];
  double f2 = (ye.transpose() * (pe - pf))[0];
  double f3 = (ze.transpose() * (pe - pf))[0];
  MatrixXd jacf1 = xe.transpose() * Jv - (pe - pf).transpose() * Utils::S(xe) * Jw;
  MatrixXd jacf2 = ye.transpose() * Jv - (pe - pf).transpose() * Utils::S(ye) * Jw;

  MatrixXd A1 = Utils::matrixVertStack(jacf1, jacf2);
  VectorXd b1 = Utils::vectorVertStack(-K * sqrtsgn(f1), -K * sqrtsgn(f2));

  ROS_INFO_STREAM(" ");
  double disfulc = round(1000 * 10 * sqrt(f1 * f1 + f2 * f2)) / 10;
  // ROS_INFO_STREAM("fx = " << f1);
  // ROS_INFO_STREAM("fy = " << f2);
  ROS_INFO_STREAM("d = " << disfulc);
  ROS_INFO_STREAM("fz = " << f3);

  MatrixXd A2 = Jv;
  VectorXd b2 = vlin_des;

  vector<MatrixXd> A = {A1, A2};
  vector<VectorXd> b = {b1, b2};

  VectorXd qdot = Utils::hierarchicalSolve(A, b, 0.001);

  return qdot;
}

double gammafun(double x, double xc, double k0, double kinf)
{
  if (x < 0)
    return -gammafun(-x, xc, k0, kinf);
  else
    return min(k0 * x, k0 * xc + kinf * (x - xc));
}

ofstream compTime;

VectorXd computeJointVelocitiesKuka4(Manipulator iiwa, VectorXd q, Vector3d vlin_des, Vector3d pf)
{
  double algorithmStartTime = (ros::Time::now() - startingTime).toSec();

  double K = 0.5;
  FulcrumPointResult fpResult = iiwa.computeFulcrumPoint(pf, q);
  MatrixXd A1 = Utils::matrixVertStack(fpResult.jacfx, fpResult.jacfy);

  VectorXd b1 = Utils::vectorVertStack(-K * fpResult.fx, -K * fpResult.fy);

  vlin_des[0] = 2 * vlin_des[0];
  vlin_des[1] = 1.5 * vlin_des[1];

  MatrixXd A2 = fpResult.fkr.jacTool.block<3, 7>(0, 0);

  VectorXd b2 = vlin_des;

  vector<MatrixXd> A = {A1, A2};
  vector<VectorXd> b = {b1, b2};

  MatrixXd G = Utils::matrixVertStack(MatrixXd::Identity(7, 7), -MatrixXd::Identity(7, 7));
  VectorXd g = -0.5 * VectorXd::Ones(14);

  VectorXd qd = Utils::solveQP(2 * (A2.transpose() * A2 + 0.01 * MatrixXd::Identity(7, 7)), -2 * A2.transpose() * b2, G,
                               g, A1, b1);

  double algorithmEndTime = (ros::Time::now() - startingTime).toSec();

  g_count++;

  if (g_count % 50 == 0)
  {
    ROS_INFO_STREAM("----------");
    ROS_INFO_STREAM("df = " << round(1000 * fpResult.df) << " (mm)");
    ROS_INFO_STREAM("vd = " << Utils::printVector(vlin_des));
    ROS_INFO_STREAM("vr = " << Utils::printVector(A2 * qd));
  }

  // Calculating the Algorithm Run Time
  double timeDifference = algorithmStartTime - algorithmEndTime;
  // cout << "Computational Time: " << timeDifference * 1000 << " ms" << endl;
  // cout << "Max Frequency: " << 1 / timeDifference << " Hz" << endl;

  return qd;
}

VectorXd computeJointVelocitiesKuka3(Manipulator iiwa, VectorXd q_kuka, Vector3d vlin_des, Vector3d pf)
{
  FKResult fkr = iiwa.jacGeo(q_kuka);

  double dt = 0.01;
  double dti = 0.00025;

  double K = 0.5;

  Vector3d xe = fkr.htmTool.block<3, 1>(0, 0);
  Vector3d ye = fkr.htmTool.block<3, 1>(0, 1);
  Vector3d ze = fkr.htmTool.block<3, 1>(0, 2);
  Vector3d pe = fkr.htmTool.block<3, 1>(0, 3);

  MatrixXd Jv = fkr.jacTool.block<3, 7>(0, 0);
  MatrixXd Jw = fkr.jacTool.block<3, 7>(3, 0);

  double f1 = (xe.transpose() * (pe - pf))[0];
  double f2 = (ye.transpose() * (pe - pf))[0];
  double f3 = (ze.transpose() * (pe - pf))[0];
  MatrixXd jacf1 = xe.transpose() * Jv - (pe - pf).transpose() * Utils::S(xe) * Jw;
  MatrixXd jacf2 = ye.transpose() * Jv - (pe - pf).transpose() * Utils::S(ye) * Jw;

  MatrixXd A1 = Utils::matrixVertStack(jacf1, jacf2);
  VectorXd b1 = Utils::vectorVertStack(-K * f1, -K * f2);

  ROS_INFO_STREAM(" ");
  // ROS_INFO_STREAM("fx = " << round(1000 * f1) << "(mm)");
  // ROS_INFO_STREAM("fy = " << round(1000 * f2) << "(mm)");
  ROS_INFO_STREAM("df = " << (1000 * sqrt(f1 * f1 + f2 * f2)) << " (mm)");
  ROS_INFO_STREAM("fz = " << (1000 * f3) << " (mm)");

  g_pTimeSeries.add(pe, getTime());
  g_fpeTimeSeries.add(f3, getTime());

  MatrixXd A2 = Jv;
  VectorXd b2 = vlin_des;

  vector<MatrixXd> A = {A1, A2};
  vector<VectorXd> b = {b1, b2};

  VectorXd qdot = Utils::hierarchicalSolve(A, b, 0.01);

  VectorXd q = q_kuka;
  vlin_des[0] = 2 * vlin_des[0];
  vlin_des[1] = 1.5 * vlin_des[1];

  for (int k = 0; k < round(dt / dti); k++)
  {
    fkr = iiwa.jacGeo(q);
    xe = fkr.htmTool.block<3, 1>(0, 0);
    ye = fkr.htmTool.block<3, 1>(0, 1);
    ze = fkr.htmTool.block<3, 1>(0, 2);
    pe = fkr.htmTool.block<3, 1>(0, 3);

    Jv = fkr.jacTool.block<3, 7>(0, 0);
    Jw = fkr.jacTool.block<3, 7>(3, 0);

    f1 = (xe.transpose() * (pe - pf))[0];
    f2 = (ye.transpose() * (pe - pf))[0];
    f3 = (ze.transpose() * (pe - pf))[0];
    jacf1 = xe.transpose() * Jv - (pe - pf).transpose() * Utils::S(xe) * Jw;
    jacf2 = ye.transpose() * Jv - (pe - pf).transpose() * Utils::S(ye) * Jw;

    A1 = Utils::matrixVertStack(jacf1, jacf2);
    double f1c = -gammafun(f1, 0.01, 150, 2.0);
    double f2c = -gammafun(f2, 0.01, 150, 2.0);
    b1 = Utils::vectorVertStack(f1c, f2c);

    A2 = Jv;
    b2 = vlin_des;

    A = {A1, A2};
    b = {b1, b2};

    // VectorXd qd = (Utils::hierarchicalSolve(A, b, 0.001));
    VectorXd qd = Utils::solveQP(2 * (A2.transpose() * A2 + 0.01 * MatrixXd::Identity(7, 7)), -2 * A2.transpose() * b2,
                                 MatrixXd::Zero(0, 0), VectorXd::Zero(0), A1, b1);

    q += dti * qd;
  }

  qdot = 0.35 * (q - q_kuka) / (dt);

  ROS_INFO_STREAM("dftg  = " << round(1000 * sqrt(f1 * f1 + f2 * f2)) << " (mm)");

  return qdot;
}

VectorXd computeJointVelocitiesKuka5(Manipulator iiwa, VectorXd q_kuka, Vector3d vlin_des, Vector3d pf)
{
  double algorithmStartTime = (ros::Time::now() - startingTime).toSec();

  // Algorithm Parameters
  double K = 2.0;

  FulcrumPointResult fpResult = iiwa.computeFulcrumPoint(pf, q_kuka);

  MatrixXd A1 = Utils::matrixVertStack(fpResult.jacfx, fpResult.jacfy);
  VectorXd b1 = Utils::vectorVertStack(-K * fpResult.fx, -K * fpResult.fy);

  vlin_des[0] = 2 * vlin_des[0];
  vlin_des[1] = 1.5 * vlin_des[1];

  // ROS_INFO_STREAM("fz = " << round(1000 * fpResult.fz) << " (mm)");

  MatrixXd A2 = fpResult.fkr.jacTool.block<3, 7>(0, 0);
  VectorXd b2 = vlin_des;

  vector<MatrixXd> A = {A1, A2};
  vector<VectorXd> b = {b1, b2};

  VectorXd qdot = Utils::hierarchicalSolve(A, b, 0.0001);

  FulcrumPointResult fpResult_next = iiwa.computeFulcrumPoint(pf, q_kuka + g_dt * qdot);

  g_count++;

  if (g_count % 50 == 0)
  {
    ROS_INFO_STREAM("--------------------");
    ROS_INFO_STREAM("df = " << round(10000 * fpResult.df) / 10 << " (mm)");
    ROS_INFO_STREAM("dftg  = " << round(10000 * fpResult_next.df) / 10 << " (mm)");
    ROS_INFO_STREAM("A1qdot = " << Utils::printVector(A1 * qdot));
    ROS_INFO_STREAM("b1 = " << Utils::printVector(b1));

    double dxcalc = (fpResult_next.fx - fpResult.fx) / (g_dt);
    double dxana = (fpResult.jacfx * qdot)(0, 0);

    double dycalc = (fpResult_next.fy - fpResult.fy) / (g_dt);
    double dyana = (fpResult.jacfy * qdot)(0, 0);

    ROS_INFO_STREAM("fx = " << fpResult.fx);
    ROS_INFO_STREAM("fy = " << fpResult.fy);
    ROS_INFO_STREAM("jacfx = " << fpResult.jacfx);
    ROS_INFO_STREAM("jacfy = " << fpResult.jacfy);

    ROS_INFO_STREAM("dxcalc = " << dxcalc);
    ROS_INFO_STREAM("dxana = " << dxana);
    ROS_INFO_STREAM("dycalc = " << dycalc);
    ROS_INFO_STREAM("dyana = " << dyana);
  }

  // cout << "Max Frequency: " << 1 / timeDifference << " Hz" << endl;

  return qdot;
}

VectorXd computeJointVelocitiesKuka2(Manipulator iiwa, VectorXd q_kuka, Vector3d vlin_des, Vector3d vang_des)
{
  FKResult fkr = iiwa.jacGeo(q_kuka);

  MatrixXd Jv = fkr.jacTool.block<3, 7>(0, 0);
  MatrixXd Jw = fkr.jacTool.block<3, 7>(3, 0);

  MatrixXd A = Utils::matrixVertStack(Jv, Jw);
  VectorXd b = Utils::vectorVertStack(vlin_des, vang_des);

  VectorXd qdot = Utils::pinv(A, 0.01) * b;

  return qdot;
}

VectorXd computeJointVelocitiesKuka(Manipulator iiwa, VectorXd q_kuka, Vector3d vlin_des, Vector3d zef_des)
{
  FKResult fkr = iiwa.jacGeo(q_kuka);

  MatrixXd Jv = fkr.jacTool.block<3, 7>(0, 0);
  MatrixXd Jw = fkr.jacTool.block<3, 7>(3, 0);
  Vector3d zef = fkr.htmTool.block<3, 1>(0, 2);

  double K = 0.5;

  double r = 1 - (zef_des.transpose() * zef)[0];
  MatrixXd Jr = zef_des.transpose() * Utils::S(zef) * Jw;

  MatrixXd A = Utils::matrixVertStack(Jv, Jr);
  VectorXd b = Utils::vectorVertStack(vlin_des, -K * r);

  VectorXd qdot = Utils::pinv(A, 0.01) * b;

  return qdot;
}

void touchCallEEFPosition(const geometry_msgs::PoseStamped msg)
{
  double currentTime = (ros::Time::now() - startingTime).toSec();
  double timeDifference = currentTime - lastTimeEEfPosition;
  lastTimeEEfPosition = currentTime;

  // Storing the Current EEF Position to the Vector touchEEF (x,y,z)

  VectorXd currentPosition(3);
  currentPosition << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  Data currentPositionData;
  currentPositionData.data = currentPosition;
  currentPositionData.timeStamp = currentTime;
  touchEEfPosition.push_back(currentPositionData);
  // ROS_INFO_STREAM(Utils::printVector(currentPositionData.data));

  if (touchEEfPosition.size() > 1)
  {
    VectorXd unfiltVelocity =
        (currentPosition - touchEEfPosition.at(touchEEfPosition.size() - 2).data) / timeDifference;
    Data currentVelocitiesData;
    currentVelocitiesData.data =
        FILTER_PARAM * unfiltVelocity + (1 - FILTER_PARAM) * touchEEfVelocitites[touchEEfVelocitites.size() - 1].data;

    currentVelocitiesData.timeStamp = currentTime;
    touchEEfVelocitites.push_back(currentVelocitiesData);
    // ROS_INFO_STREAM(Utils::printVector(100 * currentVelocitiesData.data));
  }
  else
  {
    Data currentVelocitiesData;
    currentVelocitiesData.data = VectorXd::Zero(3);
    currentVelocitiesData.timeStamp = currentTime;
    touchEEfVelocitites.push_back(currentVelocitiesData);
  }
}

void touchCallJoints(const sensor_msgs::JointState msg)
{
  double currentTime = (ros::Time::now() - startingTime).toSec();
  double timeDifference = currentTime - lastTimeJointsPosition;
  lastTimeJointsPosition = currentTime;

  // Storing the Current Joints to the Vector touchJoints ("Waist", "Shoulder", "Elbow", "Yaw", "Pitch", "Roll")
  VectorXd unfiltCurrentPosition(5);
  VectorXd currentPosition(5);
  unfiltCurrentPosition << msg.position[0], msg.position[1], msg.position[2], msg.position[3], -msg.position[4];

  if (touchJoints.size() > 1)
  {
    currentPosition = (0.05) * unfiltCurrentPosition + 0.95 * touchJoints.at(touchJoints.size() - 1).data;
  }
  else
  {
    currentPosition = unfiltCurrentPosition;
  }

  Data currentPositionData;
  currentPositionData.data = currentPosition;
  currentPositionData.timeStamp = currentTime;

  touchJoints.push_back(currentPositionData);

  // Storing the Current Velocities to the Vector touchJointsVelocities (Note: First is set to 0)

  if (touchJoints.size() > 1)
  {
    VectorXd unfiltVelocity = (currentPosition - touchJoints.at(touchJoints.size() - 2).data) / timeDifference;
    Data currentVelocitiesData;
    currentVelocitiesData.data = FILTER_PARAM * unfiltVelocity +
                                 (1 - FILTER_PARAM) * touchJointsVelocities[touchJointsVelocities.size() - 1].data;
    currentVelocitiesData.timeStamp = currentTime;
    touchJointsVelocities.push_back(currentVelocitiesData);
    // ROS_INFO_STREAM(Utils::printVector((180 / 3.14) * currentVelocitiesData.data));
  }
  else
  {
    Data currentVelocitiesData;
    currentVelocitiesData.data = VectorXd::Zero(5);
    currentVelocitiesData.timeStamp = currentTime;
    touchJointsVelocities.push_back(currentVelocitiesData);
  }
}

void kukaCallJoints(const sensor_msgs::JointState msg)
{
  VectorXd q_kuka_temp = VectorXd::Zero(7);
  q_kuka_temp << msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5],
      msg.position[6];
  q_kuka = q_kuka_temp;
  readedJoints = true;
  //  ROS_INFO_STREAM("reajoint");
}

void touchCallButton(const omni_msgs::OmniButtonEvent msg)
{
  act = msg.grey_button == 0;
}

ofstream file;

void printToFile()
{
  g_fileDebug.open("/home/cair1/Desktop/octavelogs/kukaFPlogs.m");

  g_fileDebug << "q = [];" << std::endl;
  for (int i = 0; i < g_qTimeSeries.size(); i++)
    g_fileDebug << "q = [q; " << g_qTimeSeries.print(i) << "];" << std::endl;

  g_fileDebug << "qd = [];" << std::endl;
  for (int i = 0; i < g_qdTimeSeries.size(); i++)
    g_fileDebug << "qd = [qd; " << g_qdTimeSeries.print(i) << "];" << std::endl;

  g_fileDebug << "taskStart = [];" << std::endl;
  for (int i = 0; i < g_taskTimeSeries.size(); i++)
    g_fileDebug << "taskStart = [taskStart; " << g_taskTimeSeries.print(i) << "];" << std::endl;

  g_fileDebug << "p = [];" << std::endl;
  for (int i = 0; i < g_pTimeSeries.size(); i++)
    g_fileDebug << "p = [p; " << g_pTimeSeries.print(i) << "];" << std::endl;

  g_fileDebug << "pd = [];" << std::endl;
  for (int i = 0; i < g_pdTimeSeries.size(); i++)
    g_fileDebug << "pd = [pd; " << g_pdTimeSeries.print(i) << "];" << std::endl;

  g_fileDebug << "fpe = [];" << std::endl;
  for (int i = 0; i < g_fpeTimeSeries.size(); i++)
    g_fileDebug << "fpe = [fpe; " << g_fpeTimeSeries.print(i) << "];" << std::endl;
}

int main(int argc, char* argv[])
{
  compTime.open("CompTime.txt");
  file.open("test.m");
  ros::init(argc, argv, "surgical_differential_kinematics");
  ros::NodeHandle n;
  ros::Time startingTime = ros::Time::now();

  ros::Subscriber KUKASubscriber1 = n.subscribe("/iiwa/joint_states", 100, kukaCallJoints);

  ros::Subscriber touchSubscriber1 = n.subscribe("/phantom/joint_states", 100, touchCallJoints);
  ros::Subscriber touchSubscriber2 = n.subscribe("/phantom/pose", 100, touchCallEEFPosition);
  ros::Subscriber touchSubscriber23 = n.subscribe("/phantom/button", 100, touchCallButton);
  // KUKAJointsPublisher = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 2);
  // KUKAJointsVelocityPublisher = n.advertise<iiwa_msgs::JointPositionVelocity>("/iiwa/command/JointPositionVelocity",
  // 1);

  KUKAFRIPublisher = n.advertise<std_msgs::Float64MultiArray>("/iiwa/PositionController/command", 1);

  ros::Rate loop_rate(500);

  Vector3d zef_des;
  zef_des << 0, 0, -1;
  Vector3d zlin_des;
  VectorXd q_kuka_next;
  VectorXd qdot_kuka = VectorXd::Zero(7);
  double dt = 0.01;

  Manipulator iiwa = Manipulator::createKukaIIWA();
  // Matrix4d htmStart = Utils::trn(0.45, 0, 0.65) * Utils::roty(3.14);
  Matrix4d htmStart = Utils::trn(0.45, 0, 0.55 - 0.4) * Utils::roty(3.14);
  //  Matrix4d htmStart = Utils::trn(0.45 + 0.4, 0, 0.7) * Utils::roty(3.14 / 2);
  //    Matrix4d htmStart = Utils::trn(0.4, 0, -0.4) * iiwa.fk().htmTool * Utils::roty(3.14 / 2);
  VelocityConstControlParam param;
  param.taskHtm = htmStart;
  param.obstacles = {};
  param.considerActionLimits = false;
  param.considerAutoCollision = false;
  param.considerJointLimits = true;
  param.kori = 0.4;
  iiwa.qDotMin = 0.5 * iiwa.qDotMin;
  iiwa.qDotMax = 0.5 * iiwa.qDotMax;

  Vector3d fp = htmStart.block<3, 1>(0, 3);
  fp[2] += 0.10;

  // ROS_INFO_STREAM(Utils::printMatrix(htmStart));

  bool reachedStartingPosition = false;  // false
  bool reachedZero = true;

  VectorXd q_kuka_sim = VectorXd::Zero(7);

  file << "vlin=[];" << std::endl;
  file << "t=[];" << std::endl;
  file << "qgt=[];" << std::endl;
  file << "qdotgt=[];" << std::endl;

  while (ros::ok())
  {
    // ROS_INFO_STREAM("runn");
    //  reachedStartingPosition = true;
    if (reachedStartingPosition)
    {
      g_qTimeSeries.add(q_kuka, getTime());

      double tt = (ros::Time::now() - startingTime).toSec();
      Vector3d vlin_des;
      vlin_des << 0, 0.05 * sin(0.6 / 2 * tt), 0;
      VectorXd qdot_kuka_next = computeJointVelocitiesKuka3(iiwa, q_kuka, vlin_des, fp);
      q_kuka_next = q_kuka + g_dt * qdot_kuka_next;

      publishNewJointPosition(iiwa, q_kuka_next);
    }
    else
    {
      if (reachedZero && readedJoints)
      {
        ConstControlResult ccr = iiwa.velocityConstControl(q_kuka, param);
        qdot_kuka = ccr.action;
        q_kuka_next = q_kuka + 1.5 * g_dt * qdot_kuka;

        ROS_INFO_STREAM("qdot: " << Utils::printVector(qdot_kuka));
        ROS_INFO_STREAM("qnext: " << Utils::printVector(q_kuka_next));

        ROS_INFO_STREAM("Task: " << Utils::printVector(ccr.taskResult.task));

        double t = (ros::Time::now() - startingTime).toSec();

        // double q1d = 0.3 * sin(2 * 3.14 * t / 6.0);
        // double q1dd = q1d + 0 * (q1d - q_kuka[0]);
        // q_kuka_next << q1dd, 0, 0, 0, 0, 0, 0;

        publishNewJointPosition(iiwa, q_kuka_next);
        reachedStartingPosition = ccr.taskResult.task.norm() <= 0.01;

        if (reachedStartingPosition)
        {
          ros::Time startingTime = ros::Time::now();
        }
        ROS_INFO_STREAM(reachedStartingPosition);

        file << "qgt=[qgt;" << Utils::printVectorOctave(q_kuka) << "];" << std::endl;
        file << "qdotgt=[qdotgt;" << Utils::printVectorOctave(q_kuka_next) << "];" << std::endl;
        file << "t=[t;" << t << "];" << std::endl;
      }
      else
      {
        if (readedJoints)
        {
          publishNewJointPosition(iiwa, VectorXd::Zero(7));
          reachedZero = q_kuka.norm() <= 0.02;
          ROS_INFO_STREAM("Sending to zero...");
        }
      }
    }

    // Get the current desired linear velocity
    ros::spinOnce();
    loop_rate.sleep();
  }
  printToFile();
}