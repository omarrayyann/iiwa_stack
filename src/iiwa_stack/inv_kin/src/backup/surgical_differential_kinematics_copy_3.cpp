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
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointQuantity.h"
#include "std_msgs/Float32MultiArray.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>
#include <conio.h>
#include "robot.cpp"
#include "utils.cpp"
#include "distance.cpp"

using namespace std;

double FILTER_PARAM = 0.01;
double VELCONVERT = 1;

ros::Publisher geoTouch;

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
bool readedJoints = false;

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

bool publishNewJointPosition(VectorXd q)
{
  if (false)  //(!safetyCheck(q))
  {
    ROS_INFO("DID NOT SEND ANGLES - COLLISION");
    return false;
  }

  std_msgs::Float32MultiArray messageArray;
  iiwa_msgs::JointPosition jointPosition;
  iiwa_msgs::JointQuantity quantity;

  quantity.a1 = q[0];
  quantity.a2 = q[1];
  quantity.a3 = q[2];
  quantity.a4 = q[3];
  quantity.a5 = q[4];
  quantity.a6 = q[5];
  quantity.a7 = q[6];

  jointPosition.position = quantity;
  KUKAJointsPublisher.publish(jointPosition);

  return true;
}

double sqrtsgn(double x)
{
  return ((x > 0) ? 1 : -1) * sqrt(abs(x));
}

struct FulcrumPointControlResult
{
  VectorXd action;
  double errorx;
  double errory;
  double distfulc;
  double introz;
  double errorpos;
  double residue;
  int iter_count;
};

FulcrumPointControlResult computeJointVelocitiesKuka4(Manipulator iiwa, VectorXd q_kuka, Vector3d pd, Vector3d pf)
{
  FKResult fkr = iiwa.jacGeo(q_kuka);

  double K = 0.5;  // 0.5

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

  MatrixXd A2 = Jv;
  VectorXd b2 = -K * (pe - pd);

  double distfulc = ((pe - pf) - ze * ze.transpose() * (pe - pf)).norm();

  VectorXd qdot;

  vector<MatrixXd> A = {A1, A2};
  vector<VectorXd> b = {b1, b2};
  qdot = Utils::hierarchicalSolve(A, b, 0.01);  // 0.01 //0.001

  FulcrumPointControlResult fpcr;

  fpcr.action = qdot;
  fpcr.errorx = 1000 * f1;
  fpcr.errory = 1000 * f2;
  fpcr.introz = 1000 * f3;
  fpcr.distfulc = 1000 * distfulc;
  fpcr.errorpos = 1000 * (pe - pd).norm();

  return fpcr;
}

FulcrumPointControlResult correctConfig(Manipulator iiwa, VectorXd q_kuka, Vector3d pf)
{
  double dt = 0.2;  // 0.08
  double K = 0.5;
  VectorXd q = q_kuka;
  double f1, f2, f3, distfulc;
  int count = 0;

  do
  {
    FKResult fkr = iiwa.jacGeo(q);

    Vector3d xe = fkr.htmTool.block<3, 1>(0, 0);
    Vector3d ye = fkr.htmTool.block<3, 1>(0, 1);
    Vector3d ze = fkr.htmTool.block<3, 1>(0, 2);
    Vector3d pe = fkr.htmTool.block<3, 1>(0, 3);

    MatrixXd Jv = fkr.jacTool.block<3, 7>(0, 0);
    MatrixXd Jw = fkr.jacTool.block<3, 7>(3, 0);

    f1 = (xe.transpose() * (pe - pf))[0];
    f2 = (ye.transpose() * (pe - pf))[0];
    f3 = (ze.transpose() * (pe - pf))[0];
    MatrixXd jacf1 = xe.transpose() * Jv - (pe - pf).transpose() * Utils::S(xe) * Jw;
    MatrixXd jacf2 = ye.transpose() * Jv - (pe - pf).transpose() * Utils::S(ye) * Jw;

    distfulc = sqrt(f1 * f1 + f2 * f2);

    q -= dt * (f1 * jacf1.transpose() + f2 * jacf2.transpose());
    count++;

  } while (distfulc >= 0.001);

  FulcrumPointControlResult fpcr;
  fpcr.action = q;
  fpcr.errorx = 1000 * f1;
  fpcr.errory = 1000 * f2;
  fpcr.introz = 1000 * f3;
  fpcr.distfulc = 1000 * distfulc;
  fpcr.residue = (q_kuka - q).norm();
  fpcr.iter_count = count;

  // ROS_INFO_STREAM("expdistfulc " << (round(10000 * distfulc) / 10) << " (mm)");

  return fpcr;
}

double computeFulcrumPointError(Manipulator iiwa, VectorXd q_kuka, Vector3d pf)
{
  FKResult fkr = iiwa.jacGeo(q_kuka);

  Vector3d xe = fkr.htmTool.block<3, 1>(0, 0);
  Vector3d ye = fkr.htmTool.block<3, 1>(0, 1);
  Vector3d ze = fkr.htmTool.block<3, 1>(0, 2);
  Vector3d pe = fkr.htmTool.block<3, 1>(0, 3);

  MatrixXd Jv = fkr.jacTool.block<3, 7>(0, 0);
  MatrixXd Jw = fkr.jacTool.block<3, 7>(3, 0);

  double f1 = (xe.transpose() * (pe - pf))[0];
  double f2 = (ye.transpose() * (pe - pf))[0];

  return 1000 * sqrt(f1 * f1 + f2 * f2);
}

FulcrumPointControlResult computeJointVelocitiesKuka3(Manipulator iiwa, VectorXd q_kuka, Vector3d vlin_des, Vector3d pf)
{
  FKResult fkr = iiwa.jacGeo(q_kuka);

  double K = 0.7;  // 0.5

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

  MatrixXd A2 = Jv;
  VectorXd b2 = vlin_des;
  double normb2 = b2.norm();
  if (normb2 > 1.0)
  {
    b2 = 1.0 * b2 / normb2;
  }
  b2[0] = 1.5 * b2[0];

  double distfulc = ((pe - pf) - ze * ze.transpose() * (pe - pf)).norm();

  VectorXd qdot;

  // vector<MatrixXd> A = {A1, A2};
  // vector<VectorXd> b = {b1, b2};
  vector<MatrixXd> A = {A2, A1};
  vector<VectorXd> b = {b2, b1};
  qdot = Utils::hierarchicalSolve(A, b, 0.01);  // 0.01 //0.001

  FulcrumPointControlResult fpcr;

  fpcr.action = qdot;
  fpcr.errorx = 1000 * f1;
  fpcr.errory = 1000 * f2;
  fpcr.introz = 1000 * f3;
  fpcr.distfulc = 1000 * distfulc;

  return fpcr;
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

void kukaCallJoints(const iiwa_msgs::JointPosition msg)
{
  q_kuka = VectorXd::Zero(7);
  q_kuka << msg.position.a1, msg.position.a2, msg.position.a3, msg.position.a4, msg.position.a5, msg.position.a6,
      msg.position.a7;
  readedJoints = true;
}

ofstream file;
ofstream file2;

int main(int argc, char* argv[])
{
  file.open("/home/cair1/Desktop/octavelogs/surgdata.m");
  file2.open("/home/cair1/PycharmProjects/uaibot_vinicius/test/test_fulcrum_point/ssg.py");
  ros::init(argc, argv, "surgical_differential_kinematics");
  ros::NodeHandle n;
  ros::Time startingTime = ros::Time::now();

  ros::Subscriber KUKASubscriber1 = n.subscribe("/iiwa/state/JointPosition", 1, kukaCallJoints);
  ros::Subscriber touchSubscriber1 = n.subscribe("/phantom/joint_states", 100, touchCallJoints);
  ros::Subscriber touchSubscriber2 = n.subscribe("/phantom/pose", 100, touchCallEEFPosition);
  KUKAJointsPublisher = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1);

  ros::Rate loop_rate(100);

  Vector3d zef_des;
  zef_des << 0, 0, -1;
  Vector3d zlin_des;
  VectorXd q_kuka_next;
  VectorXd qdot_kuka;
  double dt = 0.01;
  double t_start;

  Manipulator iiwa = Manipulator::createKukaIIWA();
  Manipulator iiwasurg = Manipulator::createKukaIIWASurgery();
  // Matrix4d htmStart = Utils::trn(0.45, 0, 0.65) * Utils::roty(3.14);
  double delta = 0.15;  // 0.1
  Matrix4d htmStart = Utils::trn(0.45 + delta, 0, 0.55 - delta) * Utils::roty(3.14);
  // Matrix4d htmStart = Utils::trn(0.45, 0, 0.55) * Utils::roty(3.14);
  //  Matrix4d htmStart = Utils::trn(0.4, 0, -0.4) * iiwa.fk().htmTool * Utils::roty(3.14 / 2);
  VelocityConstControlParam param;
  param.taskHtm = htmStart;
  param.obstacles = {};
  param.considerActionLimits = false;
  param.considerAutoCollision = false;
  param.considerJointLimits = false;
  param.kori = 0.4;

  Vector3d fp = htmStart.block<3, 1>(0, 3);
  fp[2] += -0.3;

  // ROS_INFO_STREAM(Utils::printMatrix(htmStart));

  bool reachedStartingPosition = false;  // false
  bool reachedZero = true;
  bool notOver = true;

  VectorXd q_kuka_sim = VectorXd::Zero(7);

  file << "t=[];" << std::endl;
  file << "qreal = [];" << std::endl;
  file << "qsent = [];" << std::endl;
  file << "distfulc = [];" << std::endl;

  file2 << "import uaibot as ub" << std::endl;
  file2 << "import numpy as np" << std::endl << std::endl;
  file2 << "kuka = ub.Robot.create_kuka_lbr_iiwa()" << std::endl;
  file2 << "fp_ball = ub.Ball(color='red', radius=0.01)" << std::endl;
  file2 << "tool = ub.Cylinder(color='black', radius=0.005, height=0.4)" << std::endl;
  file2 << "vlin = ub.Vector(color='blue')" << std::endl;
  file2 << "surgarea = ub.Box(width=0.14, depth=0.14, height=0.03, color='yellow')" << std::endl;
  file2 << "sim = ub.Simulation([kuka,fp_ball,tool,vlin, surgarea])" << std::endl;
  file2 << "sim.set_parameters(width= 1600, height= 950, background_color='gray')" << std::endl;
  file2 << "fp_ball.add_ani_frame(0,ub.Utils.trn(" << Utils::printVector(fp) << "))" << std::endl;

  // VectorXd q_try = VectorXd(7);
  // q_try << -0.00020, 0.13232, -0.00020, -1.61260, 0.00483, 1.36884, -0.00000;
  // cout << Utils::printMatrix(iiwa.taskFunction(htmStart, q_try).task) << std::endl;
  //  cout << Utils::printMatrix(htmStart) << std::endl;

  // rosservice call /iiwa/configuration/setSmartServoLimits 1.0  1.0 2.0

  ROS_INFO_STREAM("height" << iiwa.fk().htmTool(2, 3));

  int kcount = 0;

  FulcrumPointControlResult fpcr;

  while (ros::ok() && notOver)
  {
    if (reachedStartingPosition)
    {
      if (touchEEfVelocitites.size() > 1)
      {
        if (kcount % 1 == 0)
        {
          // ROS_INFO_STREAM("Listening to geotouch");

          Vector3d vlin_des_aux = touchEEfVelocitites[touchEEfVelocitites.size() - 1].data;
          Vector3d vlin_des;
          vlin_des << vlin_des_aux[1], -vlin_des_aux[0], vlin_des_aux[2];
          vlin_des = VELCONVERT * vlin_des;

          // vlin_des << 0, 0.3, 0;

          // ROS_INFO_STREAM("v (cm/s)" << Utils::printVector(vlin_des * 100));

          // ROS_INFO_STREAM("A");

          VectorXd q_gt = touchJoints[touchJoints.size() - 1].data;
          // ROS_INFO_STREAM("B");
          VectorXd qdot_gt = touchJointsVelocities[touchJointsVelocities.size() - 1].data;
          // ROS_INFO_STREAM("C");
          MatrixXd jacg = Manipulator::createGeoTouch().jacGeo(q_gt).jacTool;
          // ROS_INFO_STREAM("D");
          // ROS_INFO_STREAM(Utils::printMatrix(jacg));
          MatrixXd Jw_gt = jacg.block<3, 5>(3, 0);

          Vector3d vang_des_aux = Jw_gt * qdot_gt;

          // cout << Utils::printMatrix(Utils::rotz(-M_PI / 2) * Utils::rotx(M_PI / 2) *
          //                            Manipulator::createGeoTouch().jacGeo(q_gt).htmTool)
          //      << std::endl << std::endl;

          // cout << Utils::printMatrix(Manipulator::createGeoTouch().jacGeo(q_gt).htmTool) << std::endl << std::endl;
          // cout << Utils::printVector(vang_des_aux) << std::endl;

          Vector3d vang_des;
          vang_des << -vang_des_aux[2], -vang_des_aux[0], vang_des_aux[1];

          // ROS_INFO_STREAM("vang_des = " << Utils::printVector(vang_des));

          // ROS_INFO_STREAM("AAAA");

          // qdot_kuka = computeJointVelocitiesKuka(iiwa, q_kuka, vlin_des, zef_des);
          // qdot_kuka = computeJointVelocitiesKuka2(iiwa, q_kuka, vlin_des, vang_des);

          double fperror = computeFulcrumPointError(iiwasurg, q_kuka, fp);

          q_kuka_next = q_kuka;

          Vector3d pec = iiwasurg.fk(q_kuka).htmTool.block<3, 1>(0, 3);

          fpcr = computeJointVelocitiesKuka3(iiwasurg, q_kuka_next, 0.4 * vlin_des,
                                             fp);  // 0.1
          // fpcr = computeJointVelocitiesKuka4(iiwasurg, q_kuka_next, pec + 5 * dt * vlin_des, fp);
          qdot_kuka = fpcr.action;
          q_kuka_next += dt * qdot_kuka;
          fpcr = correctConfig(iiwasurg, q_kuka_next, fp);

          q_kuka_next = 0.05 * q_kuka + 0.95 * fpcr.action;  // q_kuka_next = 0.05 * q_kuka + 0.95 * fpcr.action;

          ROS_INFO_STREAM(" ");
          // ROS_INFO_STREAM("errorx = " << round(fpcr.errorx) << " mm");
          // ROS_INFO_STREAM("errory = " << round(fpcr.errory) << " mm");
          // ROS_INFO_STREAM("introz = " << round(fpcr.introz) << " mm");
          //  ROS_INFO_STREAM("errorp = " << round(fpcr.errorpos) << " mm");
          ROS_INFO_STREAM("distfulc = " << round(fperror) << " mm (ACTION)");
          // ROS_INFO_STREAM("residue = " << (180 / 3.14) * fpcr.residue << " ");
          // ROS_INFO_STREAM("itercount = " << fpcr.iter_count << " ");

          // ROS_INFO_STREAM("q = " << Utils::printVector(q_kuka));

          // notOver = fpcr.distfulc < 30.0;

          // ROS_INFO_STREAM("v = " << Utils::printVector(vlin_des) << ", w = " << Utils::printVector(vang_des));

          publishNewJointPosition(q_kuka_next);

          double t = (ros::Time::now() - startingTime).toSec() - t_start;
          // file << "vlin=[vlin;" << Utils::printVectorOctave(vlin_des) << "];" << std::endl;
          // file << "qgt=[qgt;" << Utils::printVectorOctave(touchJoints[touchJoints.size() - 1].data) << "];" <<
          // std::endl; file << "qdotgt=[qdotgt;"
          //      << Utils::printVectorOctave(touchJointsVelocities[touchJointsVelocities.size() - 1].data) << "];"
          //      << std::endl;
          file << "qreal = [qreal; " << Utils::printVectorOctave(q_kuka) << "];" << std::endl;
          file << "qsent = [qsent; " << Utils::printVectorOctave(q_kuka_next) << "];" << std::endl;
          file << "distfulc = [distfulc; " << fperror << "];" << std::endl;
          file << "t=[t;" << t << "];" << std::endl;

          // file2 << "kuka.add_ani_frame(" << t << ", " << Utils::printVector(q_kuka_next) << ")" << std::endl;
          file2 << "kuka.add_ani_frame(" << t << ", " << Utils::printVector(q_kuka) << ")" << std::endl;
          file2 << "vlin.add_ani_frame(" << t << ", " << Utils::printVector(pec) << ", "
                << Utils::printVector(0.3 * vlin_des) << ")" << std::endl;
        }
        else
        {
          double fperror = computeFulcrumPointError(iiwasurg, q_kuka, fp);
          ROS_INFO_STREAM(" ");
          // ROS_INFO_STREAM("errorx = " << round(fpcr.errorx) << " mm");
          // ROS_INFO_STREAM("errory = " << round(fpcr.errory) << " mm");
          // ROS_INFO_STREAM("introz = " << round(fpcr.introz) << " mm");
          //  ROS_INFO_STREAM("errorp = " << round(fpcr.errorpos) << " mm");
          ROS_INFO_STREAM("distfulc = " << round(fperror) << " mm");
          publishNewJointPosition(q_kuka_next);

          double t = (ros::Time::now() - startingTime).toSec() - t_start;
          // file << "vlin=[vlin;" << Utils::printVectorOctave(vlin_des) << "];" << std::endl;
          // file << "qgt=[qgt;" << Utils::printVectorOctave(touchJoints[touchJoints.size() - 1].data) << "];" <<
          // std::endl; file << "qdotgt=[qdotgt;"
          //      << Utils::printVectorOctave(touchJointsVelocities[touchJointsVelocities.size() - 1].data) << "];"
          //      << std::endl;
          file << "qreal = [qreal; " << Utils::printVectorOctave(q_kuka) << "];" << std::endl;
          file << "qsent = [qsent; " << Utils::printVectorOctave(q_kuka_next) << "];" << std::endl;
          file << "distfulc = [distfulc; " << fperror << "];" << std::endl;
          file << "t=[t;" << t << "];" << std::endl;

          // file2 << "kuka.add_ani_frame(" << t << ", " << Utils::printVector(q_kuka_next) << ")" << std::endl;
          file2 << "kuka.add_ani_frame(" << t << ", " << Utils::printVector(q_kuka) << ")" << std::endl;
          // file2 << "vlin.add_ani_frame(" << t << ", " << Utils::printVector(pec) << ", "
          //       << Utils::printVector(0.3 * vlin_des) << ")" << std::endl;
        }
        kcount++;
      }
    }
    else
    {
      if (reachedZero && readedJoints)
      {
        ConstControlResult ccr = iiwa.velocityConstControl(q_kuka, param);
        qdot_kuka = ccr.action;
        q_kuka_next = q_kuka + 10 * dt * qdot_kuka;
        // q_kuka_next = q_kuka + 1 * dt * qdot_kuka;
        ROS_INFO_STREAM("Task: " << Utils::printVector(ccr.taskResult.task));

        publishNewJointPosition(q_kuka_next);
        reachedStartingPosition = ccr.taskResult.task.norm() <= 0.01;
        if (reachedStartingPosition)
        {
          t_start = (ros::Time::now() - startingTime).toSec();
          file2 << "kuka.add_ani_frame(0, " << Utils::printVector(q_kuka_next) << ")" << std::endl;
          file2 << "htm_tool_0 = ub.Utils.trn([0,0,-0.2])*kuka.fkm()" << std::endl;
          file2 << "tool.add_ani_frame(0,htm_tool_0)" << std::endl;
          file2 << "surgarea.add_ani_frame(0,ub.Utils.trn([0,0,-0.415])*ub.Utils.trn(kuka.fkm()[0:3,-1]))" << std::endl;
          file2 << "kuka.attach_object(tool)" << std::endl;
        }
      }
      else
      {
        if (readedJoints)
        {
          publishNewJointPosition(VectorXd::Zero(7));
          reachedZero = q_kuka.norm() <= 0.02;
          ROS_INFO_STREAM("Sending to zero...");
        }
      }
    }

    // Get the current desired linear velocity
    ros::spinOnce();
    loop_rate.sleep();
  }
  file2 << "sim.save('/home/cair1/PycharmProjects/uaibot_vinicius/test/test_fulcrum_point', 'ssg')";
}
