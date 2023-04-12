#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>
#include <functional>
#include <boost/multi_array.hpp>
#include <typeinfo>

#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointPositionVelocity.h"
#include "iiwa_msgs/JointQuantity.h"

#include "utils.cpp"
#include "distance.cpp"
#include "robot.cpp"

#include <chrono>
using namespace std::chrono;

struct DFunctionStruct
{
  double D;
  double trueD;
  VectorXd gradD;
  vector<string> messages;
};

struct CBFCircControlResult
{
  VectorXd qdot;
  bool zeroTangent = false;
  bool feasible = true;
  DFunctionStruct dfunst;
  TaskResult tr;
};

// Global variables
double g_startTime;
double g_simTime = 0;
bool g_missionOver = false;
Manipulator g_Kuka = Manipulator::createKukaIIWAReduced();
VectorXd g_qKuka = VectorXd::Zero(7);
vector<VectorXd> g_histqKuka = {};
vector<double> g_histTime = {};
vector<double> g_histD = {};
ofstream g_file;
ofstream g_file_gen;
ros::Publisher g_KukaJointsPublisher;
VectorXd g_qdotKuka = VectorXd::Zero(7);
bool g_readedJoints = false;
Box* g_Platform;

// Parameters
vector<GeometricPrimitives*> PARAM_OBSTACLES = {};
double PARAM_H = 0.005;         // 0.02
double PARAM_DELTADIST = 0.02;  // 0.15 0.05 0.02
double PARAM_ALPHA = 0.3;
double PARAM_MAX_VEL = 0.07;  // 0.6
double PARAM_BETA_D0 = 0.04;  // 0.03 0.01
double PARAM_BETA_MAX_CIRC = 0.6 * PARAM_MAX_VEL;
bool PARAM_ISSIMULATION = true;
double PARAM_DT = 0.02;
double PARAM_SIMSPEEDUP = 5.0;
double PARAM_CONV_RAD_TO_METER = 10.0;
Matrix4d PARAM_HTM_TG = Utils::trn(0.5, -0.5, 0.6) * Utils::rotx(M_PI / 2);
// Matrix4d PARAM_HTM_TG = Utils::trn(0.5, -0.4, 0.6) * Utils::roty(M_PI / 2);
MatrixXd PARAM_OMEGA;
VectorXd PARAM_QKUKASETPOINT;
vector<string> PARAM_COLOR = {"blue", "green", "yellow", "magenta", "red", "cyan"};

double gammafun(double x, double xc, double k0, double kinf)
{
  if (x < 0)
    return -gammafun(-x, xc, k0, kinf);
  else
    return min(k0 * x, k0 * xc + kinf * (x - xc));
}

double alphafun(double d)
{
  return -PARAM_ALPHA * d;
}

double betafun(double d)
{
  return -100;
  // return PARAM_BETA_MAX_CIRC * min(1 - d / PARAM_BETA_D0, 1);
}

double getTime()
{
  if (!PARAM_ISSIMULATION)
    return ros::Time::now().toSec() - g_startTime;
  else
    return g_simTime;
}

void kukaCallJoints(const iiwa_msgs::JointPosition msg)
{
  g_qKuka = VectorXd::Zero(7);
  g_qKuka << msg.position.a1, msg.position.a2, msg.position.a3, msg.position.a4, msg.position.a5, msg.position.a6,
      msg.position.a7;
  g_readedJoints = true;
}

void sendJointVelocity(VectorXd qdot)
{
  if (PARAM_ISSIMULATION)
  {
    ROS_INFO_STREAM("Simulating joints...");
    ROS_INFO_STREAM(Utils::printVector(qdot));
    g_qKuka += qdot * 6 * PARAM_DT;

    g_simTime += PARAM_DT;
  }
  else
  {
    ROS_INFO_STREAM("Sending joints...");

    iiwa_msgs::JointPosition jointPosition;
    iiwa_msgs::JointQuantity quantity;

    VectorXd q = g_qKuka + 1.5 * 18 * qdot * PARAM_DT;

    if (g_Kuka.computeDistToObj(g_Platform).getClosest().distance > 0.02)
    {
      quantity.a1 = q[0];
      quantity.a2 = q[1];
      quantity.a3 = q[2];
      quantity.a4 = q[3];
      quantity.a5 = q[4];
      quantity.a6 = q[5];
      quantity.a7 = q[6];

      jointPosition.position = quantity;
      g_KukaJointsPublisher.publish(jointPosition);
    }
    else
    {
      ROS_INFO_STREAM("Danger... not sending");
    }
  }

  //
  /////
  g_file << "iiwa.add_ani_frame(time=" << getTime() << ", q = " << Utils::printVector(g_qKuka) << ")" << std::endl;
  g_Kuka.setConfig(g_qKuka);
  int no = 0;
  for (int k = 0; k < 7; k++)
  {
    for (int j = 0; j < g_Kuka.links[k].colObjs.size(); j++)
    {
      GeometricPrimitives* gp = g_Kuka.links[k].colObjs[j];
      g_file << "colobj[" << no << "].add_ani_frame(" << getTime() << "," << Utils::printMatrixPython(gp->htm) << ")"
             << std::endl;
      no++;
    }
  }

  DistanceRobotObjParam paramdro;

  g_file << std::endl << "#DIST" << std::endl;

  for (int k = 0; k < PARAM_OBSTACLES.size(); k++)
  {
    paramdro.h = 0.001;
    DistanceRobotObjResult dro1 = g_Kuka.computeDistToObj(PARAM_OBSTACLES[k], g_qKuka, paramdro);
    paramdro.h = 0;
    DistanceRobotObjResult dro2 = g_Kuka.computeDistToObj(PARAM_OBSTACLES[k], g_qKuka, paramdro);

    for (int n = 0; n < dro1.noElements; n++)
    {
      g_file << "#Dist link " << dro1[n].linkNumber << " and obstacle " << k << ". D_0.001 = " << dro1[n].distance
             << ", D_0 = " << dro2[n].distance << std::endl;
    }
  }

  ////
}

void sendJointAcceleration(VectorXd qddot)
{
  if (PARAM_ISSIMULATION)
  {
    g_qdotKuka += qddot * PARAM_DT;
    g_qKuka += g_qdotKuka * PARAM_DT;
    g_histqKuka.push_back(g_qKuka);
    g_histTime.push_back(getTime());

    g_simTime += PARAM_DT;
    g_file << "iiwa.add_ani_frame(time=" << getTime() << ", q = " << Utils::printVector(g_qKuka) << ")" << std::endl;
  }
  else
  {
  }
}

void generateOmega()
{
  MatrixXd bOmega = MatrixXd::Zero(g_qKuka.rows(), g_qKuka.rows());

  int nhalf = floor(g_qKuka.rows() / 2);
  for (int i = 0; i < nhalf; i++)
  {
    int i1 = i;
    int i2 = 2 * nhalf - 1 - i1;

    bOmega(i1, i2) = 1;
    bOmega(i2, i1) = -1;
  }

  MatrixXd B = MatrixXd::Zero(g_qKuka.rows(), g_qKuka.rows());
  for (int i = 0; i < g_qKuka.rows(); i++)
    for (int j = 0; j < g_qKuka.rows(); j++) B(i, j) = Utils::rand(-1, 1);

  B = B - B.transpose().eval();
  MatrixXd I = MatrixXd::Identity(g_qKuka.rows(), g_qKuka.rows());
  MatrixXd Q = (I - B).inverse() * (I + B);
  PARAM_OMEGA = Q * bOmega * Q.transpose();

  // PARAM_OMEGA = bOmega;
  //
  PARAM_OMEGA = MatrixXd::Zero(7, 7);
  PARAM_OMEGA(0, 1) = 1;
  PARAM_OMEGA(1, 0) = -1;
  PARAM_OMEGA(2, 3) = 1;
  PARAM_OMEGA(3, 2) = -1;
}

DFunctionStruct computeD(VectorXd q)
{
  vector<VectorXd> gradD_i;
  vector<VectorXd> gradD3D_i;
  vector<double> D_i;
  vector<string> messages;

  // Collision with obstacles

  for (int k = 0; k < PARAM_OBSTACLES.size(); k++)
  {
    DistanceRobotObjParam paramdro;
    paramdro.h = 0.0001;

    DistanceRobotObjResult dro = g_Kuka.computeDistToObj(PARAM_OBSTACLES[k], q, paramdro);

    for (int n = 0; n < dro.noElements; n++)
    {
      double D_i_temp = dro[n].distance;
      VectorXd gradD_i_temp = dro[n].jacDist;

      gradD_i.push_back(gradD_i_temp);
      D_i.push_back(D_i_temp);
      gradD3D_i.push_back((dro[n].witnessColObjLink - dro[n].witnessObj) / (0.000001 + D_i_temp));

      // ROS_INFO_STREAM("OBS " << k << ", LINK" << dro[n].linkNumber << " DIST: " << D_i_temp);

      if (D_i_temp < -0.03)
        messages.push_back("#Link " + std::to_string(dro[n].linkNumber) + " collided with obstacle " +
                           std::to_string(k) + " at time " + std::to_string(getTime()) + " with penetration " +
                           std::to_string(-D_i_temp));
    }
    // ROS_INFO_STREAM("OBS " << k << ", SHORTEST: " << dro.getClosest().distance);
  }

  // Joint limits
  for (int i = 0; i < 5; i++)  //(int i = 0; i < q.rows(); i++)
  {
    double D_i_temp;
    VectorXd gradD_i_temp = VectorXd::Zero(q.rows());

    D_i_temp = PARAM_CONV_RAD_TO_METER * (q[i] - g_Kuka.qMin[i]);
    gradD_i_temp[i] = PARAM_CONV_RAD_TO_METER;
    gradD_i.push_back(gradD_i_temp);
    D_i.push_back(D_i_temp);

    if (D_i_temp < -0.03)
      messages.push_back("#Joint " + std::to_string(i) + " violated lower limit at time " + std::to_string(getTime()) +
                         " with penetration " + std::to_string(-D_i_temp));

    D_i_temp = PARAM_CONV_RAD_TO_METER * (g_Kuka.qMax[i] - q[i]);
    gradD_i_temp[i] = -PARAM_CONV_RAD_TO_METER;
    gradD_i.push_back(gradD_i_temp);
    D_i.push_back(D_i_temp);

    if (D_i_temp < -0.03)
      messages.push_back("#Joint " + std::to_string(i) + " violated upper limit at time " + std::to_string(getTime()) +
                         " with penetration " + std::to_string(-D_i_temp));
  }

  // Compute the vector N and D

  for (int i = 0; i < D_i.size(); i++)
  {
    // ROS_INFO_STREAM("D = "<<D_i[i]<<", gradD = "<<Utils::printVector(gradD_i[i]));
  }

  SoftSelectMinResult ssmr = Utils::softSelectMin(D_i, gradD_i, PARAM_H);

  DFunctionStruct dfs;
  dfs.D = ssmr.softMin - PARAM_DELTADIST;
  dfs.trueD = ssmr.trueMin;
  dfs.gradD = ssmr.selected;
  dfs.messages = messages;

  return dfs;
}

void computeSetPoint()
{
  bool found = false;
  VectorXd qt = VectorXd::Zero(7);

  double bestBetaD = 100000;
  double bestTrueD = -100000;
  do
  {
    qt = Utils::randVec(7, -3, 3);
    // qt << -0.05480,  0.48761, -0.17090, -1.17250, -1.59130, -1.20790,  1.07573;

    double betaD = betafun(computeD(qt).D);
    double trueD = computeD(qt).trueD;
    double x = g_Kuka.fk(qt).htmTool(0, 3);
    double z = g_Kuka.fk(qt).htmTool(2, 3);
    found = (x > 0.5) && (z > 0.5) && (betaD < 0) && (trueD > 0.01);

    if ((x > 0.5) && (z > 0.5))
    {
      bestBetaD = min(bestBetaD, betaD);
      bestTrueD = max(bestTrueD, trueD);
      ROS_INFO_STREAM("trueD: " << trueD << ", betaD: " << betaD);
    }

    // ROS_INFO_STREAM("bestBetaD = " << bestBetaD << ", bestTrueD = " << bestTrueD);
  } while (!found);

  PARAM_QKUKASETPOINT = qt;

  // g_qKukaSetpoint << 2.09326, -0.94630, -2.40020, -0.69690, 0.97054, 1.89346, -2.64390;
}

CBFCircControlResult computeCBFCircVelocity()
{
  vector<Vector3d> v;
  vector<VectorXd> gradD_i;
  vector<double> D_i;
  vector<string> messages;
  CBFCircControlResult cbfcr;

  DFunctionStruct dfs = computeD(g_qKuka);

  TaskResult tr = g_Kuka.taskFunction(PARAM_HTM_TG, g_qKuka);

  cbfcr.dfunst = dfs;
  cbfcr.tr = tr;

  if (dfs.gradD.norm() >= 0.03)
  {
    VectorXd N = dfs.gradD.normalized();
    double D = dfs.D;

    // Compute the vector T
    VectorXd T = PARAM_OMEGA * N;

    // Compute the parameters of the quadratic program

    MatrixXd jacTask = tr.jacTask.block<3, 7>(0, 0);
    VectorXd task = tr.task.block<3, 1>(0, 0);

    double tn = pow(jacTask.norm(), 2);

    // VectorXd qdottg = -0.1 * jacTask.transpose() * task / pow(tn, 0.4);
    VectorXd qdottg = -0.5 * (g_qKuka - PARAM_QKUKASETPOINT);

    VectorXd f = -2 * qdottg;
    MatrixXd H = 2 * MatrixXd::Identity(g_qKuka.rows(), g_qKuka.rows());

    double alpha = alphafun(D);
    double beta = betafun(D);

    MatrixXd A = Utils::matrixVertStack(N.transpose(), T.transpose());
    VectorXd b = Utils::vectorVertStack(alpha, beta);

    // Remove the 6th and 7th joint
    MatrixXd Ht = H.block<5, 5>(0, 0);
    VectorXd ft = f.block<5, 1>(0, 0);
    const int a = A.rows();
    MatrixXd At = A.block(0, 0, a, 5);
    VectorXd bt = b.block(0, 0, a, 1);

    H = Ht;
    f = ft;
    A = At;
    b = bt;

    // A = Utils::matrixVertStack(A,  MatrixXd::Identity(g_qKuka.rows(), g_qKuka.rows()));
    // A = Utils::matrixVertStack(A, -MatrixXd::Identity(g_qKuka.rows(), g_qKuka.rows()));

    // b = Utils::vectorVertStack(b, -PARAM_MAX_VEL * VectorXd::Ones(2 * g_qKuka.rows()));

    A = Utils::matrixVertStack(A, MatrixXd::Identity(5, 5));
    A = Utils::matrixVertStack(A, -MatrixXd::Identity(5, 5));

    b = Utils::vectorVertStack(b, -PARAM_MAX_VEL * VectorXd::Ones(2 * 5));

    //

    VectorXd qdot_temp = Utils::solveQP(H, f, A, b);
    VectorXd qdot = VectorXd::Zero(7);
    qdot << qdot_temp[0], qdot_temp[1], qdot_temp[2], qdot_temp[3], qdot_temp[4], 0, 0;

    ROS_INFO_STREAM("proj = " << (qdot.normalized().transpose() * qdottg.normalized())[0]);
    // ROS_INFO_STREAM("N: " << (N.transpose() * qdottg)[0] << " >= " << alpha);
    // ROS_INFO_STREAM("T: " << (T.transpose() * qdottg)[0] << " >= " << beta);

    // ROS_INFO_STREAM("N = "<<Utils::printVector(N));
    // ROS_INFO_STREAM("T = "<<Utils::printVector(T));
    // ROS_INFO_STREAM(Utils::printMatrix(A));
    // ROS_INFO_STREAM(Utils::printVector(b));
    // ROS_INFO_STREAM("alpha = "<<b[0]);
    // ROS_INFO_STREAM("beta = "<<b[1]);
    // ROS_INFO_STREAM("qdot = "<<Utils::printVector(qdot));

    if (qdot.rows() > 0)
    {
      cbfcr.qdot = qdot;
      cbfcr.zeroTangent = false;
      cbfcr.feasible = true;

      return cbfcr;
    }
    else
    {
      cbfcr.qdot = VectorXd::Zero(7);
      cbfcr.zeroTangent = false;
      cbfcr.feasible = false;

      return cbfcr;
    }
  }
  else
  {
    ROS_INFO_STREAM(Utils::printVector(dfs.gradD));
    ROS_INFO_STREAM("Dzt: " << dfs.trueD);

    cbfcr.qdot = VectorXd::Zero(7);
    cbfcr.zeroTangent = true;
    cbfcr.feasible = true;

    return cbfcr;
  }
}

void controlUpdate(const ros::TimerEvent& e)
{
  ROS_INFO_STREAM("A");
  CBFCircControlResult cbfcr = computeCBFCircVelocity();

  if (cbfcr.feasible && !cbfcr.zeroTangent)
  {
    ROS_INFO_STREAM("---- t = " << getTime() << " s --------");
    ROS_INFO_STREAM("D = " << Utils::printNumber(cbfcr.dfunst.D));
    ROS_INFO_STREAM("V = " << (g_qKuka - PARAM_QKUKASETPOINT).norm());
    // ROS_INFO_STREAM("task = " << Utils::printVector(cbfcr.tr.task));

    // Store
    g_histqKuka.push_back(g_qKuka);
    g_histD.push_back(cbfcr.dfunst.D);
    g_histTime.push_back(getTime());

    sendJointVelocity(cbfcr.qdot);
    // sendJointAcceleration(ccr.action);
  }
  else
  {
    if (!cbfcr.feasible)
    {
      ROS_INFO_STREAM("Unfeasible!");
      g_missionOver = true;
    }
    if (cbfcr.zeroTangent)
    {
      ROS_INFO_STREAM("Zero tangent!");
      g_missionOver = true;
    }
  }
}

void placeWallWithHole(Matrix4d htm)
{
  Box* o1 = new Box(htm * Utils::trn(0, -0.5, 0.7), 0.1, 0.6, 1.4);
  PARAM_OBSTACLES.push_back(o1);

  Box* o2 = new Box(htm * Utils::trn(0, 0.5, 0.7), 0.1, 0.6, 1.4);
  PARAM_OBSTACLES.push_back(o2);

  Box* o3 = new Box(htm * Utils::trn(0, 0, 0.25), 0.1, 0.4, 0.5);
  PARAM_OBSTACLES.push_back(o3);

  Box* o4 = new Box(htm * Utils::trn(0, 0, 1.15), 0.1, 0.4, 0.5);
  PARAM_OBSTACLES.push_back(o4);
}

static string getFileName()
{
  time_t t = time(NULL);
  tm* timePtr = localtime(&t);

  string str = "";

  if (PARAM_ISSIMULATION)
    str = "sim_cbf_circ_iiwa_";
  else
    str = "exp_cbf_circ_iiwa_";

  str += std::to_string(timePtr->tm_mday) + "_" + std::to_string(timePtr->tm_mon + 1) + "_ts_" +
         std::to_string(timePtr->tm_hour) + "_" + std::to_string(timePtr->tm_min);

  str += ".py";

  return str;
}

void initializeUAIBotFile()
{
  string fileName = getFileName();

  g_file.open("/home/cair1/PycharmProjects/uaibot_vinicius/test/cbfvf/data/" + fileName, ofstream::trunc);

  g_file_gen.open("/home/cair1/PycharmProjects/uaibot_vinicius/test/cbfvf/read_cbf_circ.py");

  g_file_gen << "file1 = open('data/" << fileName << "', 'r')" << std::endl;
  g_file_gen << "Lines = file1.readlines()" << std::endl;
  g_file_gen << "for line in Lines:" << std::endl;
  g_file_gen << "    if line[0]!='#':" << std::endl;
  g_file_gen << "        exec(line.strip())" << std::endl;

  if (PARAM_ISSIMULATION)
  {
    g_file << "#SIMULATION" << std::endl << std::endl;
  }
  else
  {
    g_file << "#REAL EXPERIMENT" << std::endl << std::endl;
  }

  g_file << "#Omega used:" << std::endl;
  g_file << "#" << Utils::printMatrixPython(PARAM_OMEGA) << std::endl;

  g_file << "import uaibot as ub" << std::endl;
  g_file << "import numpy as np" << std::endl << std::endl;

  g_file << "sim = ub.Simulation.create_sim_factory([])" << std::endl;
  g_file << "sim.set_parameters(width=1600, height=950)" << std::endl << std::endl;

  g_file << "iiwa = ub.Robot.create_kuka_lbr_iiwa()" << std::endl;
  g_file << "sim.add(iiwa)" << std::endl;

  g_file << "sim.add(ub.Frame(htm=" << Utils::printMatrixPython(PARAM_HTM_TG) << "))" << std::endl;

  g_file << "obstacles=[]" << std::endl << std::endl;

  for (int k = 0; k < PARAM_OBSTACLES.size(); k++)
  {
    GeometricPrimitives* gp = PARAM_OBSTACLES[k];

    if (typeid(*(PARAM_OBSTACLES[k])) == typeid(Box))
    {
      Box obj = *dynamic_cast<Box*>(gp);
      g_file << "obstacles.append(ub.Box(width=" << std::to_string(obj.width) << ", depth=" + std::to_string(obj.depth)
             << ", height=" + std::to_string(obj.height) << ", color='black',opacity=0.5))" << std::endl;
    }

    if (typeid(*(PARAM_OBSTACLES[k])) == typeid(Cylinder))
    {
      Cylinder obj = *dynamic_cast<Cylinder*>(gp);
      g_file << "obstacles.append(ub.Cylinder(radius=" << std::to_string(obj.radius)
             << ", height=" + std::to_string(obj.height) << ", color='black',opacity=0.5))" << std::endl;
    }

    if (typeid(*(PARAM_OBSTACLES[k])) == typeid(Sphere))
    {
      Sphere obj = *dynamic_cast<Sphere*>(gp);
      g_file << "obstacles.append(ub.Ball(radius=" << std::to_string(obj.radius) << ", color='black',opacity=0.5))"
             << std::endl;
    }

    g_file << "sim.add(obstacles[-1])" << std::endl;
    g_file << "obstacles[-1].add_ani_frame(0," << Utils::printMatrixPython(gp->htm) << ")" << std::endl;
  }

  g_file << std::endl << std::endl;
}

void finalizeUAIBotFile()
{
  g_file << "#Hist" << std::endl;
  g_file << "#t = [";
  for (int n = 0; n < g_histTime.size(); n++) g_file << g_histTime[n] << ", ";

  g_file << "]" << std::endl;

  g_file << "#D = [";
  for (int n = 0; n < g_histD.size(); n++) g_file << g_histD[n] << ", ";

  g_file << "]" << std::endl;

  g_file << "#q = [";
  for (int n = 0; n < g_histqKuka.size(); n++) g_file << Utils::printVector(g_histqKuka[n]) << ", ";

  g_file << "]" << std::endl;

  g_file << "#Save" << std::endl;
  g_file << "sim.save('//home//cair1//Desktop//uaibottest','replayexpcbfiiwa')" << std::endl;
}

int main(int argc, char** argv)
{
  int input = -1;
  while ((input != 0 && input != 1) || cin.fail())
  {
    ROS_INFO_STREAM("Choose simulation (0) or real experiment (1)");
    cin >> input;
  }
  PARAM_ISSIMULATION = input == 0;

  if (PARAM_ISSIMULATION)
    ROS_INFO_STREAM("Starting simulation...");
  else
    ROS_INFO_STREAM("Starting control loop...");

  ros::init(argc, argv, "manager");
  ros::NodeHandle nodeHandler;

  ros::Subscriber kukaSubscriber = nodeHandler.subscribe("/iiwa/state/JointPosition", 100, kukaCallJoints);
  g_KukaJointsPublisher = nodeHandler.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 2);

  // Create omega matrix
  // g_qKuka[5] = 0.1;
  generateOmega();

  // g_qKuka << 0, g_Kuka.qMax[1], 0, 0, 0, 0, 0;
  // g_qKukaSetpoint = VectorXd::Zero(7);
  // g_qKukaSetpoint << 0, 0, 0, 0, 0, 0, 0;

  g_qKuka << 0, 0., 0, 0, 0, 0, 0;
  PARAM_QKUKASETPOINT = VectorXd::Zero(7);
  PARAM_QKUKASETPOINT << 0, 0.90 * g_Kuka.qMax[1], 0, 0, 0, 0, 0;

  // Create obstacle and environment
  // placeWallWithHole(Utils::trn(0.3, 0, 0));
  Cylinder* c1 = new Cylinder(Utils::trn(0.5, 0, 0.6) * Utils::rotx(3.14 / 2), 0.15, 0.8);
  PARAM_OBSTACLES.push_back(c1);

  Box* b1 = new Box(Utils::trn(-0.4, 0, 0.17), 1.0, 1.0, 0.03);
  // PARAM_OBSTACLES.push_back(b1);
  g_Platform = b1;

  Sphere* e1 = new Sphere(Utils::trn(0.5, -0.5, 0.6), 0.3);
  PARAM_OBSTACLES.push_back(e1);
  // PARAM_OBSTACLES.push_back(b2);
  // computeSetPoint();

  // g_qKukaSetpoint = VectorXd::Zero(7);
  // g_qKukaSetpoint << 2.09326, -0.94630, -2.40020, -0.69690, 0.97054, 1.89346, -2.64390;

  // Initialize file
  initializeUAIBotFile();

  ///
  // g_file << "iiwa.add_ani_frame(time = 0,q=" << Utils::printVector(g_qKukaSetpoint) << ")" << std::endl;
  g_file << "colobj=[]" << std::endl;

  // ROS_INFO_STREAM("A2: " << g_Kuka.computeDistToObj(PARAM_OBSTACLES[0], g_qKukaSetpoint).getClosest().distance);
  // ROS_INFO_STREAM("****************************");
  // ROS_INFO_STREAM("Test:");

  // ROS_INFO_STREAM("A2: " << g_Kuka.computeDistToObj(PARAM_OBSTACLES[0], g_qKukaSetpoint).getClosest().distance);
  // ROS_INFO_STREAM("A2: " << g_Kuka.computeDistToObj(PARAM_OBSTACLES[0], g_qKukaSetpoint).getClosest().distance);
  // ROS_INFO_STREAM("B1: " << computeD(g_qKukaSetpoint).trueD);

  // cin>>input;

  for (int k = 0; k < 7; k++)
  {
    for (int j = 0; j < g_Kuka.links[k].colObjs.size(); j++)
    {
      GeometricPrimitives* gp = g_Kuka.links[k].colObjs[j];

      if (typeid(*(g_Kuka.links[k].colObjs[j])) == typeid(Box))
      {
        Box obj = *dynamic_cast<Box*>(gp);
        g_file << "colobj.append(ub.Box(width=" << std::to_string(obj.width) << ", depth=" + std::to_string(obj.depth)
               << ", height=" + std::to_string(obj.height) << ", color='" << PARAM_COLOR[k] << "',opacity=0.5))"
               << std::endl;
      }

      if (typeid(*(g_Kuka.links[k].colObjs[j])) == typeid(Cylinder))
      {
        Cylinder obj = *dynamic_cast<Cylinder*>(gp);
        g_file << "colobj.append(ub.Cylinder(radius=" << std::to_string(obj.radius)
               << ", height=" + std::to_string(obj.height) << ", color='" << PARAM_COLOR[k] << "',opacity=0.5))"
               << std::endl;
      }

      if (typeid(*(g_Kuka.links[k].colObjs[j])) == typeid(Sphere))
      {
        Sphere obj = *dynamic_cast<Sphere*>(gp);
        g_file << "colobj.append(ub.Ball(radius=" << std::to_string(obj.radius) << ", color='" << PARAM_COLOR[k]
               << "', opacity=0.5))" << std::endl;
      }

      g_file << "sim.add(colobj[-1])" << std::endl;
      g_file << "colobj[-1].add_ani_frame(0," << Utils::printMatrixPython(gp->htm) << ")" << std::endl;
    }
  }

  // Teste

  // VectorXd qt = VectorXd::Zero(7);
  // qt << -1.97400, 0.62988, 2.06829, -2.12900, -0.36560, -0.04510, 1.58723;

  // g_Kuka.setConfig(qt);

  // for (int g = 0; g < 20; g++)
  // {
  //     ROS_INFO_STREAM("-----");

  //     GeometricPrimitives *ob1 = g_Kuka.links[4].colObjs[1];
  //     GeometricPrimitives *ob2 = PARAM_OBSTACLES[0];

  //     Vector3d p0 = ob1->htm.block<3,1>(0,3);

  //     Vector3d p1 = ob1->projection(p0);
  //     Vector3d p2 = ob2->projection(p1);
  //     Vector3d p3 = ob1->projection(p2);

  //     ROS_INFO_STREAM("p1 = "<<Utils::printVector(p1));
  //     ROS_INFO_STREAM("p2 = "<<Utils::printVector(p2));
  //     ROS_INFO_STREAM("p3 = "<<Utils::printVector(p3));

  //     //double dd = GeometricPrimitives::computeDist(, , p0, 0, 0.0005).distance;

  //     //ROS_INFO_STREAM("dd = " << dd);
  // }

  // g_file << "iiwa.add_ani_frame(time = 0,q=" << Utils::printVector(qt) << ")" << std::endl;
  // cin >> input;

  // ROS_INFO_STREAM("****************************");

  // Initialize code

  g_startTime = ros::Time::now().toSec();

  ros::Timer controller;
  if (PARAM_ISSIMULATION)
    controller = nodeHandler.createTimer(ros::Duration(PARAM_DT / PARAM_SIMSPEEDUP), controlUpdate);
  else
    controller = nodeHandler.createTimer(ros::Duration(PARAM_DT), controlUpdate);

  ros::Rate rate(1 / PARAM_DT);

  while (ros::ok() && !g_missionOver)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // Final writing
  finalizeUAIBotFile();

  return 0;
}
