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
#include "std_msgs/Float32MultiArray.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>
#include <conio.h>
#include "robot.cpp"
#include "utils.cpp"
#include "distance.cpp"
#include "OmniButtonEvent.h"

#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointPositionVelocity.h"
#include "iiwa_msgs/JointQuantity.h"

using namespace std;

struct Data
{
  VectorXd data;
  double timeStamp;
};

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

struct TwoTimeSeries
{
  TimeSeries a;
  TimeSeries b;
};

// Global variables
VectorXd g_qkuka = VectorXd::Zero(7);
bool g_readedJoints = false;
ros::Time g_startingTime;
int g_count = 0;
Vector3d g_pd;
Manipulator g_manip = Manipulator::createKukaIIWA();
bool g_reachedStartingPosition = false;
ofstream g_fileDebug;
TimeSeries g_qTimeSeries;
TimeSeries g_qdTimeSeries;
TimeSeries g_taskTimeSeries;
TimeSeries g_pTimeSeries;
TimeSeries g_pdTimeSeries;
TimeSeries g_fpeTimeSeries;
// TimeSeries g_targetq;
TwoTimeSeries g_targetq;

vector<int> g_startIndex = {1};
vector<VectorXd> g_allpds;

double g_tsim = 0;

ros::Publisher geoTouch;
double lastTimeEEfPosition;
double lastTimeJointsPosition;
ros::Publisher KUKAJointsPublisher;
ros::Publisher KUKAJointsVelocityPublisher;
ros::Publisher KUKAFRIPublisher;
bool act = true;
vector<Data> touchJoints;
vector<Data> touchJointsVelocities;
vector<Data> touchEEfPosition;
vector<Data> touchEEfVelocitites;

// Parameters
Matrix4d PARAM_HTMSTART = Utils::trn(0.45, 0, 0.55 - 0.4) * Utils::roty(3.14);
double PARAM_DT = 1.0 / 100;
Vector3d PARAM_FP;
bool PARAM_ISSIM = false;
double PARAM_TIME = 0.25;
double FILTER_PARAM = 0.01;
double VELCONVERT = 1;

double getTime()
{
  if (!PARAM_ISSIM)
    return (ros::Time::now() - g_startingTime).toSec();
  else
    return g_tsim;
}

VectorXd getConfig()
{
  return g_qkuka;
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

bool setConfig(VectorXd q)
{
  if (g_manip.fk(q).htmTool(2, 3) < 0.45 - 0.4)  //! safetyCheck(q)
  {
    ROS_INFO("ERROR: Collision of the tool with the ground. Not sending");
    return false;
  }
  else
  {
    bool jointsWithinLimits = true;
    for (int i = 0; i < q.rows(); i++)
      jointsWithinLimits = jointsWithinLimits && (q[i] >= g_manip.qMin[i] && q[i] <= g_manip.qMax[i]);

    if (jointsWithinLimits)
    {
      if (!PARAM_ISSIM)
      {
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
      }
      else
      {
        g_qkuka = q;
      }

      if (g_reachedStartingPosition)
      {
        g_qTimeSeries.add(getConfig(), getTime());
        g_qdTimeSeries.add(q, getTime() + PARAM_DT);
      }

      return true;
    }
    else
    {
      ROS_INFO("ERROR: Joints not within limits. Not sending...");
      return false;
    }
  }
}

bool setConfigSpeed(VectorXd qdot)
{
  bool speedWithinLimits = true;
  for (int i = 0; i < qdot.rows(); i++)
    speedWithinLimits =
        speedWithinLimits && (qdot[i] >= 1.5 * g_manip.qDotMin[i] && qdot[i] <= 1.5 * g_manip.qDotMax[i]);

  if (speedWithinLimits)
    setConfig(getConfig() + PARAM_DT * qdot);
  else
    ROS_INFO_STREAM("ERROR: Speet outside limits... not sending");
}

double sqrtsgn(double x)
{
  return ((x > 0) ? 1 : -1) * sqrt(abs(x));
}

VectorXd fulcrumPointControl(VectorXd q, Vector3d pd, Vector3d pf)
{
  // Algorithm Parameters
  double K = 0.2;

  FulcrumPointResult fpResult = g_manip.computeFulcrumPoint(pf, q);

  MatrixXd A1 = Utils::matrixVertStack(fpResult.jacfx, fpResult.jacfy);
  VectorXd b1 = Utils::vectorVertStack(-K * sqrtsgn(fpResult.fx), -K * sqrtsgn(fpResult.fy));

  MatrixXd A2 = fpResult.fkr.jacTool.block<3, 7>(0, 0);
  Vector3d pe = fpResult.fkr.htmTool.block<3, 1>(0, 3);
  VectorXd b2 = -8.0 * (pe - pd);

  vector<MatrixXd> A = {A1, A2};
  vector<VectorXd> b = {b1, b2};

  VectorXd qdot = Utils::hierarchicalSolve(A, b, 0.0001);

  FulcrumPointResult fpResult_next = g_manip.computeFulcrumPoint(pf, q + PARAM_DT * qdot);

  return qdot;
}

VectorXd fulcrumPointControl2(VectorXd q, Vector3d pd, Vector3d pf)
{
  // Algorithm Parameters
  double K = 0.1;

  VectorXd qr = q;

  int N = 200;
  double dti = PARAM_DT / ((double)N);

  vector<double> err;
  FulcrumPointResult fpResult;

  for (int i = 0; i < N; i++)
  {
    fpResult = g_manip.computeFulcrumPoint(pf, qr);

    err.push_back(round(10000 * fpResult.df) / 10);

    MatrixXd A1 = Utils::matrixVertStack(fpResult.jacfx, fpResult.jacfy);
    VectorXd b1 = Utils::vectorVertStack(-K * sqrtsgn(fpResult.fx), -K * sqrtsgn(fpResult.fy));

    MatrixXd A2 = fpResult.fkr.jacTool.block<3, 7>(0, 0);
    Vector3d pe = fpResult.fkr.htmTool.block<3, 1>(0, 3);
    VectorXd b2 = -2.0 * (pe - pd);

    vector<MatrixXd> A = {A1, A2};
    vector<VectorXd> b = {b1, b2};

    VectorXd qdot = Utils::hierarchicalSolve(A, b, 0.0001);
    qr += dti * qdot;
  }

  if (g_count % 50)
  {
    ROS_INFO_STREAM("e[0] = " << err[0]);
    ROS_INFO_STREAM("e[N-1] = " << err[err.size() - 1]);
  }

  return (qr - q) / PARAM_DT;
}

void touchCallEEFPosition(const geometry_msgs::PoseStamped msg)
{
  double currentTime = getTime();
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
  double currentTime = getTime();
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
  if (!PARAM_ISSIM)
  {
    VectorXd q_kuka_temp = VectorXd::Zero(7);
    q_kuka_temp << msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5],
        msg.position[6];
    g_qkuka = q_kuka_temp;
  }
  g_readedJoints = true;
}

void touchCallButton(const omni_msgs::OmniButtonEvent msg)
{
  act = msg.grey_button == 0;
}

void printToFile()
{
  g_fileDebug.open("/home/cair1/Desktop/octavelogs/kukaFPlogs.m");

  if (PARAM_ISSIM)
    g_fileDebug << "\%Simulation" << std::endl;
  else
    g_fileDebug << "\%Real data" << std::endl;

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

  g_fileDebug << "index = [];" << std::endl;
  for (int i = 0; i < g_startIndex.size(); i++)
    g_fileDebug << "index = [index; " << g_startIndex[i] << "];" << std::endl;

  g_fileDebug << "allpd = [];" << std::endl;
  for (int i = 0; i < g_allpds.size(); i++)
    g_fileDebug << "allpd = [allpd; " << Utils::printVectorOctave(g_allpds[i]) << "];" << std::endl;
}

// TimeSeries generatePath()
// {
//   VectorXd q = getConfig();
//   double t0 = getTime();
//   double t = t0;
//   VectorXd pd = g_pd;
//   Vector3d vlin_des;

//   TimeSeries path;
//   double dt = 4 * PARAM_DT;
//   while (t - t0 < 40)
//   {
//     vlin_des << 0, 0.015 * sin(2 * 3.14 * (t - t0) / 6), 0;
//     pd = pd + PARAM_DT * vlin_des;
//     VectorXd qdot = fulcrumPointControl(q, pd, PARAM_FP);

//     q += PARAM_DT * qdot;
//     t += PARAM_DT;

//     path.add(q, t);
//   }
//   return path;
// }

TwoTimeSeries generatePath(VectorXd pd0, double t0, double maxtime)
{
  VectorXd q = getConfig();
  double t = t0;
  VectorXd pd = pd0;
  Vector3d vlin_des;

  TimeSeries path;
  TimeSeries desp;

  double dt = 3 * PARAM_DT;
  while (t - t0 < maxtime + 0.1)
  {
    desp.add(pd, t);
    vlin_des << 0, 0.012 * sin(2 * 3.14 * t / 20), 0;
    pd = pd + dt * vlin_des;
    VectorXd qdot = fulcrumPointControl(q, pd, PARAM_FP);

    q += dt * qdot;
    t += dt;

    path.add(q, t);
  }
  TwoTimeSeries tts;
  tts.a = path;
  tts.b = desp;

  return tts;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "fulcrumpoint");
  ros::NodeHandle n;

  ros::Subscriber KUKASubscriber1 = n.subscribe("/iiwa/joint_states", 100, kukaCallJoints);
  ros::Subscriber touchSubscriber1 = n.subscribe("/phantom/joint_states", 100, touchCallJoints);
  ros::Subscriber touchSubscriber2 = n.subscribe("/phantom/pose", 100, touchCallEEFPosition);
  ros::Subscriber touchSubscriber23 = n.subscribe("/phantom/button", 100, touchCallButton);

  KUKAFRIPublisher = n.advertise<std_msgs::Float64MultiArray>("/iiwa/PositionController/command", 3);

  ros::Rate loop_rate(1 / PARAM_DT);

  VelocityConstControlParam param;
  param.taskHtm = PARAM_HTMSTART;
  param.obstacles = {};
  param.considerActionLimits = false;
  param.considerAutoCollision = false;
  param.considerJointLimits = true;
  param.kpos = 2.0;
  param.kori = 0.4;

  PARAM_FP = PARAM_HTMSTART.block<3, 1>(0, 3);
  PARAM_FP[2] += 0.10;
  g_pd = PARAM_HTMSTART.block<3, 1>(0, 3);

  double t_reached;

  if (PARAM_ISSIM) g_readedJoints = true;

  //
  TimeSeries qpath;
  double tend, tstart;
  VectorXd qr;
  //

  while (ros::ok())
  {
    // Mode: going to starting position
    if (g_readedJoints && !g_reachedStartingPosition)
    {
      ConstControlResult ccr = g_manip.velocityConstControl(getConfig(), param);
      setConfigSpeed(ccr.action);
      g_reachedStartingPosition = ccr.taskResult.task.norm() <= 0.01;

      if (g_reachedStartingPosition)
      {
        g_startingTime = ros::Time::now();
        g_count = 0;
      }

      // Display
      if (g_count % 50)
      {
        ROS_INFO_STREAM("--Going to starting position--");
        // ROS_INFO_STREAM("t = " << getTime());
        ROS_INFO_STREAM("Task = " << Utils::printVector(ccr.taskResult.task));
      }
    }

    // Mode: fulcrum point constraint
    if (g_readedJoints && g_reachedStartingPosition)
    {
      // Vector3d vlin_des;
      // vlin_des << 0, 0.03 * sin(2 * 3.14 * (getTime() - t_reached) / 6), 0;

      // g_pd = g_pd + PARAM_DT * vlin_des;

      // VectorXd qdot = fulcrumPointControl2(getConfig(), g_pd, PARAM_FP);
      // setConfigSpeed(qdot);

      if (!(g_count % ((int)(PARAM_TIME / PARAM_DT))))
      {
        Vector3d p_real = g_manip.fk(getConfig()).htmTool.block<3, 1>(0, 3);
        // Vector3d pd_last = g_targetq.a.data[g_targetq.a.size() - 1];
        double tstart = getTime();
        g_targetq = generatePath(p_real, getTime(), PARAM_TIME);

        //
        g_startIndex.push_back(g_startIndex[g_startIndex.size() - 1] + g_targetq.b.size());
        for (int s = 0; s < g_targetq.b.size(); s++) g_allpds.push_back(g_targetq.b.data[s]);
      }
      else
      {
        setConfig(g_targetq.a.atTime(getTime()));

        // Compute some things to store
        FulcrumPointResult fpr = g_manip.computeFulcrumPoint(PARAM_FP, getConfig());
        Vector3d pe = fpr.fkr.htmTool.block<3, 1>(0, 3);
        g_pTimeSeries.add(pe, getTime());
        g_pdTimeSeries.add(g_targetq.b.atTime(getTime()), getTime());
        g_fpeTimeSeries.add(fpr.df, getTime());

        // Display
        if (g_count % 100)
        {
          ROS_INFO_STREAM("--Fulcrum point movement--");
          ROS_INFO_STREAM("t = " << getTime());
          ROS_INFO_STREAM("errorfp = " << round(10000 * fpr.df) / 10 << " (mm)");
        }
      }
    }

    g_count++;
    ros::spinOnce();
    loop_rate.sleep();

    if (PARAM_ISSIM) g_tsim += PARAM_DT;
  }

  printToFile();
}