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

bool run = true;

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
TimeSeries g_fzTimeSeries;

TwoTimeSeries g_targetq;

vector<int> g_startIndex = {1};
vector<VectorXd> g_allpds;
float previous_error = 0;

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

Matrix4d PARAM_HTMSTART = Utils::trn(0.45, 0, 0.55 - 0.4) * Utils::roty(3.14);

double PARAM_DT = 1.0 / 250;
Vector3d PARAM_FP;
bool PARAM_ISSIM = false;
double PARAM_TIME = 0.01;
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

bool setConfig(VectorXd q)
{
  if (g_manip.fk(q).htmTool(2, 3) < 0.45 - 0.5 - 1000)
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

VectorXd sqrtsgn(VectorXd x)
{
  VectorXd y = VectorXd::Zero(x.rows());

  for (int i = 0; i < x.rows(); i++) y[i] = sqrtsgn(x[i]);

  return y;
}

VectorXd fulcrumPointControl(VectorXd qt, Vector3d pd, Vector3d pf)
{
  double K = 14.0;

  VectorXd q = qt;
  double dt = PARAM_DT;
  VectorXd qdot;

  for (int n = 0; n < round(PARAM_DT / dt); n++)
  {
    FulcrumPointResult fpResult = g_manip.computeFulcrumPoint(pf, q);

    MatrixXd A1 = Utils::matrixVertStack(fpResult.jacfx, fpResult.jacfy);
    VectorXd b1 = Utils::vectorVertStack(-K * (fpResult.fx), -K * (fpResult.fy));

    MatrixXd A2 = fpResult.fkr.jacTool.block<3, 7>(0, 0);
    Vector3d pe = fpResult.fkr.htmTool.block<3, 1>(0, 3);
    VectorXd b2 = -(27.0) * (pe - pd);

    vector<MatrixXd> A = {A2, A1};
    vector<VectorXd> b = {b2, b1};

    qdot = Utils::hierarchicalSolve(A, b, 0.0000001);
    q += qdot * dt;
  }

  qdot = (q - qt) / PARAM_DT;

  FulcrumPointResult fpResult_next = g_manip.computeFulcrumPoint(pf, q);

  return qdot;
}

void touchCallEEFPosition(const geometry_msgs::PoseStamped msg)
{
  double currentTime = getTime();
  double timeDifference = currentTime - lastTimeEEfPosition;
  lastTimeEEfPosition = currentTime;

  VectorXd currentPosition(3);
  currentPosition << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  Data currentPositionData;
  currentPositionData.data = currentPosition;
  currentPositionData.timeStamp = currentTime;
  touchEEfPosition.push_back(currentPositionData);

  if (touchEEfPosition.size() > 1)
  {
    VectorXd unfiltVelocity =
        (currentPosition - touchEEfPosition.at(touchEEfPosition.size() - 2).data) / timeDifference;
    Data currentVelocitiesData;
    currentVelocitiesData.data =
        FILTER_PARAM * unfiltVelocity + (1 - FILTER_PARAM) * touchEEfVelocitites[touchEEfVelocitites.size() - 1].data;

    currentVelocitiesData.timeStamp = currentTime;
    touchEEfVelocitites.push_back(currentVelocitiesData);
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

  if (touchJoints.size() > 1)
  {
    VectorXd unfiltVelocity = (currentPosition - touchJoints.at(touchJoints.size() - 2).data) / timeDifference;
    Data currentVelocitiesData;
    currentVelocitiesData.data = FILTER_PARAM * unfiltVelocity +
                                 (1 - FILTER_PARAM) * touchJointsVelocities[touchJointsVelocities.size() - 1].data;
    currentVelocitiesData.timeStamp = currentTime;
    touchJointsVelocities.push_back(currentVelocitiesData);
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

  g_fileDebug << "fz = [];" << std::endl;
  for (int i = 0; i < g_fzTimeSeries.size(); i++)
    g_fileDebug << "fz = [fz; " << g_fzTimeSeries.print(i) << "];" << std::endl;

  g_fileDebug << "index = [];" << std::endl;
  for (int i = 0; i < g_startIndex.size(); i++)
    g_fileDebug << "index = [index; " << g_startIndex[i] << "];" << std::endl;

  g_fileDebug << "allpd = [];" << std::endl;
  for (int i = 0; i < g_allpds.size(); i++)
    g_fileDebug << "allpd = [allpd; " << Utils::printVectorOctave(g_allpds[i]) << "];" << std::endl;

  g_fileDebug << "figure; plot3(-p(:,3),p(:,2),p(:,4),'bo'); hold on; plot3(-pd(:,3),pd(:,2),pd(:,4),'r'); hold off"
              << std::endl;
  g_fileDebug << "figure; plot(p(:,1),p(:,2),'b'); hold on; plot(p(:,1),pd(:,2),'r'); hold off;" << std::endl;
  g_fileDebug << "figure; plot(p(:,1),p(:,3),'b'); hold on; plot(p(:,1),pd(:,3),'r'); hold off;" << std::endl;
  g_fileDebug << "figure; plot(p(:,1),p(:,4),'b'); hold on; plot(p(:,1),pd(:,4),'r'); hold off;" << std::endl;
  g_fileDebug << "figure; plot(fpe(:,1),1000*fpe(:,2)); hold on; plot(fpe(:,1),0*fpe(:,1) + 4,'r--'); hold off;"
              << std::endl;
  g_fileDebug << "figure; plot(p(:,1),sqrt(sum((p(:,2:4)-pd(:,2:4)).^2'))*1000);" << std::endl;
}

TwoTimeSeries generatePath(VectorXd p0, double t0, double maxtime)
{
  VectorXd q = getConfig();
  double t = t0;
  Vector3d vlin_des;

  TimeSeries path;
  TimeSeries desp;

  double dt = 3 * PARAM_DT;
  while (t - t0 < maxtime + 0.1)
  {
    vlin_des << 0, 0.02 * cos(2 * 3.14 * t / 10), 0;

    Vector3d deltap = Vector3d::Zero(3);
    double rho = min(t / 5, 1);

    deltap << 0.03 * rho * cos(2 * 3.14 * t / 20), 0.03 * sin(2 * 3.14 * t / 20),
        -0.04 * rho + 0.06 * sin(2 * 3.14 * t / 40);

    VectorXd qdot = fulcrumPointControl(q, p0 + deltap, PARAM_FP);
    desp.add(p0 + deltap, t);

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
  ros::init(argc, argv, "fulcrumpoint_clean");
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

  TimeSeries qpath;
  double tend, tstart;
  VectorXd qr;

  Vector3d p0;

  VectorXd PARAM_STARTQ0 = VectorXd::Zero(7);

  PARAM_STARTQ0 << 0.61916, 1.42963, -1.60960, -1.60650, 1.43321, 1.59124, -1.25610;

  Matrix4d htmstart = g_manip.fk(PARAM_STARTQ0).htmTool;
  PARAM_FP = htmstart.block<3, 1>(0, 3) - 0.1 * htmstart.block<3, 1>(0, 2);

  g_pd = htmstart.block<3, 1>(0, 3);

  while (ros::ok() && run)
  {
    // Going to Starting Position
    // Starting Joint Positions: PARAM_STARTQ0
    if (g_readedJoints && !g_reachedStartingPosition)
    {
      setConfigSpeed(-1.0 * sqrtsgn(getConfig() - PARAM_STARTQ0));

      g_reachedStartingPosition = (getConfig() - PARAM_STARTQ0).norm() <= 0.005;

      if (g_reachedStartingPosition)
      {
        g_startingTime = ros::Time::now();
        g_count = 0;
        p0 = g_manip.fk(getConfig()).htmTool.block<3, 1>(0, 3);
      }

      if (g_count % 50)
      {
        ROS_INFO_STREAM("--Going to starting position--");
        ROS_INFO_STREAM("t = " << getTime());

        ROS_INFO_STREAM("e = " << Utils::printVector(getConfig() - PARAM_STARTQ0));
      }
    }

    if (g_readedJoints && g_reachedStartingPosition)
    {
      if (!(g_count % ((int)(PARAM_TIME / PARAM_DT))))
      {
        Vector3d p_real = g_manip.fk(getConfig()).htmTool.block<3, 1>(0, 3);

        double tstart = getTime();
        g_targetq = generatePath(p0, getTime(), PARAM_TIME);

        g_startIndex.push_back(g_startIndex[g_startIndex.size() - 1] + g_targetq.b.size());
        for (int s = 0; s < g_targetq.b.size(); s++) g_allpds.push_back(g_targetq.b.data[s]);
      }
      else
      {
        setConfig(g_targetq.a.atTime(getTime()));

        FulcrumPointResult fpr = g_manip.computeFulcrumPoint(PARAM_FP, getConfig());
        Vector3d pe = fpr.fkr.htmTool.block<3, 1>(0, 3);
        g_pTimeSeries.add(pe, getTime());
        g_pdTimeSeries.add(g_targetq.b.atTime(getTime()), getTime());
        g_fpeTimeSeries.add(fpr.df, getTime());
        g_fzTimeSeries.add(fpr.fz, getTime());

        if (g_count % 100)
        {
          ROS_INFO_STREAM("--Fulcrum point movement--");
          ROS_INFO_STREAM("t = " << getTime());
          ROS_INFO_STREAM("errorfp = " << round(10000 * fpr.df) / 10 << " (mm)");
        }
        previous_error = fpr.df * 1000;
      }
    }

    g_count++;
    ros::spinOnce();
    loop_rate.sleep();

    if (PARAM_ISSIM) g_tsim += PARAM_DT;
  }

  printToFile();
}