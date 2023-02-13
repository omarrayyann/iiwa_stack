#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>
#include <fstream>
#include <ros/ros.h>
#include <ros/service.h>
#include <chrono>

// #include <trajectory_msgs/JointTrajectory.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>

#include "libraries/utils.cpp"
#include "libraries/distance.cpp"
#include "libraries/robot.cpp"

// #include "geometry_msgs/Twist.h"
// #include "geometry_msgs/Pose.h"

// #include "gazebo_msgs/LinkStates.h"
// #include "gazebo_msgs/ModelState.h"
// #include "gazebo_msgs/SetModelState.h"

// #include "iiwa_ros/command/joint_velocity.hpp"
// #include "iiwa_ros/state/joint_position.hpp"
#include "iiwa_msgs/JointVelocity.h"
#include "iiwa_msgs/JointPosition.h"

using std::fstream;

// Main

vector<VectorXd> histq;
vector<VectorXd> histdotq;

void callbackdotq(const iiwa_msgs::JointVelocity::ConstPtr& msg)
{
  iiwa_msgs::JointVelocity qROS = *msg;
  VectorXd qdot(7);

  // Capture the joint position
  qdot[0] = qROS.velocity.a1;
  qdot[1] = qROS.velocity.a2;
  qdot[2] = qROS.velocity.a3;
  qdot[3] = qROS.velocity.a4;
  qdot[4] = qROS.velocity.a5;
  qdot[5] = qROS.velocity.a6;
  qdot[6] = qROS.velocity.a7;

  histdotq.push_back(qdot);
}

void callbackq(const iiwa_msgs::JointPosition::ConstPtr& msg)
{
  iiwa_msgs::JointPosition qROS = *msg;

  // Capture the joint position
  VectorXd q(7);
  q[0] = qROS.position.a1;
  q[1] = qROS.position.a2;
  q[2] = qROS.position.a3;
  q[3] = qROS.position.a4;
  q[4] = qROS.position.a5;
  q[5] = qROS.position.a6;
  q[6] = qROS.position.a7;

  histq.push_back(q);
}

string printVector(VectorXd v)
{
  string str = "[";

  for (int i = 0; i < v.rows(); i++)
  {
    str += std::to_string(v[i]) + ((i < v.rows() - 1) ? ", " : "]");
  }

  return str;
}

string printVectorOctave(VectorXd v)
{
  string str = "[";

  for (int i = 0; i < v.rows(); i++)
  {
    str += std::to_string(v[i]) + ((i < v.rows() - 1) ? " " : "]");
  }

  return str;
}

int main(int argc, char** argv)
{
  // Initialize ROS variables

  ros::Publisher velPub;
  ros::Publisher jointPub;

  ros::init(argc, argv, "iiwa_control_vinicius");
  ros::NodeHandle n;
  iiwa_msgs::JointVelocity qdotROS;
  iiwa_msgs::JointPosition qROS;
  ros::Time begin;

  jointPub = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 100);
  velPub = n.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 100);

  ros::Subscriber qSub = n.subscribe("/iiwa/state/JointPosition", 100, callbackq);
  ros::Subscriber qdotSub = n.subscribe("/iiwa/state/JointVelocity", 100, callbackdotq);

  // Create the environment variables
  Matrix4d htmTg;
  Manipulator* robot;
  Box *wall1, *wall2, *wall3, *wall4;
  vector<GeometricPrimitives*> obstacles;

  Manipulator iiwa = Manipulator::createKukaIIWA();
  robot = &iiwa;

  wall1 = new Box(Utils::trn(0.3, -0.5, 0.7), 0.05, 0.6, 1.4);
  wall2 = new Box(Utils::trn(0.3, 0.5, 0.7), 0.05, 0.6, 1.4);
  wall3 = new Box(Utils::trn(0.3, 0, 0.25), 0.05, 0.4, 0.5);
  wall4 = new Box(Utils::trn(0.3, 0, 1.15), 0.05, 0.4, 0.5);

  obstacles = {wall1, wall2, wall3, wall4};
  // obstacles = {};

  htmTg = Utils::trn(0.5, -0.3, 0.7) * Utils::rotx(M_PI / 2);  // y = -0.2

  // Initialize log file
  ofstream file;
  string filename;
  filename = "/home/cair1/Desktop/octavelogs/logkuka.txt";

  // filename = "/home/cair1/Desktop/octavelogs/log.txt";
  file.open(filename, std::ios::out);

  ofstream file2;
  string filename2;
  filename2 = "/home/cair1/Desktop/octavelogs/logkukaoctave.m";
  file2.open(filename2, std::ios::out);
  file2 << "data=[]" << std::endl;

  // Main control loop

  ROS_INFO("MAIN LOOP STARTED!!!");

  VectorXd qAnt, q, qNext;
  VectorXd qdot, qdotNext;
  VectorXd qdotReal, qdotRealSmoothed;
  ConstControlResult ccr;
  AccelConstControlParam param;
  double t;
  double frq = 50.0;  // 10
  double dt = 1.0 / frq;
  int N = 5;
  double dti = dt / ((double)N);
  double dist;

  // Now begin the controller

  ros::Rate loop_rate(frq);

  param.h = 0.05;  // 0.05
  param.kconv = 0.5;
  param.distSafeObs = param.h > 0 ? 0.205 : 0.02;  // 0.2 for h>0 and 0.015 for h=0
  param.considerAutoCollision = false;
  param.etaObs = 0.8;
  param.maxDistAABB = 100000;

  bool reachedZeroPosition = false;

  file << "q = [];" << std::endl;
  file << "qdot = [];" << std::endl;
  file << "qddot = [];" << std::endl;

  bool first = true;

  double fac = 60;  // 80

  bool notEnded = true;

  double totalTime = 0;
  int noRuns = 0;

  file2 << "t = [];" << std::endl;
  file2 << "q = [];" << std::endl;
  file2 << "qdot = [];" << std::endl;
  file2 << "qddot = [];" << std::endl;
  file2 << "timeSpent = [];" << std::endl;
  file2 << "minD = [];" << std::endl;
  file2 << "minD0 = [];" << std::endl;
  file2 << "minD1 = [];" << std::endl;
  file2 << "minD2 = [];" << std::endl;
  file2 << "minD3 = [];" << std::endl;
  file2 << "V = [];" << std::endl;

  qdot = VectorXd::Zero(7);
  while (ros::ok() && notEnded)
  {
    ros::spinOnce();

    if (!reachedZeroPosition)
    {
      // Send the robot to the zero mechanical position
      ROS_INFO_STREAM("Sending the robot to mechanical zero...");

      qROS.position.a1 = 0;
      qROS.position.a2 = 0;
      qROS.position.a3 = 0;
      qROS.position.a4 = 0;
      qROS.position.a5 = 0;
      qROS.position.a6 = 0;
      qROS.position.a7 = 0;
      jointPub.publish(qROS);
    }

    if (histq.size() > 0 && histq[histq.size() - 1].norm() <= 0.001 && !reachedZeroPosition)
    {
      ROS_INFO_STREAM("Mechanical zero reached. Starting controller." << std::endl);
      reachedZeroPosition = true;
      begin = ros::Time::now();
    }

    if (reachedZeroPosition)
    {
      double t = ros::Time::now().toSec() - begin.toSec();
      ROS_INFO_STREAM("Time: " << t);

      if (histq.size() > 0 && histdotq.size() > 0)
      {
        q = histq[histq.size() - 1];

        if (histq.size() > 1)
        {
          qdot = 0.98 * qdot + 0.02 * (histq[histq.size() - 1] - histq[histq.size() - 2]) / dt;
        }
        else
        {
          // qdot = histdotq[histdotq.size() - 1];
          qdot = VectorXd::Zero(7);
        }

        // Make the simulation for N steps ahead
        VectorXd qsim = q;
        VectorXd qdotsim = qdot;

        for (int n = 0; n < N; n++)
        {
          ccr = robot->accelConstControl(htmTg, qdotsim / fac, obstacles, NULL, param, qsim);
          ccr.action = (fac * fac) * ccr.action;

          if (ccr.feasible)
          {
            qsim += qdotsim * dti;
            qdotsim += ccr.action * dti;
          }
          else
          {
            ROS_INFO_STREAM("Unfeasible after " << n << " steps...");
            n = N;
          }
        }

        // ROS_INFO_STREAM("diff" << (q - qsim).norm());

        if (!ccr.feasible)
        {
          ROS_INFO_STREAM("Not feasible!!!" << std::endl);
          notEnded = false;

          // Start debug

          // Create all constraints with collisions with the obstacles
          DistanceRobotObjResult dlo, dloOld, dloNext;

          q = qsim;
          qdot = qdotsim;

          file << "#Debug for the optimization problem:" << std::endl;

          file << "# b = " << printVector(ccr.b) << std::endl << std::endl;

          for (int i = 0; i < obstacles.size(); i++)
          {
            dloOld = NULL;
            dlo = iiwa.computeDistToObj(obstacles[i], q, Matrix4d::Identity(), &dloOld, param.tol, param.h,
                                        param.maxDistAABB);
            dloNext = iiwa.computeDistToObj(obstacles[i], q + qdot * param.dt, Matrix4d::Identity(), &dlo, param.tol,
                                            param.h, param.maxDistAABB);

            ccr.distancesObjResult.push_back(dlo);

            file << "# Obstacle " << i << ": " << std::endl << std::endl;
            for (int j = 0; j < dlo.noElements; j++)
            {
              // Try to find the respective element in dloNext.

              DistanceLinkObjResult* dlorNext = dloNext.getItem(dlo[j].linkNumber, dlo[j].colObjLinkNumber);
              if (dlorNext != NULL)
              {
                double dist = dlo[j].distance - param.distSafeObs;
                double distDot = (dlorNext->distance - dlo[j].distance) / param.dt;
                VectorXd gradDot = (dlorNext->jacDist - dlo[j].jacDist) / param.dt;
                double coriolisCentrifugal = (gradDot.transpose() * qdot)[0];
                double K = param.etaObs;
                double b = -coriolisCentrifugal - 2 * K * distDot - K * K * dist;

                file << "#  Link " << dlo[j].linkNumber << ", col object " << dlo[j].colObjLinkNumber << ": "
                     << std::endl;
                file << "#   dist = " << dist << std::endl;
                file << "#   distDot = " << distDot << std::endl;
                file << "#   gradDot = " << printVector(gradDot) << std::endl;
                file << "#   coriolisCentrifugal = " << coriolisCentrifugal << std::endl;
                file << "#   b = " << b << (b > 0 ? " !CRITICAL " : " ") << std::endl << std::endl;
              }
            }
          }
          //
        }
        else
        {
          qdotNext = (qsim - q) / (dt);
          qdotNext = qdotsim;

          qdotROS.velocity.a1 = qdotNext[0];
          qdotROS.velocity.a2 = qdotNext[1];
          qdotROS.velocity.a3 = qdotNext[2];
          qdotROS.velocity.a4 = qdotNext[3];
          qdotROS.velocity.a5 = qdotNext[4];
          qdotROS.velocity.a6 = qdotNext[5];
          qdotROS.velocity.a7 = qdotNext[6];

          // velPub.publish(qdotROS);

          qNext = qsim;

          // Position controllerq =
          qROS.position.a1 = qNext[0];
          qROS.position.a2 = qNext[1];
          qROS.position.a3 = qNext[2];
          qROS.position.a4 = qNext[3];
          qROS.position.a5 = qNext[4];
          qROS.position.a6 = qNext[5];
          qROS.position.a7 = qNext[6];
          jointPub.publish(qROS);

          // Print

          // file << "q = [q " << qNext[0] << "];" << std::endl;
          // file << "qdot = [q " << qdotsim[0] << "];" << std::endl;
          // file << "qddot = [qddot " << ccr.action[0] << "];" << std::endl;

          iiwa.setConfig(q);

          // ROS_INFO_STREAM(iiwa.links[5].colObjs[0]->htm);
          // ROS_INFO_STREAM("BAAA");
          // ROS_INFO_STREAM(obstacles[3]->htm);

          DistanceStruct dsh =
              GeometricPrimitives::computeDist(iiwa.links[5].colObjs[0], obstacles[3], Vector3d::Zero(), param.h);
          DistanceStruct ds0 =
              GeometricPrimitives::computeDist(iiwa.links[5].colObjs[0], obstacles[3], Vector3d::Zero(), 0);
          // ROS_INFO_STREAM("aa");

          file << "q.append(" << printVector(q) << ")" << std::endl;
          file << "qdot.append(" << printVector(qdot) << ")" << std::endl;
          file << "qddot.append(" << printVector(ccr.action) << ")" << std::endl;
          file << "t.append(" << t << ")" << std::endl;
          for (int n = 0; n < ccr.distancesObjResult.size(); n++)
          {
            for (int g = 0; g < ccr.distancesObjResult[n].noElements; g++)
            {
              int indObs = n;
              int indLink = ccr.distancesObjResult[n][g].linkNumber;
              int indColLink = ccr.distancesObjResult[n][g].colObjLinkNumber;
              Vector3d pointObs = ccr.distancesObjResult[n][g].witnessObj;
              Vector3d pointLink = ccr.distancesObjResult[n][g].witnessColObjLink;

              file << "ballObs_Obs" << indObs << "Link" << indLink << "ColObj" << indColLink << ".add_ani_frame(" << t
                   << ", ub.Utils.trn(" << printVector(pointObs) << "))" << std::endl;
              file << "ballLink_Obs" << indObs << "Link" << indLink << "ColObj" << indColLink << ".add_ani_frame(" << t
                   << ", ub.Utils.trn(" << printVector(pointLink) << "))" << std::endl;
            }
          }

          ROS_INFO_STREAM("Position error:" << std::to_string(1000 * ccr.taskResult.maxErrorPos) << " milimeters");
          ROS_INFO_STREAM("Orientation error:" << std::to_string(ccr.taskResult.maxErrorOri) << " degrees"
                                               << std::endl);

          // Print to octave

          file2 << "t = [t " << t << "];" << std::endl;
          file2 << "q = [q; " << printVectorOctave(q) << "];" << std::endl;
          file2 << "qdot = [qdot; " << printVectorOctave(qdot) << "];" << std::endl;
          file2 << "qddot = [qddot; " << printVectorOctave(ccr.action) << "];" << std::endl;
          file2 << "timeSpent = [timeSpent " << ccr.milisecondsSpent << "];" << std::endl;

          double D0 = iiwa.computeDistToObj(obstacles[0], q, Matrix4d::Zero(), NULL, 0.0005, 0.001, 10000)
                          .getClosest()
                          .distance;
          double D1 = iiwa.computeDistToObj(obstacles[1], q, Matrix4d::Zero(), NULL, 0.0005, 0.001, 10000)
                          .getClosest()
                          .distance;
          double D2 = iiwa.computeDistToObj(obstacles[2], q, Matrix4d::Zero(), NULL, 0.0005, 0.001, 10000)
                          .getClosest()
                          .distance;
          double D3 = iiwa.computeDistToObj(obstacles[3], q, Matrix4d::Zero(), NULL, 0.0005, 0.001, 10000)
                          .getClosest()
                          .distance;

          double minD = 100000;
          minD = D0;
          minD = min(minD, D1);
          minD = min(minD, D2);
          minD = min(minD, D3);

          file2 << "minD = [minD " << minD << "];" << std::endl;
          file2 << "minD0 = [minD0 " << D0 << "];" << std::endl;
          file2 << "minD1 = [minD1 " << D1 << "];" << std::endl;
          file2 << "minD2 = [minD2 " << D2 << "];" << std::endl;
          file2 << "minD3 = [minD3 " << D3 << "];" << std::endl;

          double alpha = param.kconv;
          double beta = param.beta;
          double sigma = param.sigma;

          TaskResult tr = iiwa.taskFunction(htmTg, q);
          VectorXd r = tr.task;
          VectorXd rdot = tr.jacTask * qdot;

          double V = alpha * pow((rdot + alpha * r).norm(), 2) + sigma * beta * pow(qdot.norm(), 2);

          file2 << "V = [V " << V << "];" << std::endl;

          ROS_INFO_STREAM("Critical distance: " << minD);

          if (minD < 0.005)
          {
            notEnded = false;
            ROS_INFO_STREAM("Colided!");
          }
        }
      }
    }

    loop_rate.sleep();
  }

  // ros::spin();
}
