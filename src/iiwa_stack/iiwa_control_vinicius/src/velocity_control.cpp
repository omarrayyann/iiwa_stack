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

// #include <trajectory_msgs/JointTrajectory.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>

#include "utils.cpp"
#include "distance.cpp"
#include "robot.cpp"

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
vector<VectorXd> histdotqReal;

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

  histdotqReal.push_back(qdot);
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

  htmTg = Utils::trn(0.5, -0.3, 0.7) * Utils::rotx(M_PI / 2);  // y = -0.2

  // Initialize log file
  ofstream file;
  string filename;
  filename = "/home/cair1/Desktop/octavelogs/log" + std::to_string(ros::Time::now().nsec) + ".m";

  // filename = "/home/cair1/Desktop/octavelogs/log.txt";
  file.open(filename, std::ios::out);
  file << "t=[];" << std::endl;
  file << "tsim=[];" << std::endl;
  file << "q=[];" << std::endl;
  file << "qsim=[];" << std::endl;
  file << "qdot=[];" << std::endl;
  file << "qdotr=[];" << std::endl;
  file << "distObs=[];" << std::endl;
  file << "distAuto=[];" << std::endl;
  file << "errorPos=[];" << std::endl;
  file << "errorOri=[];" << std::endl;

  ofstream file2;
  string filename2;
  filename2 = "/home/cair1/Desktop/uaibotlogs/log" + std::to_string(ros::Time::now().nsec) + ".txt";
  file2.open(filename2, std::ios::out);
  file2 << "data=[]" << std::endl;

  // Main control loop

  ROS_INFO("MAIN LOOP STARTED!!!");

  VectorXd q;
  VectorXd qdot;
  VectorXd qdotReal, qdotRealSmoothed;
  ConstControlResult ccr;
  ConstControlParam param;
  double t;
  double frq = 10;  // 10
  double dt = 1 / frq;
  double dist;
  double kgain = 0.35;  // 0.3

  // Run simulation for comparison later
  q = iiwa.q;
  t = 0;

  qdotRealSmoothed = VectorXd::Zero(7);

  while (t < 60)
  {
    param.kpos = 0.5 * min(t / 4.0, 1.0);
    param.kori = 0.2 * min(t / 4.0, 1.0);

    ccr = robot->constControl(htmTg, obstacles, NULL, param, q);
    q += kgain * ccr.action * dt;

    file << "qsim = [qsim; [";
    for (int i = 0; i < 7; i++)
    {
      file << q[i];
      if (i != 6)
      {
        file << " ";
      }
    }
    file << "]];" << endl;
    file << "tsim = [tsim " << t << "];" << endl;

    t += dt;
  }

  // Now begin the controller

  ros::Rate loop_rate(frq);

  bool reachedZeroPosition = false;

  while (ros::ok())
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

      // Set smooth start
      param.kpos = 0.5 * min(t / 4.0, 1.0);
      param.kori = 0.2 * min(t / 4.0, 1.0);

      if (histq.size() > 0 && histdotqReal.size() > 0)
      {
        q = histq[histq.size() - 1];

        ccr = robot->constControl(htmTg, obstacles, NULL, param, q);

        if (!ccr.feasible)
        {
          ROS_INFO_STREAM("Not feasible!!!" << std::endl);
        }
        else
        {
          qdot = kgain * ccr.action;

          histdotq.push_back(qdot);

          qdotReal = histdotqReal[histdotqReal.size() - 1];

          // Modified qdot
          // qdotRealSmoothed = 0.99 * qdotRealSmoothed + 0.01 * qdotReal;
          // qdot = qdot + 0.1 * (qdot - qdotRealSmoothed);

          // Velocity controller

          qdotROS.velocity.a1 = qdot[0];
          qdotROS.velocity.a2 = qdot[1];
          qdotROS.velocity.a3 = qdot[2];
          qdotROS.velocity.a4 = qdot[3];
          qdotROS.velocity.a5 = qdot[4];
          qdotROS.velocity.a6 = qdot[5];
          qdotROS.velocity.a7 = qdot[6];

          velPub.publish(qdotROS);

          // Position controllerq =
          // qROS.position.a3 = nextq[2];
          // qROS.position.a4 = nextq[3];
          // qROS.position.a5 = nextq[4];
          // qROS.position.a6 = nextq[5];
          // qROS.position.a7 = nextq[6];
          // jointPub.publish(qROS);

          // Print messages

          ROS_INFO_STREAM("Position error:" << std::to_string(1000 * ccr.taskResult.maxErrorPos) << " milimeters");
          ROS_INFO_STREAM("Orientation error:" << std::to_string(ccr.taskResult.maxErrorOri) << " degrees"
                                               << std::endl);

          // Print results for debug

          // Print time
          file << "t = [t " << t << "];" << endl;

          // Print minimum distance to obstacles

          dist = 1000.0;
          for (int i = 0; i < obstacles.size(); i++)
          {
            dist = min(dist, iiwa.computeDistToObj(obstacles[i], q).getClosest().distance);
          }

          file << "distObs = [distObs " << dist << "];" << endl;

          // Print minimum distance to autocollision

          dist = 1000.0;
          for (int i = 0; i < obstacles.size(); i++)
          {
            dist = min(dist, iiwa.computeDistAuto(q).getClosest().distance);
          }

          file << "distAuto = [distAuto " << dist << "];" << endl;

          // Print position and orientation error
          file << "errorPos = [errorPos " << ccr.taskResult.maxErrorPos << "];" << endl;
          file << "errorOri = [errorOri " << ccr.taskResult.maxErrorOri << "];" << endl;

          // Print configuration

          file << "q = [q; [";
          for (int i = 0; i < 7; i++)
          {
            file << q[i];
            if (i != 6)
            {
              file << " ";
            }
          }
          file << "]];" << endl;

          // Print target velocity

          file << "qdot = [qdot; [";
          for (int i = 0; i < 7; i++)
          {
            file << qdot[i];
            if (i != 6)
            {
              file << " ";
            }
          }
          file << "]];" << endl;

          // Print real velocity
          file << "qdotr = [qdotr; [";
          for (int i = 0; i < 7; i++)
          {
            file << qdotReal[i];
            if (i != 6)
            {
              file << " ";
            }
          }
          file << "]];" << endl;

          // Print to uaibot replay
          // file2 << "manip.add_ani_frame(time = " << t << ", q = [";
          // for (int i = 0; i < 7; i++)
          // {
          //   file2 << q[i];
          //   if (i != 6)
          //   {
          //     file2 << ", ";
          //   }
          // }
          // file2 << "])" << endl;
          file2 << "data.append([" << t << ", [";
          for (int i = 0; i < 7; i++)
          {
            file2 << q[i];
            if (i != 6)
            {
              file2 << ", ";
            }
          }
          file2 << "]])" << endl;
        }
      }
    }

    loop_rate.sleep();
  }

  // ros::spin();
}

// #include <iostream>
// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Dense>
// #include <list>
// #include <math.h>
// #include <vector>
// #include <random>
// #include <memory>
// #include <ros/ros.h>
// #include <ros/service.h>
// #include <fstream>
// #include <string>

// // #include <trajectory_msgs/JointTrajectory.h>
// // #include <trajectory_msgs/JointTrajectoryPoint.h>
// #include <string>

// #include "utils.cpp"
// #include "distance.cpp"
// #include "robot.cpp"

// // #include "geometry_msgs/Twist.h"
// // #include "geometry_msgs/Pose.h"

// // #include "gazebo_msgs/LinkStates.h"
// // #include "gazebo_msgs/ModelState.h"
// // #include "gazebo_msgs/SetModelState.h"

// // #include "iiwa_ros/command/joint_velocity.hpp"
// // #include "iiwa_ros/state/joint_position.hpp"
// #include "iiwa_msgs/JointVelocity.h"
// #include "iiwa_msgs/JointPosition.h"
// #include <ctime>

// // Main
// Matrix4d htmTg;
// Manipulator* robot;
// double kp = 0.01;
// ros::Publisher vel_pub;
// vector<VectorXd> path;

// using namespace std;
// vector<string> split(string s, string delimiter);
// bool startedTime = false;
// time_t startingTime;

// void commandCallback(const iiwa_msgs::JointPosition::ConstPtr& msg)
// {
//   if (!startedTime)
//   {
//     startingTime = time(0);
//     startedTime = 1;
//   }

//   ofstream qFile;
//   qFile.open("q.txt", ios::app);

//   ofstream qDotFile;
//   qDotFile.open("qdot.txt", ios::app);

//   ofstream piFile;
//   piFile.open("pi.txt", ios::app);

//   ofstream tFile;
//   tFile.open("t.txt", ios::app);

//   ofstream dMinFile;
//   dMinFile.open("d_min.txt", ios::app);

//   iiwa_msgs::JointVelocity qdotROS;
//   iiwa_msgs::JointPosition qROS = *msg;

//   // Capture the joint position
//   VectorXd q(7);
//   q[0] = qROS.position.a1;
//   q[1] = qROS.position.a2;
//   q[2] = qROS.position.a3;
//   q[3] = qROS.position.a4;
//   q[4] = qROS.position.a5;
//   q[5] = qROS.position.a6;
//   q[6] = qROS.position.a7;

//   ROS_INFO("Recieved a new message: ");

//   // TaskResult taskres = robot->taskFunction(htmTg, q);
//   // ROS_INFO_STREAM(taskres.task);
//   VectorFieldResult vfr = Utils::vectorField(q, path, 8);
//   VectorXd qdot = 0.2 * vfr.v;

//   cout << "Joint3" << q[3] << endl;

//   qFile << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "," << q[4] << "," << q[5] << "," << q[6] << endl;
//   qDotFile << qdot[0] << "," << qdot[1] << "," << qdot[2] << "," << qdot[3] << "," << qdot[4] << "," << qdot[5] <<
//   ","
//            << qdot[6] << endl;
//   piFile << vfr.pi[0] << "," << vfr.pi[1] << "," << vfr.pi[2] << "," << vfr.pi[3] << "," << vfr.pi[4] << ","
//          << vfr.pi[5] << "," << vfr.pi[6] << endl;
//   tFile << ros::Time::now() << endl;
//   dMinFile << vfr.dist << endl;

//   // cout << "Joint Position Velocities: " << endl << qdot.norm() << endl;

//   // cout << "Distance: " << vfr.dist << std::endl;

//   // Publish the computed joint velocity

//   qdotROS.velocity.a1 = qdot[0];
//   qdotROS.velocity.a2 = qdot[1];
//   qdotROS.velocity.a3 = qdot[2];
//   qdotROS.velocity.a4 = qdot[3];
//   qdotROS.velocity.a5 = qdot[4];
//   qdotROS.velocity.a6 = qdot[5];
//   qdotROS.velocity.a7 = qdot[6];

//   vel_pub.publish(qdotROS);
// }

// float degToRad = M_PI / 180.0;

// void fillVector()
// {
//   ifstream file;
//   file.open("/home/cair1/github_packages/iiwa_stack/src/iiwa_stack/iiwa_control_vinicius/src/test.txt");
//   string line;
//   while (!file.eof())
//   {
//     getline(file, line);
//     vector<string> joint = split(line, ", ");
//     // cout << line << endl;
//     VectorXd set(7);
//     set << stof(joint.at(0)), stof(joint.at(1)), stof(joint.at(2)), stof(joint.at(3)), stof(joint.at(4)),
//         stof(joint.at(5)), stof(joint.at(6));
//     cout << set * degToRad;
//     path.push_back(set * degToRad);
//   }
// }

// int main(int argc, char** argv)
// {
//   fillVector();
//   ros::init(argc, argv, "iiwa_control_vinicius");
//   ros::NodeHandle n;
//   vel_pub = n.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 1000);

//   Manipulator iiwa = Manipulator::createKukaIIWA();
//   robot = &iiwa;

//   htmTg = Utils::trn(0.5, 0, -0.5) * iiwa.fk().htmTool;
//   ros::Subscriber q_sub = n.subscribe("/iiwa/state/JointPosition", 1000, commandCallback);

//   ros::Rate loop_rate(100);

//   ros::spin();
// }

// vector<string> split(string s, string delimiter)
// {
//   size_t pos_start = 0, pos_end, delim_len = delimiter.length();
//   string token;
//   vector<string> res;

//   while ((pos_end = s.find(delimiter, pos_start)) != string::npos)
//   {
//     token = s.substr(pos_start, pos_end - pos_start);
//     pos_start = pos_end + delim_len;
//     res.push_back(token);
//   }

//   res.push_back(s.substr(pos_start));
//   return res;
// }