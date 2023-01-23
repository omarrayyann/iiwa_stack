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

// Main
Matrix4d htmTg;
Manipulator* robot;
double kp = 0.03;
ros::Publisher vel_pub;

void commandCallback(const iiwa_msgs::JointPosition::ConstPtr& msg)
{
  iiwa_msgs::JointVelocity qdotROS;
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

  ROS_INFO("Recieved a new message: ");
  //   cout << q[0] << endl;
  //   cout << q[1] << endl;
  //   cout << q[2] << endl;

  TaskResult taskres = robot->taskFunction(htmTg, q);
  ROS_INFO_STREAM(taskres.task);
  VectorXd qdot = -kp * Utils::pinv(taskres.jacTask, 0.001) * (Utils::vecPow(taskres.task, 0, 6, 0.5));

  // Publish the computed joint velocity

  qdotROS.velocity.a1 = qdot[0];
  qdotROS.velocity.a2 = qdot[1];
  qdotROS.velocity.a3 = qdot[2];
  qdotROS.velocity.a4 = qdot[3];
  qdotROS.velocity.a5 = qdot[4];
  qdotROS.velocity.a6 = qdot[5];
  qdotROS.velocity.a7 = qdot[6];

  vel_pub.publish(qdotROS);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "iiwa_control_vinicius");
  ros::NodeHandle n;
  vel_pub = n.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 1000);

  Manipulator iiwa = Manipulator::createKukaIIWA();
  robot = &iiwa;

  htmTg = Utils::trn(0.5, 0, -0.5) * iiwa.fk().htmTool;
  ros::Subscriber q_sub = n.subscribe("/iiwa/state/JointPosition", 1000, commandCallback);

  ros::Rate loop_rate(100);

  ros::spin();
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