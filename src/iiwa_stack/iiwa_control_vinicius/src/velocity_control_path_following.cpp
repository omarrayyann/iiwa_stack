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
#include <fstream>
#include <string>

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
double kp = 0.01;
ros::Publisher vel_pub;
vector<VectorXd> path;

using namespace std;
vector<string> split(string s, string delimiter);

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

  // TaskResult taskres = robot->taskFunction(htmTg, q);
  // ROS_INFO_STREAM(taskres.task);
  VectorXd qdot = 0.1 * Utils::vectorField(q, path);

  // Publish the computed joint velocity

  qdotROS.velocity.a1 = qdot[0];
  qdotROS.velocity.a2 = qdot[1];
  qdotROS.velocity.a3 = qdot[2];
  qdotROS.velocity.a4 = qdot[3];
  qdotROS.velocity.a5 = qdot[4];
  qdotROS.velocity.a6 = qdot[5];
  qdotROS.velocity.a7 = qdot[6];

  // vel_pub.publish(qdotROS);
}

float degToRad = M_PI / 180.0;

void fillVector()
{
  path;
  ifstream file;
  file.open("/home/cair1/github_packages/iiwa_stack/src/iiwa_stack/iiwa_control_vinicius/src/test.txt");
  string line;
  while (!file.eof())
  {
    getline(file, line);
    vector<string> joint = split(line, ", ");
    cout << line << endl;
    VectorXd set;
    // set << stof(joint.at(0)), stof(joint.at(1)), stof(joint.at(2)), stof(joint.at(3)), stof(joint.at(4)),
    //     stof(joint.at(5)), stof(joint.at(6));
    cout << set * degToRad;
    path.push_back(set * degToRad);
  }
}

int main(int argc, char** argv)
{
  fillVector();
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

vector<string> split(string s, string delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  string token;
  vector<string> res;

  while ((pos_end = s.find(delimiter, pos_start)) != string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));
  return res;
}