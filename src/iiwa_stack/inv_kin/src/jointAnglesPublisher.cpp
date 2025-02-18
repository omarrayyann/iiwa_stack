/*
 * Inverse Kinematiks of 7-DOF KUKA LBR
 * Author: Sebastian Doliwa
 * Date: 24.03.2020
 * --------------------------------------------------------------------------
 * function input:
 *   - [X Y Z] is the task position in Cartesian coordinates [mm]
 *   - eef_phi and eef_theta is the task orientation of the end-effector [deg]
 *   - armAng is the position on the redundancy circle of the elbow [deg]
 *
 * function output:
 *   - 7 module angels [deg]
 * --------------------------------------------------------------------------
 */

// #include <QCoreApplication>
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

using namespace std;

// for udp -----------------------------------------------------------------------------------------
#define NPACK 10
#define PORT 30000
#define SERVER "10.2.128.86"  // dell local
// #define SERVER "10.2.4.53"
#define BUFLEN 512  // Max length of buffer
struct sockaddr_in si_other;
int s, slen = sizeof(si_other);
char message[BUFLEN];
void die(char* s);
void init_udp();
// command line (linux): nc -lvu <port>

//----------------------------------------------------------

#define pi 3.14159265

double deg2rad = pi / 180.0;
bool axis_limit_violation = false;

vector<double> ex = {1, 0, 0};
vector<double> ey = {0, 1, 0};
vector<double> ez = {0, 0, 1};

vector<double> pw = {0, 0, 0};
vector<double> psw = {0, 0, 0};
vector<double> pc = {0, 0, 0};

// angles [deg]
double phi1 = 0;
double phi1_1 = 0;
double phi1_2 = 0;
double phi2 = 0;
double phi2_1 = 0;
double phi2_2 = 0;
double phi3 = 0;
double phi3_1 = 0;
double phi3_2 = 0;
double phi4 = 0;
double phi4_1 = 0;
double phi4_2 = 0;
double phi5 = 0;
double phi5_1 = 0;
double phi5_2 = 0;
double phi6 = 0;
double phi6_1 = 0;
double phi6_2 = 0;
double phi7 = 0;
double phi7_1 = 0;
double phi7_2 = 0;

// old angles [rad]
double phi1_old = 0;
double phi2_old = 0;
double phi3_old = 0;
double phi4_old = 0;
double phi5_old = 0;
double phi6_old = 0;
double phi7_old = 0;

// // axis limit [rad]
// double phi1_max = 2.967055;  // 170°
// double phi2_max = 2.09435;   // 120°
// double phi3_max = 2.967055;  // 170°
// double phi4_max = 2.09435;   // 120°
// double phi5_max = 2.967055;  // 170°
// double phi6_max = 2.09435;   // 120°
// double phi7_max = 3.054325;  // 175°

double phi1_max = 2.875;  // 170°
double phi2_max = 2.0;    // 120°
double phi3_max = 2.875;  // 170°
double phi4_max = 2.0;    // 120°
double phi5_max = 2.875;  // 170°
double phi6_max = 2.0;    // 120°
double phi7_max = 2.96;   // 175°

float stickLength = 0;

// arm angle
double armAng = 0;  // 0-360°

// Parameters ----------------------------------------------------------------
// limb length [mm], total: 1330mm (with zimmer group gripper), else: 1266mm
double d_bs = 360;
double d_se = 420;
double d_ew = 400;
// double d_wf = 152;  // without gripper (x,y,z output of smartPad is without gripper)
// double d_wf = 639;  // with// double d_wf = 639;                         // with 487 gripper (x,y,z output of
// smartPad is without gripper)
double d_wf = 152 + 400;  // without 48.7 gripper (x,y,z output of smartPad is without gripper)

vector<double> p_shoulder = {0, 0, d_bs};  // position of shoulder
vector<double> vec_eef = {0, 0, 0};        // vector base (x=y=z=0) to tcp tip
vector<double> vec_wf_tmp = {0, 0, d_wf};  // vector wrist endeffector [wf]

// prototypes
bool inv_kin_kuka(double X, double Y, double Z, double eef_phi, double eef_theta, double armAng_in, float* jointAngles);
void inv_kin_kuka_angle_calc(double X, double Y, double Z, double eef_phi, double eef_theta, double armAng);
double adapt_elbow_position(double X, double Y, double Z, double eef_phi, double eef_theta, double armAng);
vector<double> Mat_vec_prod_Rx(double theta, vector<double> input_vector);
vector<double> Mat_vec_prod_Ry(double theta, vector<double> input_vector);
vector<double> Mat_vec_prod_Rz(double theta, vector<double> input_vector);
vector<double> Vector_substraction(vector<double> input_vector, vector<double> input_vector2);
vector<double> Vector_addition(vector<double> input_vector, vector<double> input_vector2);
vector<double> Vector_division(vector<double> input_vector, double divider);
vector<double> Vector_multi(vector<double> input_vector, double value);
vector<double> Vector_cross(vector<double> input_vector, vector<double> input_vector2);
double Vector_scalar(vector<double> input_vector, vector<double> input_vector2);
vector<double> Elbow_Position(double armAng, double R);
void init_udp();
void die(char* s);
vector<string> split(string s, string delimiter);
string removeSpaces(string s);
ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pubSim;

#include <iostream>
#include <conio.h>

using namespace std;

float increment = 0.2;

void getVariablesFromConsole(float& x, float& y, float& z, float& eefPhi, float& eefTheta, float& armAngle)
{
  cout << "X position: ";
  while (!(cin >> x))
  {
    cin.clear();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << "Value entered incorrect, please try again!" << endl;
    cout << "X position: ";
  }

  cout << "Y position: ";
  while (!(cin >> y))
  {
    cin.clear();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << "Value entered incorrect, please try again!" << endl;
    cout << "Y position: ";
  }

  cout << "Z position: ";
  while (!(cin >> z))
  {
    cin.clear();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << "Value entered incorrect, please try again!" << endl;
    cout << "Z position: ";
  }

  cout << "End Effecetor Phi orientation angle (degrees): ";
  while (!(cin >> eefPhi))
  {
    cin.clear();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << "Value entered incorrect, please try again!" << endl;
    cout << "End Effecetor Phi orientation angle (degrees): ";
  }

  cout << "End Effecetor Theta orientation angle (degrees): ";
  while (!(cin >> eefTheta))
  {
    cin.clear();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << "Value entered incorrect, please try again!" << endl;
    cout << "End Effecetor Theta orientation angle (degrees): ";
  }

  cout << "Arm Angle: ";
  while (!(cin >> armAngle))
  {
    cin.clear();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << "Value entered incorrect, please try again!" << endl;
    cout << "Arm Angle: ";
  }
}

void fixForStick(float& xPosition, float& yPosition, float& zPosition, float eefPhiOrientation,
                 float eefThetaOrientation)
{
  float eefThetaOrientationRadians = eefThetaOrientation * deg2rad;
  float eefPhiOrientationRadians = eefPhiOrientation * deg2rad;
  cout << "Before: " << endl << xPosition << endl << yPosition << endl << zPosition << endl;
  xPosition -= stickLength * sin(-eefThetaOrientationRadians) * cos(eefPhiOrientationRadians + (M_PI / 2));
  yPosition -= stickLength * sin(-eefThetaOrientationRadians) * sin(eefPhiOrientationRadians + (M_PI / 2));
  zPosition -= stickLength * cos(-eefThetaOrientationRadians);
  cout << "After: " << endl << xPosition << endl << yPosition << endl << zPosition << endl;
}

bool publishNewEEF(ros::Publisher jointAnglesPublisher, ros::Publisher xyzPublisher, float xPosition, float yPosition,
                   float zPosition, float eefPhiOrientation, float eefThetaOrientation, float armAngle)
{
  if (zPosition <= 42)
  {
    ROS_INFO("DID NOT SEND ANGLES - SAFETY");

    return false;
  }

  std_msgs::Float32MultiArray messageArray;
  float* jointAngles = new float[7];

  // fixForStick(xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation);
  if (inv_kin_kuka(xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle, jointAngles))
  {
    if (abs(jointAngles[0]) > 165 || abs(jointAngles[1]) > 115 || abs(jointAngles[2]) > 165 ||
        abs(jointAngles[3]) > 115 || abs(jointAngles[4]) > 165 || abs(jointAngles[5]) > 115 ||
        abs(jointAngles[6]) > 170)
    {
      ROS_INFO("DID NOT SEND ANGLES - SAFETY");

      return false;
    }

    messageArray.data.clear();

    iiwa_msgs::JointPosition jointPosition;

    iiwa_msgs::JointQuantity quantity;

    quantity.a1 = jointAngles[0] * M_PI / 180;
    quantity.a2 = jointAngles[1] * M_PI / 180;
    quantity.a3 = jointAngles[2] * M_PI / 180;
    quantity.a4 = jointAngles[3] * M_PI / 180;
    quantity.a5 = jointAngles[4] * M_PI / 180;
    quantity.a6 = jointAngles[5] * M_PI / 180;
    quantity.a7 = jointAngles[6] * M_PI / 180;

    // trajectory_msgs::JointTrajectory jointTrajectory;
    // trajectory_msgs::JointTrajectoryPoint jointTrajectoryPoint;

    // jointTrajectoryPoint.positions.push_back(quantity.a1);
    // jointTrajectoryPoint.positions.push_back(quantity.a2);
    // jointTrajectoryPoint.positions.push_back(quantity.a3);
    // jointTrajectoryPoint.positions.push_back(quantity.a4);
    // jointTrajectoryPoint.positions.push_back(quantity.a5);
    // jointTrajectoryPoint.positions.push_back(quantity.a6);
    // jointTrajectoryPoint.positions.push_back(quantity.a7);

    // jointTrajectory.joint_names.push_back("iiwa_joint0");
    // jointTrajectory.joint_names.push_back("iiwa_joint1");
    // jointTrajectory.joint_names.push_back("iiwa_joint2");
    // jointTrajectory.joint_names.push_back("iiwa_joint3");
    // jointTrajectory.joint_names.push_back("iiwa_joint4");
    // jointTrajectory.joint_names.push_back("iiwa_joint5");
    // jointTrajectory.joint_names.push_back("iiwa_joint6");
    // jointTrajectory.joint_names.push_back("iiwa_joint7");

    // jointTrajectory.points.push_back(jointTrajectoryPoint);

    // cout << "PUB SIM" << endl;

    // pubSim.publish(jointTrajectory);

    jointPosition.position = quantity;

    jointAnglesPublisher.publish(jointPosition);
    messageArray.data.clear();
    messageArray.data = {xPosition, yPosition, zPosition};

    xyzPublisher.publish(messageArray);

    delete[] jointAngles;
    return true;
  }
  else
  {
    ROS_INFO("Could not come up with joint angles required");
    delete[] jointAngles;

    return false;
  }
}

bool goToRequired(ros::Publisher jointAnglesPublisher, ros::Publisher publisher2)

{
  publishNewEEF(jointAnglesPublisher, publisher2, 501.47, -48.7559, 180, 110, 180, 159.106);
}

bool goToZero(ros::Publisher jointAnglesPublisher)
{
  std_msgs::Float32MultiArray messageArray;
  float* jointAngles = new float[7];

  messageArray.data.clear();

  iiwa_msgs::JointPosition jointPosition;

  iiwa_msgs::JointQuantity quantity;

  quantity.a1 = 0;
  quantity.a2 = 0;
  quantity.a3 = 0;
  quantity.a4 = 0;
  quantity.a5 = 0;
  quantity.a6 = 0;
  quantity.a7 = 0;

  jointPosition.position = quantity;

  jointAnglesPublisher.publish(jointPosition);
  messageArray.data.clear();

  delete[] jointAngles;
  return true;
}

void printCommands()

{
  cout << "-------------------------------" << endl;
  cout << "1: Move EEF" << endl;
  cout << "2: Read File" << endl;
  cout << "3: Set new origin" << endl;
  cout << "4: Run txt package" << endl;
  cout << "5: Control using keys" << endl;
  cout << "6: Edit Stick" << endl;
  cout << "7: Exit" << endl;
  cout << "8: Mechanical Zero Position" << endl;
  cout << "10: KUKA CONTROLLER" << endl;
  cout << "11: Touch 3D (Place in Origin Before Starting)" << endl;
  cout << "15: Required" << endl;
  cout << "16: Exit" << endl;

  cout << "Command: ";
}

bool run = true;
bool reachedSentPoint = false;
bool started = false;
vector<float> sentPoint;
vector<float> origin;
vector<vector<float>> points;
int pointIndex = 0;

float tolerance;

void commandCallback(const iiwa_msgs::CartesianPose& msg)
{
  cout << "hrer" << endl;
  if (!started)
  {
    vector<float> vectorPoint = points.at(pointIndex);
    pointIndex++;
    started = true;
    publishNewEEF(pub, pub2, vectorPoint.at(0), vectorPoint.at(1), vectorPoint.at(2), vectorPoint.at(3),
                  vectorPoint.at(4), vectorPoint.at(5));
    sentPoint = vectorPoint;

    cout << "Sent Point: " << endl;
    cout << sentPoint.at(0) << endl;
    cout << sentPoint.at(1) << endl;

    cout << "Current Point: " << endl;
    cout << msg.poseStamped.pose.position.x * 1000 << endl;
    cout << msg.poseStamped.pose.position.y * 1000 << endl;
  }
  else
  {
    float distance = pow(msg.poseStamped.pose.position.x * 1000.0 - sentPoint.at(0), 2.0) +
                     pow(msg.poseStamped.pose.position.y * 1000.0 - sentPoint.at(1), 2.0);
    distance = pow(distance, 0.5);
    // cout << "Sent Point: " << endl;
    // cout << sentPoint.at(0) << endl;
    // cout << sentPoint.at(1) << endl;

    // cout << "Current Point: " << endl;
    // cout << msg.poseStamped.pose.position.x * 1000 << endl;
    // cout << msg.poseStamped.pose.position.y * 1000 << endl;

    cout << "Distance: " << distance << endl;
    // distance = 10900000;
    if (distance < tolerance)
    {
      if (pointIndex < points.size())
      {
        vector<float> vectorPoint = points.at(pointIndex);
        pointIndex++;
        cout << "Index: " << pointIndex << endl;
        publishNewEEF(pub, pub2, vectorPoint.at(0), vectorPoint.at(1), vectorPoint.at(2), vectorPoint.at(3),
                      vectorPoint.at(4), vectorPoint.at(5));
        sentPoint = vectorPoint;
      }
    }
    else
    {
      cout << "More than tolerance" << endl;
    }
  }
}

void preparePoints()
{
  points.clear();
  string fileName = "";
  cout << "File name: " << endl;
  cin >> fileName;
  ifstream file;
  file.open(fileName);

  if (!file.fail())
  {
    file >> tolerance;
    while (!file.eof())
    {
      string filePath = "";
      getline(file, filePath);

      ifstream inputFile;
      inputFile.open(filePath);

      if (!inputFile.fail())
      {
        string line = "";
        std_msgs::Float32MultiArray messageArray;
        getline(inputFile, line);
        while (!inputFile.eof())
        {
          string line = "";
          getline(inputFile, line);
          line = removeSpaces(line);
          vector<string> eefPosition = split(line, ",");
          float xPosition = stof(eefPosition.at(0)) + origin.at(0);
          float yPosition = stof(eefPosition.at(1)) + origin.at(1);
          float zPosition = stof(eefPosition.at(2)) + origin.at(2);
          float eefPhiOrientation = stof(eefPosition.at(3));
          float eefThetaOrientation = stof(eefPosition.at(4));
          float armAngle = stof(eefPosition.at(5));
          vector<float> v;
          v.empty();
          v.push_back(xPosition);
          v.push_back(yPosition);
          v.push_back(zPosition);
          v.push_back(eefPhiOrientation);
          v.push_back(eefThetaOrientation);
          v.push_back(armAngle);
          points.push_back(v);
        }
      }
      else
      {
        cout << "File: " << fileName << " does not exist" << endl;
      };
    }
  }
  else
  {
    cout << "File: " << fileName << " does not exist" << endl;
  };
  cout << "Successfully prepared file!" << endl;
}

vector<float> touch_origin;

float xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle;
float* jointAngles;

void moved_touch(const geometry_msgs::PoseStamped msg)
{
  float x_pos = msg.pose.position.x;
  float y_pos = msg.pose.position.y;
  float z_pos = msg.pose.position.z;

  float x_ori = msg.pose.orientation.x * 180 / M_PI;
  float y_ori = msg.pose.orientation.y * 180 / M_PI;
  float z_ori = msg.pose.orientation.z * 180 / M_PI;
  float w_ori = msg.pose.orientation.w * 180 / M_PI;

  cout << "Position:" << endl;
  cout << "x: " << x_pos << endl;
  cout << "y: " << y_pos << endl;
  cout << "z: " << z_pos << endl;

  cout << "Orientation:" << endl;
  cout << "x: " << x_ori << endl;
  cout << "y: " << y_ori << endl;
  cout << "z: " << z_ori << endl;
  cout << "w: " << w_ori << endl;

  if (touch_origin.empty())
  {
    touch_origin.push_back(x_pos);
    touch_origin.push_back(y_pos);
    touch_origin.push_back(z_pos);
    cout << "set origin successfully" << endl;
  }
  else
  {
    float x_dif = x_pos - touch_origin.at(0);
    float y_dif = y_pos - touch_origin.at(1);
    float z_dif = z_pos - touch_origin.at(2);
    x_dif *= 1000;
    y_dif *= 1000;
    z_dif *= 1000;

    cout << "Position:" << endl;
    cout << "x: " << x_pos << endl;
    cout << "y: " << y_pos << endl;
    cout << "z: " << z_pos << endl;

    // x_dif /= 2;
    // y_dif /= 2;
    // z_dif /= 2;

    publishNewEEF(pub, pub2, y_dif + origin.at(0), -x_dif + origin.at(1), z_dif + origin.at(2), eefPhiOrientation,
                  eefThetaOrientation, armAngle);
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "jointAnglesPublisher");
  ros::NodeHandle n;
  pub = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 100);

  pubSim =
      n.advertise<trajectory_msgs::JointTrajectory>("/iiwa/PositionJointInterface_trajectory_controller/command", 1000);
  pub2 = n.advertise<std_msgs::Float32MultiArray>("eefGoal", 100);

  origin.push_back(450.0);
  origin.push_back(0.0);
  origin.push_back(170.0);

  // preparePoints();
  // init_udp();

  while (ros::ok() && run)
  {
    string commandPicked = "-1";
    printCommands();
    cin >> commandPicked;

    if (commandPicked == "11")
    {
      ros::Subscriber sub = n.subscribe("/phantom/pose", 1000, moved_touch);
      ros::Rate loop_rate(100);
      ros::spin();
    }

    if (commandPicked == "8")
    {
      goToZero(pub);
    }

    if (commandPicked == "16")
    {
      run = 0;
    }
    else if (commandPicked == "15")
    {
      goToRequired(pub, pub2);
    }

    else if (commandPicked == "1")
    {
      getVariablesFromConsole(xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle);

      jointAngles = new float[7];

      publishNewEEF(pub, pub2, xPosition + origin.at(0), yPosition + origin.at(1), zPosition + origin.at(2),
                    eefPhiOrientation, eefThetaOrientation, armAngle);
    }

    else if (commandPicked == "10")
    {
      float increment = 1;
      float increment_angles = 0.5;
      bool fix_joint = false;

      while (1)
      {
        char c = getch();
        switch (c)
        {
          case 'w':
            xPosition += increment;
            break;
          case 's':
            xPosition -= increment;
            break;
          case 'a':
            yPosition += increment;
            break;
          case 'd':
            yPosition -= increment;
            break;
          case 't':
            zPosition += increment;
            break;
          case 'g':
            zPosition -= increment;
            break;
          case 'c':
            eefThetaOrientation += increment;
            break;
          case 'v':
            eefThetaOrientation -= increment;
            break;
          case 'b':
            eefPhiOrientation += increment;
            break;
          case 'n':
            eefPhiOrientation -= increment;
            break;
          case 'i':
            zPosition += cos(-eefPhiOrientation);
            yPosition += sin(-eefPhiOrientation) * sin(eefThetaOrientation - 180);
            cout << eefThetaOrientation << endl;
            xPosition += sin(-eefPhiOrientation) * cos(eefThetaOrientation - 180);
            break;
          case 'o':
            zPosition -= cos(-eefPhiOrientation);
            yPosition -= sin(-eefPhiOrientation) * sin(eefThetaOrientation - 180);
            xPosition -= sin(-eefPhiOrientation) * cos(eefThetaOrientation - 180);
            break;
          default:
            cout << c << endl;
            break;
        }

        publishNewEEF(pub, pub2, xPosition + origin.at(0), yPosition + origin.at(1), zPosition + origin.at(2),
                      eefPhiOrientation, eefThetaOrientation, armAngle);
      }
    }

    else if (commandPicked == "2")
    {
      string fileName = "";
      cout << "File name: " << endl;
      cin >> fileName;
      ifstream inputFile;
      inputFile.open(fileName);
      if (!inputFile.fail())
      {
        cout << "Success" << endl;
        string line = "";
        std_msgs::Float32MultiArray messageArray;
        float increment = 1;

        getline(inputFile, line);
        vector<string> configLine = split(line, ",");
        int maxRate = stof(configLine.at(0));
        ros::Rate rate = ros::Rate(maxRate);
        while (!inputFile.eof())
        {
          string line = "";
          getline(inputFile, line);
          line = removeSpaces(line);

          vector<string> eefPosition = split(line, ",");
          xPosition = stof(eefPosition.at(0)) + origin.at(0);
          yPosition = stof(eefPosition.at(1)) + origin.at(1);
          zPosition = stof(eefPosition.at(2)) + origin.at(2);
          eefPhiOrientation = stof(eefPosition.at(3));
          eefThetaOrientation = stof(eefPosition.at(4));
          armAngle = stof(eefPosition.at(5));

          jointAngles = new float[7];

          publishNewEEF(pub, pub2, xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle);

          rate.sleep();
        }
      }
      else
      {
        cout << "File: " << fileName << " does not exist" << endl;
      };
    }

    else if (commandPicked == "3")
    {
      cout << "Current Origin Relative to to Global Cartesian Graph: " << endl;
      cout << "X: " << origin.at(0) << endl;
      cout << "Y: " << origin.at(1) << endl;
      cout << "Z: " << origin.at(2) << endl;

      cout << "Enter new origin" << endl;

      float x = 0;
      float y = 0;
      float z = 0;

      cout << "X: ";
      cin >> x;
      cout << "Y: ";
      cin >> y;
      cout << "Z: ";
      cin >> z;

      origin[0] = x;
      origin[1] = y;
      origin[2] = z;

      cout << "New Origin Relative to to Global Cartesian Graph: " << endl;
      cout << "X: " << origin.at(0) << endl;
      cout << "Y: " << origin.at(1) << endl;
      cout << "Z: " << origin.at(2) << endl;
    }

    else if (commandPicked == "4")
    {
      string fileName = "";
      cout << "File name: " << endl;
      cin >> fileName;
      ifstream file;
      file.open(fileName);

      if (file.is_open())
      {
        while (!file.eof())
        {
          string filePath = "";
          getline(file, filePath);

          if (filePath.substr(0, 5) == "pause")
          {
            filePath.erase(0, 5);
            ros::Rate rate = ros::Rate(1 / (stof(filePath)));
            rate.sleep();
            continue;
          }

          ifstream inputFile;

          inputFile.open(filePath);

          if (!inputFile.fail())
          {
            cout << "Success" << endl;
            string line = "";
            std_msgs::Float32MultiArray messageArray;
            getline(inputFile, line);
            ros::Rate rate = ros::Rate(80);
            while (!inputFile.eof())
            {
              cout << "here" << endl;
              string line = "";
              getline(inputFile, line);
              line = removeSpaces(line);
              vector<string> eefPosition = split(line, ",");
              xPosition = stof(eefPosition.at(0)) + origin.at(0);
              yPosition = stof(eefPosition.at(1)) + origin.at(1);
              zPosition = stof(eefPosition.at(2)) + origin.at(2);
              eefPhiOrientation = stof(eefPosition.at(3));
              eefThetaOrientation = stof(eefPosition.at(4));
              armAngle = stof(eefPosition.at(5));

              jointAngles = new float[7];

              publishNewEEF(pub, pub2, xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation,
                            armAngle);
              rate.sleep();
            }
          }
          else
          {
            cout << "File: " << fileName << " does not exist" << endl;
          };
        }
      }
    }

    else if (commandPicked == "5")
    {
      float array[6] = {xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle};
      cout << "1: X\n2: Y\n3: Z\n4: Phi\n5: Theta\n6: Arm Angle" << endl;
      int indexToChange;
      cout << "Enter Parameter to change: ";
      cin >> indexToChange;
      indexToChange--;
      publishNewEEF(pub, pub2, xPosition + origin.at(0), yPosition + origin.at(1), zPosition + origin.at(2),
                    eefPhiOrientation, eefThetaOrientation, armAngle);
      bool run = true;
      string commandi = "";
      while (run)
      {
        cout << "Current Value: " << array[indexToChange] << endl;
        cin >> commandi;
        if (commandi == "u")
        {
          array[indexToChange] += 2;
        }
        else if (commandi == "d")
        {
          array[indexToChange] -= 2;
        }
        else
        {
          run = false;
          break;
        }

        cout << "New Value: " << array[indexToChange] << endl;
        xPosition = array[0];
        yPosition = array[1];
        zPosition = array[2];
        eefPhiOrientation = array[3];
        eefThetaOrientation = array[4];
        armAngle = array[5];

        jointAngles = new float[7];

        publishNewEEF(pub, pub2, xPosition + origin.at(0), yPosition + origin.at(1), zPosition + origin.at(2),
                      eefPhiOrientation, eefThetaOrientation, armAngle);
      }
    }

    else if (commandPicked == "6")
    {
      cout << "Current Stick Lenght: " << stickLength << endl;
      cout << "Enter new Stick Length: ";
      cin >> stickLength;
      cout << "Stick Length Updated to: " << stickLength << endl;
    }

    else if (commandPicked == "7")
    {
      run = false;
    }

    else
    {
      cout << "File does not exist!" << endl;
    }
  }
}

float constrainAngle(float xz)
{
  double x = static_cast<double>(xz);
  x = fmod(x, 360);
  if (x < 0) x += 360;
  return (float)x;
}

bool inv_kin_kuka(double X, double Y, double Z, double eef_phi, double eef_theta, double armAng_in, float* jointAngles)
{
  cout << "Commanded: \nPosition (X,Y,Z) [mm]: [" << X << ", " << Y << ", " << Z << "]" << endl;
  cout << "EEF Orientation (Phi, Theta) [deg]: [" << eef_phi << ", " << eef_theta << "]" << endl;
  cout << "Entered arm Angle [deg]: " << armAng << endl;
  cout << "-------------------------------------" << endl;

  // calc joint values
  inv_kin_kuka_angle_calc(X, Y, Z, eef_phi, eef_theta, armAng_in);

  // check for joint violation
  if (abs(phi4) >= phi4_max)
  {
    cout << "Wrist to close to shoulder!" << endl;
    return false;
  }

  else if (abs(phi1) >= phi1_max | abs(phi2) >= phi2_max | abs(phi3) >= phi3_max | abs(phi4) >= phi4_max |
           abs(phi5) >= phi5_max | abs(phi6) >= phi6_max | abs(phi7) >= phi7_max)
  {
    armAng = adapt_elbow_position(X, Y, Z, eef_phi, eef_theta, armAng_in);

    // return false;
  }

  // store calculated joint values for next round
  phi1_old = phi1;
  phi2_old = phi2;
  phi3_old = phi3;
  phi4_old = phi4;
  phi5_old = phi5;
  phi6_old = phi6;
  phi7_old = phi7;

  // print out results ---------------------------------------------------------------------------
  cout << "Angles [deg]: " << endl;
  cout << "(new) Arm Angle: " << armAng << endl;
  cout << "Phi 1: " << phi1_1 << "\nPhi 2: " << phi2_2 << "\nPhi 3: " << phi3_2 << endl;
  cout << "Phi 4: " << phi4_2 << "\nPhi 5: " << phi5_2 << "\nPhi 6: " << phi6_2 << "\nPhi 7: " << phi7_2 << endl;
  cout << "-------------------------------------" << endl;

  if (isnan(phi1_1) || isnan(phi2_2) || isnan(phi3_2) || isnan(phi4_2) || isnan(phi5_2) || isnan(phi6_2) ||
      isnan(phi7_2))
  {
    return false;
  }
  jointAngles[0] = (float)phi1_1;
  jointAngles[1] = (float)phi2_2;
  jointAngles[2] = (float)phi3_2;
  jointAngles[3] = (float)phi4_2;
  jointAngles[4] = (float)phi5_2;
  jointAngles[5] = (float)phi6_2;
  jointAngles[6] = (float)phi7_2;

  return true;
}

double adapt_elbow_position(double X, double Y, double Z, double eef_phi, double eef_theta, double armAng)
{
  double armAng_entered = armAng;  // store enter value by user [deg]
  double armAng_1 = armAng;
  double armAng_2 = armAng;
  double loop_count = 0;

  cout << "ADAPTINGGGG" << endl;
  // first solution
  while (abs(phi1) >= phi1_max | abs(phi2) >= phi2_max | abs(phi3) >= phi3_max | abs(phi4) >= phi4_max |
         abs(phi5) >= phi5_max | abs(phi6) >= phi6_max | abs(phi7) >= phi7_max)
  {
    // use redundancy circle to avoid joint limits
    armAng = armAng + 0.010;
    loop_count += 0.01;

    // calc new angle values
    inv_kin_kuka_angle_calc(X, Y, Z, eef_phi, eef_theta, armAng);

    cout << "!!!ADAPTION!!! " << endl;
    cout << "(new) Arm Angle: " << armAng << endl;
    cout << "Phi 1: " << phi1_1 << "\nPhi 2: " << phi2_2 << "\nPhi 3: " << phi3_2 << endl;
    cout << "Phi 4: " << phi4_2 << "\nPhi 5: " << phi5_2 << "\nPhi 6: " << phi6_2 << "\nPhi 7: " << phi7_2 << endl;
    cout << "-------------------------------------" << endl;

    // if there is no solution
    if (loop_count > 360)
    {
      cout << "Target not reachable (impossible pose)" << endl;
      return 0;
    }
  }

  armAng_1 = armAng;  // new arm angle

  // store final values
  phi1_1 = phi1;
  phi2_1 = phi2;
  phi3_1 = phi3;
  phi4_1 = phi4;
  phi5_1 = phi5;
  phi6_1 = phi6;
  phi7_1 = phi7;

  // calc difference to previous angle values [deg]
  double phi1_diff = phi1_1 - phi1_old;
  double phi2_diff = phi2_1 - phi2_old;
  double phi3_diff = phi3_1 - phi3_old;
  double phi4_diff = phi4_1 - phi4_old;
  double phi5_diff = phi5_1 - phi5_old;
  double phi6_diff = phi6_1 - phi6_old;
  double phi7_diff = phi7_1 - phi7_old;
  double phi_diff_plus = abs(phi1_diff) + abs(phi2_diff) + abs(phi3_diff) + abs(phi4_diff) + abs(phi5_diff) +
                         abs(phi6_diff) + abs(phi7_diff);

  // second solution------------------------------------------------------
  // reset to init values (data entered by user)
  armAng = armAng_entered;
  loop_count = 0;
  inv_kin_kuka_angle_calc(X, Y, Z, eef_phi, eef_theta, armAng);

  while (abs(phi1) >= phi1_max | abs(phi2) >= phi2_max | abs(phi3) >= phi3_max | abs(phi4) >= phi4_max |
         abs(phi5) >= phi5_max | abs(phi6) >= phi6_max | abs(phi7) >= phi7_max)
  {
    // use redundancy circle to avoid joint limits
    armAng = armAng - 0.01;
    loop_count += 0.01;

    // calc new angle values
    inv_kin_kuka_angle_calc(X, Y, Z, eef_phi, eef_theta, armAng);

    // if there is no solution
    if (loop_count > 360)
    {
      cout << "Target not reachable (impossible pose)" << endl;
      return 0;
    }
  }

  armAng_2 = armAng;  // new arm angle

  // calc difference to previous angle values [deg]
  phi1_diff = phi1_2 - phi1_old;
  phi2_diff = phi2_2 - phi2_old;
  phi3_diff = phi3_2 - phi3_old;
  phi4_diff = phi4_2 - phi4_old;
  phi5_diff = phi5_2 - phi5_old;
  phi6_diff = phi6_2 - phi6_old;
  phi7_diff = phi7_2 - phi7_old;
  double phi_diff_minus = abs(phi1_diff) + abs(phi2_diff) + abs(phi3_diff) + abs(phi4_diff) + abs(phi5_diff) +
                          abs(phi6_diff) + abs(phi7_diff);

  //---------------------------------------------------------------------

  // find solution with least difference to previous joint values
  if (phi_diff_plus < phi_diff_minus)  // first solution is better
  {
    phi1 = phi1_1;
    phi2 = phi2_1;
    phi3 = phi3_1;
    phi4 = phi4_1;
    phi5 = phi5_1;
    phi6 = phi6_1;
    phi7 = phi7_1;
    armAng = armAng_1;  // update arm angle
  }
  else  // second solution is better
  {
    phi1 = phi1_2;
    phi2 = phi2_2;
    phi3 = phi3_2;
    phi4 = phi4_2;
    phi5 = phi5_2;
    phi6 = phi6_2;
    phi7 = phi7_2;
    armAng = armAng_2;  // update arm angle
  }

  return armAng;  // return updated arm angle
}

void inv_kin_kuka_angle_calc(double X, double Y, double Z, double eef_phi, double eef_theta, double armAng)
{
  // Check if target is in workspace (circle with radius 800mm) ----------------------------------
  vec_eef = {X, Y, Z};

  // vector wrist endeffector [wf]
  vector<double> vec_wf_tmp_2 = Mat_vec_prod_Ry(eef_theta * deg2rad, vec_wf_tmp);
  vector<double> vec_wf = Mat_vec_prod_Rz(eef_phi * deg2rad, vec_wf_tmp_2);

  // vector base to wrist, global position of wrist
  pw = Vector_substraction(vec_eef, vec_wf);
  cout << "Vec_eef: [" << vec_eef.at(0) << " " << vec_eef.at(1) << " " << vec_eef.at(2) << "]" << endl;
  cout << "vec_wf: [" << vec_wf.at(0) << " " << vec_wf.at(1) << " " << vec_wf.at(2) << "]" << endl;
  cout << "Wrist position (X,Y,Z) [mm]: [" << pw.at(0) << " " << pw.at(1) << " " << pw.at(2) << "]" << endl;

  // vector shoulder to wrist
  psw = Vector_substraction(pw, p_shoulder);

  // check if length of vector psw greater then "d_se + d_ew - 1" (-1 to avoid singularity)
  if (sqrt(pow(psw.at(0), 2) + pow(psw.at(1), 2) + pow(psw.at(2), 2)) > (d_se + d_ew - 1))
  {
    cout << "target out of workspace!" << endl;
    return;  // abort calculation
  }

  // Elbow Position ------------------------------------------------------------------------------
  /*This only works for d_se = b, otherwise use formula for general triangle (Tafelwerk P.26)*/

  // length of vector shoulder to center of circle

  double psw_length = sqrt(pow(psw.at(0), 2) + pow(psw.at(1), 2) + pow(psw.at(2), 2));

  // double pc_length = sqrt(pow(pc.at(0), 2) + pow(pc.at(1), 2) + pow(pc.at(2), 2));

  double pc_length = abs(420.0 * (pow(400.0, 2) - pow(420.0, 2) - pow(psw_length, 2)) / (840.0 * psw_length));

  pc = Vector_division(psw, (psw_length / pc_length));

  // circle radius
  double alpha = asin(pc_length / d_se);
  double alpha2 = (alpha * 180) / pi;
  double R = cos(alpha) * d_se;

  // elbow position
  vector<double> Pe = Elbow_Position(armAng, R);

  // Target Center Axis --------------------------------------------------------------------------
  vector<double> vec_wf_unit =
      Vector_division(vec_wf, (sqrt(pow(vec_wf.at(0), 2) + pow(vec_wf.at(1), 2) + pow(vec_wf.at(2), 2))));
  vector<double> zero_axis = {1, 0, 0};  // arbitrary axis a, (here ArmAngle = 0)

  // find orthogonal vector to vec_wf
  //= zero_axis-(zero_axis'*pc_unit)*pc_unit;
  double u_tmp = Vector_scalar(zero_axis, vec_wf_unit);
  vector<double> vec_u_tmp_1 = Vector_multi(vec_wf_unit, u_tmp);
  vector<double> vec_u_tmp_2 = Vector_substraction(zero_axis, vec_u_tmp_1);
  double vec_u_tmp_len = sqrt(pow(vec_u_tmp_2.at(0), 2) + pow(vec_u_tmp_2.at(1), 2) + pow(vec_u_tmp_2.at(2), 2));
  vector<double> vec_u_tmp_3 = Vector_division(vec_u_tmp_2, vec_u_tmp_len);

  // rotate found orthogonal vector (around z-axis needed only)
  vector<double> vec_u = Mat_vec_prod_Rz(eef_phi * deg2rad, vec_u_tmp_3);

  vector<double> vec_v = Vector_cross(vec_u, vec_wf_unit);
  double vec_b_length = sqrt(pow(vec_v.at(0), 2) + pow(vec_v.at(1), 2) + pow(vec_v.at(2), 2));
  vector<double> Tcb = Vector_multi(Vector_division(vec_v, vec_b_length), (-1));  // unit vector

  // Angle Calculation----------------------------------------------------------------------------
  // phi 1
  phi1 = atan2(Pe.at(1), Pe.at(0));  // atan2 is only needed here!?
  if (abs(phi1) >= 2 * pi)           // map rotation to 360° circle
    phi1 = 2 * pi - phi1;
  // phi1 = round(phi1*10000)/10000; //round to 2 decimal places
  // phi1 = phi1 - pi;
  if (abs(phi1) == 0)  // avoid negative 0
    phi1 = 0;
  phi1_1 = (phi1 * 180) / pi;
  // phi1_1 = round(phi1_1*10000)/10000; //round to 2 decimal places

  // phi 2 ---------------------------------------------------------------------------------------
  phi2 = acos((Pe.at(2) - d_bs) / d_se);
  if (abs(phi2) >= 2 * pi)  // map rotation to 360° circle
    phi2 = 2 * pi - phi2;
  // phi2 = round(phi2*10000)/10000; //round to 2 decimal places
  if (abs(phi2) == 0)  // avoid negative 0
    phi2 = 0;
  phi2_2 = (phi2 * 180) / pi;
  // phi2_2 = round(phi2_2*10000)/10000; //round to 2 decimal places

  // phi 3 ---------------------------------------------------------------------------------------

  phi3 = atan2(
      (pw.at(1) * cos(phi1) - pw.at(0) * sin(phi1)),
      (d_bs * sin(phi2) - pw.at(2) * sin(phi2) + pw.at(0) * cos(phi1) * cos(phi2) + pw.at(1) * cos(phi2) * sin(phi1)));

  phi3 = phi3 - pi;  // otherwise wrist position wrong!?

  // if (abs(phi3) >= 2*pi) //map rotation to 360° circle
  //    phi3 = 2*pi - phi3;

  if (phi3 <= (-1) * 2 * pi)
    phi3 = 2 * pi + phi3;
  else if (phi3 > 2 * pi)
    phi3 = 2 * pi - phi3;

  // phi3 = round(phi3*10000)/10000; //round to 2 decimal places
  if (abs(phi3) == 0)  // avoid negative 0
    phi3 = 0;
  phi3_2 = (phi3 * 180) / pi;
  // phi3_2 = round(phi3_2*10000)/10000; //round to 2 decimal places

  // phi 4 ---------------------------------------------------------------------------------------
  phi4 = acos((pw.at(2) * cos(phi2) - d_bs * cos(phi2) + pw.at(0) * cos(phi1) * sin(phi2) +
               pw.at(1) * sin(phi1) * sin(phi2) - d_se) /
              d_ew);

  if (abs(phi4) >= 2 * pi)  // map rotation to 360° circle
    phi4 = 2 * pi - phi4;
  // phi4 = round(phi4*10000)/10000; //round to 2 decimal places
  if (abs(phi4) == 0)  // avoid negative 0
    phi4 = 0;
  phi4_2 = (phi4 * 180) / pi;
  // phi4_2 = round(phi4_2*10000)/10000; //round to 2 decimal places

  // phi 5 ---------------------------------------------------------------------------------------
  phi5 = atan2((Y * cos(phi1) * cos(phi3) - X * cos(phi3) * sin(phi1) + Z * sin(phi2) * sin(phi3) -
                d_bs * sin(phi2) * sin(phi3) - X * cos(phi1) * cos(phi2) * sin(phi3) -
                Y * cos(phi2) * sin(phi1) * sin(phi3)),
               (Z * cos(phi2) * sin(phi4) - d_se * sin(phi4) - d_bs * cos(phi2) * sin(phi4) +
                Y * cos(phi1) * cos(phi4) * sin(phi3) - Z * cos(phi3) * cos(phi4) * sin(phi2) +
                X * cos(phi1) * sin(phi2) * sin(phi4) - X * cos(phi4) * sin(phi1) * sin(phi3) +
                d_bs * cos(phi3) * cos(phi4) * sin(phi2) + Y * sin(phi1) * sin(phi2) * sin(phi4) +
                X * cos(phi1) * cos(phi2) * cos(phi3) * cos(phi4) + Y * cos(phi2) * cos(phi3) * cos(phi4) * sin(phi1)));
  if (abs(phi5) >= 2 * pi)  // map rotation to 360° circle
    phi5 = 2 * pi - phi5;
  // phi5 = round(phi5*10000)/10000; //round to 2 decimal places
  if (abs(phi5) == 0)  // avoid negative 0
    phi5 = 0;
  phi5_2 = (phi5 * 180) / pi;
  // phi5_2 = round(phi5_2*10000)/10000; //round to 2 decimal places

  // phi 6 ---------------------------------------------------------------------------------------
  phi6 = acos((Z * cos(phi2) * cos(phi4) - d_se * cos(phi4) - d_bs * cos(phi2) * cos(phi4) +
               X * cos(phi1) * cos(phi4) * sin(phi2) + Y * cos(phi4) * sin(phi1) * sin(phi2) -
               Y * cos(phi1) * sin(phi3) * sin(phi4) + Z * cos(phi3) * sin(phi2) * sin(phi4) +
               X * sin(phi1) * sin(phi3) * sin(phi4) - d_bs * cos(phi3) * sin(phi2) * sin(phi4) -
               X * cos(phi1) * cos(phi2) * cos(phi3) * sin(phi4) - Y * cos(phi2) * cos(phi3) * sin(phi1) * sin(phi4) -
               d_ew) /
              d_wf);
  if (abs(phi6) >= 2 * pi)  // map rotation to 360° circle
    phi6 = 2 * pi - phi6;
  // phi6 = round(phi6*10000)/10000; //round to 2 decimal places
  if (abs(phi6) == 0)  // avoid negative 0
    phi6 = 0;
  phi6_2 = (phi6 * 180) / pi;
  // phi6_2 = round(phi6_2*10000)/10000; //round to 2 decimal places

  // phi 7 ---------------------------------------------------------------------------------------
  phi7 =
      atan((-1) *
           (Tcb.at(1) * cos(phi1) * cos(phi3) * cos(phi6) * sin(phi5) - Tcb.at(2) * cos(phi2) * cos(phi4) * sin(phi6) +
            Tcb.at(2) * cos(phi2) * cos(phi5) * cos(phi6) * sin(phi4) -
            Tcb.at(0) * cos(phi1) * cos(phi4) * sin(phi2) * sin(phi6) -
            Tcb.at(0) * cos(phi3) * cos(phi6) * sin(phi1) * sin(phi5) -
            Tcb.at(1) * cos(phi4) * sin(phi1) * sin(phi2) * sin(phi6) +
            Tcb.at(1) * cos(phi1) * sin(phi3) * sin(phi4) * sin(phi6) -
            Tcb.at(2) * cos(phi3) * sin(phi2) * sin(phi4) * sin(phi6) +
            Tcb.at(2) * cos(phi6) * sin(phi2) * sin(phi3) * sin(phi5) -
            Tcb.at(0) * sin(phi1) * sin(phi3) * sin(phi4) * sin(phi6) +
            Tcb.at(1) * cos(phi2) * cos(phi3) * sin(phi1) * sin(phi4) * sin(phi6) -
            Tcb.at(1) * cos(phi2) * cos(phi6) * sin(phi1) * sin(phi3) * sin(phi5) +
            Tcb.at(1) * cos(phi5) * cos(phi6) * sin(phi1) * sin(phi2) * sin(phi4) +
            Tcb.at(1) * cos(phi1) * cos(phi4) * cos(phi5) * cos(phi6) * sin(phi3) -
            Tcb.at(2) * cos(phi3) * cos(phi4) * cos(phi5) * cos(phi6) * sin(phi2) +
            Tcb.at(0) * cos(phi1) * cos(phi2) * cos(phi3) * sin(phi4) * sin(phi6) -
            Tcb.at(0) * cos(phi1) * cos(phi2) * cos(phi6) * sin(phi3) * sin(phi5) +
            Tcb.at(0) * cos(phi1) * cos(phi5) * cos(phi6) * sin(phi2) * sin(phi4) -
            Tcb.at(0) * cos(phi4) * cos(phi5) * cos(phi6) * sin(phi1) * sin(phi3) +
            Tcb.at(0) * cos(phi1) * cos(phi2) * cos(phi3) * cos(phi4) * cos(phi5) * cos(phi6) +
            Tcb.at(1) * cos(phi2) * cos(phi3) * cos(phi4) * cos(phi5) * cos(phi6) * sin(phi1)) /
           (Tcb.at(1) * cos(phi1) * cos(phi3) * cos(phi5) - Tcb.at(0) * cos(phi3) * cos(phi5) * sin(phi1) +
            Tcb.at(2) * cos(phi5) * sin(phi2) * sin(phi3) - Tcb.at(2) * cos(phi2) * sin(phi4) * sin(phi5) -
            Tcb.at(0) * cos(phi1) * cos(phi2) * cos(phi5) * sin(phi3) -
            Tcb.at(1) * cos(phi2) * cos(phi5) * sin(phi1) * sin(phi3) -
            Tcb.at(1) * cos(phi1) * cos(phi4) * sin(phi3) * sin(phi5) +
            Tcb.at(2) * cos(phi3) * cos(phi4) * sin(phi2) * sin(phi5) -
            Tcb.at(0) * cos(phi1) * sin(phi2) * sin(phi4) * sin(phi5) +
            Tcb.at(0) * cos(phi4) * sin(phi1) * sin(phi3) * sin(phi5) -
            Tcb.at(1) * sin(phi1) * sin(phi2) * sin(phi4) * sin(phi5) -
            Tcb.at(0) * cos(phi1) * cos(phi2) * cos(phi3) * cos(phi4) * sin(phi5) -
            Tcb.at(1) * cos(phi2) * cos(phi3) * cos(phi4) * sin(phi1) * sin(phi5)));
  if (abs(phi7) >= 2 * pi)  // map rotation to 360° circle
    phi7 = 2 * pi - phi7;
  // phi7 = round(phi7*10000)/10000; //round to 2 decimal places
  if (abs(phi7) == 0)  // avoid negative 0
    phi7 = 0;
  phi7_2 = (phi7 * 180) / pi;
  // phi7_2 = round(phi7_2*10000)/10000; //round to 2 decimal places
}

vector<double> Mat_vec_prod_Rx(double theta, vector<double> input_vector)
{
  vector<double> tmp_vec = {0, 0, 0};
  double Rx[3][3] = {{1, 0, 0}, {cos(theta), 0, sin(theta)}, {0, -sin(theta), cos(theta)}};

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      tmp_vec.at(i) += Rx[i][j] * input_vector.at(j);
    }
  }

  return tmp_vec;
}

vector<double> Mat_vec_prod_Ry(double theta, vector<double> input_vector)
{
  vector<double> tmp_vec = {0, 0, 0};
  double Ry[3][3] = {{0, cos(theta), -sin(theta)}, {0, 1, 0}, {0, -sin(theta), cos(theta)}};

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      tmp_vec.at(i) += Ry[i][j] * input_vector.at(j);
    }
  }

  return tmp_vec;
}

vector<double> Mat_vec_prod_Rz(double theta, vector<double> input_vector)
{
  vector<double> tmp_vec = {0, 0, 0};
  double Rz[3][3] = {{cos(theta), -sin(theta), 0}, {sin(theta), cos(theta), 0}, {0, 0, 1}};

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      tmp_vec.at(i) += Rz[i][j] * input_vector.at(j);
    }
  }

  return tmp_vec;
}

vector<double> Vector_substraction(vector<double> input_vector, vector<double> input_vector2)
{
  vector<double> tmp_vec = {0, 0, 0};

  tmp_vec.at(0) = input_vector.at(0) - input_vector2.at(0);
  tmp_vec.at(1) = input_vector.at(1) - input_vector2.at(1);
  tmp_vec.at(2) = input_vector.at(2) - input_vector2.at(2);

  return tmp_vec;
}

vector<double> Vector_addition(vector<double> input_vector, vector<double> input_vector2)
{
  vector<double> tmp_vec = {0, 0, 0};

  tmp_vec.at(0) = input_vector.at(0) + input_vector2.at(0);
  tmp_vec.at(1) = input_vector.at(1) + input_vector2.at(1);
  tmp_vec.at(2) = input_vector.at(2) + input_vector2.at(2);

  return tmp_vec;
}

vector<double> Vector_division(vector<double> input_vector, double divider)
{
  vector<double> tmp_vec = {0, 0, 0};

  tmp_vec.at(0) = input_vector.at(0) / divider;
  tmp_vec.at(1) = input_vector.at(1) / divider;
  tmp_vec.at(2) = input_vector.at(2) / divider;

  return tmp_vec;
}

vector<double> Vector_multi(vector<double> input_vector, double value)
{
  vector<double> tmp_vec = {0, 0, 0};

  tmp_vec.at(0) = input_vector.at(0) * value;
  tmp_vec.at(1) = input_vector.at(1) * value;
  tmp_vec.at(2) = input_vector.at(2) * value;

  return tmp_vec;
}

vector<double> Vector_cross(vector<double> input_vector, vector<double> input_vector2)
{
  vector<double> tmp_vec = {0, 0, 0};

  tmp_vec.at(0) = input_vector.at(1) * input_vector2.at(2) - input_vector.at(2) * input_vector2.at(1);
  tmp_vec.at(1) = input_vector.at(2) * input_vector2.at(0) - input_vector.at(0) * input_vector2.at(2);
  tmp_vec.at(2) = input_vector.at(0) * input_vector2.at(1) - input_vector.at(1) * input_vector2.at(0);

  return tmp_vec;
}

double Vector_scalar(vector<double> input_vector, vector<double> input_vector2)
{
  double scalar = input_vector.at(0) * input_vector2.at(0) + input_vector.at(1) * input_vector2.at(1) +
                  input_vector.at(2) * input_vector2.at(2);

  return scalar;
}

vector<double> Elbow_Position(double armAng, double R)
{
  // vectors of circle (all orthogonal, unit vectors), see pdf: Real-Time Inverse Kinematics...P.12
  vector<double> pc_unit = Vector_division(pc, (sqrt(pow(pc.at(0), 2) + pow(pc.at(1), 2) + pow(pc.at(2), 2))));
  vector<double> zero_axis = {1, 0, 0};  // arbitrary axis a, (here ArmAngle = 0)

  //= zero_axis-(zero_axis'*pc_unit)*pc_unit;
  double u_tmp = Vector_scalar(zero_axis, pc_unit);
  vector<double> vec_u_tmp_1 = Vector_multi(pc_unit, u_tmp);
  vector<double> vec_u_tmp_2 = Vector_substraction(zero_axis, vec_u_tmp_1);
  double vec_u_tmp_len = sqrt(pow(vec_u_tmp_2.at(0), 2) + pow(vec_u_tmp_2.at(1), 2) + pow(vec_u_tmp_2.at(2), 2));
  vector<double> vec_u = Vector_division(vec_u_tmp_2, vec_u_tmp_len);

  vector<double> vec_v = Vector_cross(vec_u, pc);
  double vec_b_length = sqrt(pow(vec_v.at(0), 2) + pow(vec_v.at(1), 2) + pow(vec_v.at(2), 2));
  vec_v = Vector_division(vec_v, vec_b_length);  // unit vector

  // elbow position
  // Formula: Pe = R * (cos(armAng * deg2rad) * u + sin(armAng * deg2rad) * v) + pc + p_shoulder;
  vector<double> vec_tmp_1 = Vector_multi(vec_u, cos(armAng * deg2rad));  // cos(armAng * deg2rad) * u
  vector<double> vec_tmp_2 = Vector_multi(vec_v, sin(armAng * deg2rad));  // sin(armAng * deg2rad) * v
  vector<double> vec_tmp_3 = Vector_addition(vec_tmp_1, vec_tmp_2);       //... + ...
  vector<double> vec_tmp_4 = Vector_multi(vec_tmp_3, R);                  // R * ...
  vector<double> vec_tmp_5 = Vector_addition(vec_tmp_4, pc);              // ... + pc
  vector<double> vec_tmp_6 = Vector_addition(vec_tmp_5, p_shoulder);      //... + p_shoudler

  // cout << "Elbow position (X,Y,Z) [mm]: [" << vec_tmp_6.at(0) << " " << vec_tmp_6.at(1) << " " << vec_tmp_6.at(2) <<
  // "]"
  //      << endl;
  // cout << "-------------------------------------" << endl;

  return vec_tmp_6;
}

void init_udp()
{
  if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    die("socket");
  }

  memset((char*)&si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(PORT);

  if (inet_aton(SERVER, &si_other.sin_addr) == 0)
  {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }
}

void die(char* s)
{
  perror(s);
  exit(1);
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

string removeSpaces(string s)
{
  string newS = "";
  for (int i = 0; i < s.length(); i++)
  {
    if (s[i] != ' ')
    {
      newS += s[i];
    }
  }
  return newS;
}
