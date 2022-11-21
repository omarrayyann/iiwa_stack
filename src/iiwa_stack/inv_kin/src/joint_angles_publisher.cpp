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

//#include <QCoreApplication>
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
#include "inverse_kinematics.cpp"

using namespace std;

// command line (linux): nc -lvu <port>

//----------------------------------------------------------
float stickLength = 0;
// vector wrist endeffector [wf]

vector<string> split(string s, string delimiter);
string removeSpaces(string s);

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
                   float zPosition, float eefPhiOrientation, float eefThetaOrientation, float armAngle,
                   float* jointAngles)
{
  std_msgs::Float32MultiArray messageArray;

  messageArray.data = {xPosition, yPosition, zPosition};

  xyzPublisher.publish(messageArray);

  // fixForStick(xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation);
  if (inv_kin_kuka(xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle, jointAngles))
  {
    // safety

    if (abs(jointAngles[0]) > 169 || abs(jointAngles[1]) > 119 || (jointAngles[2]) > 169 || abs(jointAngles[3]) > 119 ||
        abs(jointAngles[4]) > 169 || abs(jointAngles[5]) > 119 || abs(jointAngles[6]) > 174)
    {
      ROS_INFO("DID NOT SEND ANGLES - SAFETY");

      return false;
    }

    messageArray.data.clear();
    messageArray.data = {jointAngles[0], jointAngles[1], jointAngles[2], jointAngles[3],
                         jointAngles[4], jointAngles[5], jointAngles[6]};

    jointAnglesPublisher.publish(messageArray);
    messageArray.data.clear();

    delete[] jointAngles;
    return true;
  }
  else
  {
    ROS_INFO("Could not come up with joint angles required");

    return false;
  }
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
  cout << "Command: ";
}

float x_before, y_before, z_before;

int main(int argc, char* argv[])
{
  vector<float> origin;
  origin.push_back(0.0);
  origin.push_back(0.0);
  origin.push_back(0.0);

  init_udp();

  ros::init(argc, argv, "jointAnglesPublisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("jointAnglesGoal", 100);
  ros::Publisher pub2 = n.advertise<std_msgs::Float32MultiArray>("eefGoal", 100);

  float* jointAngles;
  float xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle;

  bool run = true;

  while (ros::ok() && run)
  {
    string commandPicked = "-1";
    printCommands();
    cin >> commandPicked;

    while (commandPicked != "1" && commandPicked != "2" && commandPicked != "3" && commandPicked != "4" &&
           commandPicked != "5" && commandPicked != "6" && commandPicked != "7" && commandPicked != "8")
    {
      printCommands();
      cin >> commandPicked;
    }

    if (commandPicked == "1")
    {
      getVariablesFromConsole(xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle);

      jointAngles = new float[7];

      publishNewEEF(pub, pub2, xPosition + origin.at(0), yPosition + origin.at(1), zPosition + origin.at(2),
                    eefPhiOrientation, eefThetaOrientation, armAngle, jointAngles);
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
        getline(inputFile, line);
        vector<string> configLine = split(line, ",");
        int numberOfPoints = stof(configLine.at(0));
        int currentPoint = 1;
        int currentRate = 5;
        int maxRate = stof(configLine.at(1));
        ros::Rate rate = ros::Rate(currentRate);
        while (!inputFile.eof())
        {
          // if (currentPoint <= 30)
          // {
          //   currentRate = maxRate * currentPoint / 30;
          // }

          // if (currentPoint > (numberOfPoints - 30) && currentPoint < 150)
          // {
          //   currentRate = maxRate - int(maxRate * float(currentPoint - (numberOfPoints - 30)) / 30.0);
          // }
          // cout << "rate: " << currentRate << endl;

          // rate = ros::Rate(currentRate);

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

          x_before = xPosition;
          y_before = yPosition;
          z_before = zPosition;

          publishNewEEF(pub, pub2, xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle,
                        jointAngles);
          currentPoint++;
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
            ros::Rate rate = ros::Rate(stof(line));
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

              if (line[1] == ':')
              {
                cout << "line: " << line << endl;
                int jointAngleNumber = stoi(to_string(line[0] - '0'));
                line = line.erase(0, 2);
                jointAngles = new float[7];
                jointAngles[0] = (float)phi1_old * 180 / M_PI;
                jointAngles[1] = (float)phi2_old * 180 / M_PI;
                jointAngles[2] = (float)phi3_old * 180 / M_PI;
                jointAngles[3] = (float)phi4_old * 180 / M_PI;
                jointAngles[4] = (float)phi5_old * 180 / M_PI;
                jointAngles[5] = (float)phi6_old * 180 / M_PI;
                jointAngles[6] = (float)phi7_old * 180 / M_PI;
                jointAngles[jointAngleNumber - 1] = stof(line);
                cout << "joint Angle Number: " << jointAngleNumber << endl
                     << "joint Angle 0: " << jointAngles[0] << endl
                     << "new joint Angle: " << stof(line) << endl;

                messageArray.data.clear();
                messageArray.data = {jointAngles[0], jointAngles[1], jointAngles[2], jointAngles[3],
                                     jointAngles[4], jointAngles[5], jointAngles[6]};

                delete[] jointAngles;

                pub.publish(messageArray);

                messageArray.data.clear();
                messageArray.data = {x_before, y_before, z_before};

                pub2.publish(messageArray);

                ROS_INFO("Published new joint angles required");
              }
              else
              {
                publishNewEEF(pub, pub2, xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation,
                              armAngle, jointAngles);
                rate.sleep();
              }
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
                      eefPhiOrientation, eefThetaOrientation, armAngle, jointAngles);
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
