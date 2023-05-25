#include <iostream>
#include <fstream>
#include "utils.cpp"
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <regex>
#include <boost/algorithm/string.hpp>

using namespace std;

std::vector<std::vector<double>> quaternion_rotation_matrix(const std::vector<double>& Q)
{
  double q0 = Q[0];
  double q1 = Q[1];
  double q2 = Q[2];
  double q3 = Q[3];

  double r00 = 2 * (q0 * q0 + q1 * q1) - 1;
  double r01 = 2 * (q1 * q2 - q0 * q3);
  double r02 = 2 * (q1 * q3 + q0 * q2);

  double r10 = 2 * (q1 * q2 + q0 * q3);
  double r11 = 2 * (q0 * q0 + q2 * q2) - 1;
  double r12 = 2 * (q2 * q3 - q0 * q1);

  double r20 = 2 * (q1 * q3 - q0 * q2);
  double r21 = 2 * (q2 * q3 + q0 * q1);
  double r22 = 2 * (q0 * q0 + q3 * q3) - 1;

  std::vector<std::vector<double>> rot_matrix{{r00, r01, r02}, {r10, r11, r12}, {r20, r21, r22}};

  return rot_matrix;
}

double determinant(const std::vector<std::vector<double>>& matrix)
{
  double det = 0.0;
  det = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
        matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
        matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

  return det;
}

int main()
{
  ifstream file;
  ofstream newfile;
  file.open("aurora1724.csv");
  newfile.open("aurora_modified.csv");

  string line;
  int count = 0;
  vector<double> q;
  getline(file, line);
  newfile << "ROS Seconds,ROS Nanoseconds,X,Y,Z,Rx0,Rx1,Rx2,Ry0,Ry1,Ry2,Rz0,Rz1,Rz2" << endl;
  while (getline(file, line))
  {
    if (line.empty())
    {
      continue;
    }
    stringstream ss(line);
    string str;
    vector<double> quaternion;
    vector<double> position;
    long int seconds;
    long int nanonseconds;
    int count = 0;
    while (getline(ss, str, ','))
    {
      if (count == 15)
      {
        seconds = stod(str);
      }
      if (count == 16)
      {
        nanonseconds = stod(str);
      }
      if (count <= 12 && count >= 10)
      {
        position.push_back(stod(str));
      }
      if (count <= 9 && count >= 6)
      {
        quaternion.push_back(stod(str));
      }
      count++;
    }

    if (quaternion.size() == 4)
    {
      newfile << seconds << "," << nanonseconds << "," << position[0] << "," << position[1] << "," << position[2]
              << ",";
      vector<vector<double>> rotation_matrix = quaternion_rotation_matrix(quaternion);
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          if (i == 2 && j == 2)
          {
            newfile << rotation_matrix[j][i];
          }
          else
          {
            newfile << rotation_matrix[j][i] << ",";
          }
        }
      }
    }

    newfile << endl;
  }
}