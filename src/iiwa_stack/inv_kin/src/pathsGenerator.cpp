#include <iostream>

#include <vector>

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

using namespace std;

using namespace std;

int main(int argc, char* argv[])

{
  vector<float> origin;

  origin.push_back(0.0);

  origin.push_back(0.0);

  origin.push_back(0.0);

  float* jointAngles;

  float xPosition;

  float yPosition;

  float zPosition;

  float eefPhiOrientation;

  float eefThetaOrientation;

  float armAngle;

  while (1)

  {
    string commandPicked = "-1";

    cout << "-------------------------------" << endl;

    cout << "1: Move EEF" << endl;

    cout << "2: Read File" << endl;

    cout << "3: Set new origin" << endl;

    cout << "4: Create File" << endl;

    cout << "5: Run txt package" << endl;

    cout << "6: Edit Stick" << endl;

    cout << "7: Control using keys" << endl;

    cout << "8: Reverse File" << endl;

    cout << "Command: ";

    cin >> commandPicked;

    while (commandPicked != "1" && commandPicked != "2" && commandPicked != "3" && commandPicked != "4" &&
           commandPicked != "5" && commandPicked != "6" && commandPicked != "7" && commandPicked != "8")
    {
      cout << "Wrong command entered, please try again!" << endl;

      cout << "1: Move EEF" << endl;

      cout << "2: Read File" << endl;

      cout << "3: Set new origin" << endl;

      cout << "4: Create File" << endl;

      cout << "5: Run txt package" << endl;

      cout << "6: Edit Stick" << endl;

      cout << "7: Control using keys" << endl;

      cout << "8: Reverse File" << endl;

      cout << "Command: ";

      cin >> commandPicked;
    }

    if (commandPicked == "8")
    {
      string fileName;

      cout << "Enter file name: " << endl;

      cin >> fileName;

      stack<string> lines;

      ifstream ordered;

      ordered.open(fileName);

      string line = "";

      if (!ordered.fail())
      {
        while (!ordered.eof())
        {
          getline(ordered, line);

          lines.push(line);
        }

        cout << "Enter new file name (reversed): ";

        cin >> fileName;

        ofstream reversedFile;

        reversedFile.open(fileName);

        while (!lines.empty())
        {
          reversedFile << lines.top() << endl;

          lines.pop();
        }
      }

      else
      {
        cout << "Could not find the file" << endl;
      }

      continue;
    }

    else if (commandPicked == "4")
    {
      cout << "Enter file name: ";

      string fileName;

      cin >> fileName;

      ofstream file;

      file.open(fileName);

      float m = 0;

      float c = 0;

      float from = 0;

      float to = 0;

      float y = 0;

      float n = 0;

      float cTo = 0;

      float startZ = 0;

      float endZ = 0;

      float frequency = 0;

      cout << "1- Non-Vertical Line" << endl
           << "2- Vertical Line" << endl
           << "3- Through Z" << endl
           << "4- XYZ" << endl
           << "command: ";

      string command;

      cin >> command;

      if (command == "1")
      {
        cout << "Enter the gradient: ";

        cin >> m;

        cout << "Enter the y-intercept: ";

        cin >> c;

        cout << "Enter z: ";

        cin >> zPosition;

        cout << "Enter the EEF Phi: ";

        cin >> eefPhiOrientation;

        cout << "Enter the EEF Theta: ";

        cin >> eefThetaOrientation;

        cout << "Enter the Arm Angle: ";

        cin >> armAngle;

        cout << "Starting X: ";

        cin >> from;

        cout << "Ending X: ";

        cin >> to;

        cout << "Enter number of points: ";

        cin >> n;

        cout << "Frames per second (frequencey): ";

        cin >> frequency;
      }

      else if (command == "2")
      {
        cout << "Enter the y-starting ";

        cin >> c;

        cout << "Enter the y-ending ";

        cin >> cTo;

        cout << "Enter z: ";

        cin >> zPosition;

        cout << "Enter the EEF Phi: ";

        cin >> eefPhiOrientation;

        cout << "Enter the EEF Theta: ";

        cin >> eefThetaOrientation;

        cout << "Enter the Arm Angle: ";

        cin >> armAngle;

        cout << "X: ";

        cin >> from;

        cout << "Enter number of points: ";

        cin >> n;

        cout << "Frames per second (frequencey): ";

        cin >> frequency;
      }

      else if (command == "3")
      {
        cout << "Enter X: ";

        cin >> xPosition;

        cout << "Enter Y: ";

        cin >> yPosition;

        cout << "Enter starting Z: ";

        cin >> startZ;

        cout << "Enter ending Z: ";

        cin >> endZ;

        cout << "Enter the EEF Phi: ";

        cin >> eefPhiOrientation;

        cout << "Enter the EEF Theta: ";

        cin >> eefThetaOrientation;

        cout << "Enter the Arm Angle: ";

        cin >> armAngle;

        cout << "Enter number of points: ";

        cin >> n;

        cout << "Frames per second (frequencey): ";

        cin >> frequency;
      }

      else if (command == "4")
      {
        float startingX, endingX, startingY, endingY, startingZ, endingZ;

        cout << "Enter Starting X: ";

        cin >> startingX;

        cout << "Enter Ending X: ";

        cin >> endingX;

        cout << "Enter Starting Y: ";

        cin >> startingY;

        cout << "Enter Ending Y: ";

        cin >> endingY;

        cout << "Enter Starting Z: ";

        cin >> startingZ;

        cout << "Enter Ending Z: ";

        cin >> endingZ;

        cout << "Enter the EEF Phi: ";

        cin >> eefPhiOrientation;

        cout << "Enter the EEF Theta: ";

        cin >> eefThetaOrientation;

        cout << "Enter the Arm Angle: ";

        cin >> armAngle;

        cout << "Enter number of points: ";

        cin >> n;

        cout << "Frames per second (frequencey): ";

        cin >> frequency;

        file << frequency << endl;

        for (int i = 0; i <= n; i++)
        {
          float currentX = startingX + (((endingX - startingX) / (float)n) * (float)i);

          float currentY = startingY + (((endingY - startingY) / (float)n) * (float)i);

          float currentZ = startingZ + (((endingZ - startingZ) / (float)n) * (float)i);

          file << currentX << ", " << currentY << ", " << currentZ << ", " << eefPhiOrientation << ", "
               << eefThetaOrientation << ", " << armAngle << endl;
        }
      }

      if (command != "4")
      {
        file << frequency << endl;
      }

      if (command == "2")
      {
        for (int i = 0; i <= n; i++)
        {
          y = ((cTo - c) / n) * i + c;

          if (i == n)
          {
            file << from << ", " << y << ", " << zPosition << ", " << eefPhiOrientation << ", " << eefThetaOrientation
                 << ", " << armAngle;
          }

          else
          {
            file << from << ", " << y << ", " << zPosition << ", " << eefPhiOrientation << ", " << eefThetaOrientation
                 << ", " << armAngle << endl;
          }
        }
      }

      else if (command == "1")
      {
        cout << "from: " << from << endl;

        cout << "to: " << to << endl;

        for (int j = 0; j < n; j++)
        {
          float x = from + (((to - from) / n) * j);

          y = m * x + c;

          file << x << ", " << y << ", " << zPosition << ", " << eefPhiOrientation << ", " << eefThetaOrientation
               << ", " << armAngle << endl;
        }

        y = m * to + c;

        file << to << ", " << y << ", " << zPosition << ", " << eefPhiOrientation << ", " << eefThetaOrientation << ", "
             << armAngle;
      }

      else if (command == "3")
      {
        for (float z = startZ; (int)z != (int)endZ; z += ((endZ - startZ) / (float)n))
        {
          file << xPosition << ", " << yPosition << ", " << z << ", " << eefPhiOrientation << ", "
               << eefThetaOrientation << ", " << armAngle << endl;
        }

        file << xPosition << ", " << yPosition << ", " << endZ << ", " << eefPhiOrientation << ", "
             << eefThetaOrientation << ", " << armAngle;
      }
    }

    else
    {
      cout << "File does not exist!" << endl;
    }
  }
}