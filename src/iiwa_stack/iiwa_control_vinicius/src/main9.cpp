#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>
#include <chrono>

#include <iostream>
#include <fstream>
#include <string>

#include "utils.cpp"
#include "distance.cpp"
#include "robot.cpp"

//#include "quadprogpp/src/QuadProg++.hh"
//#include "quadprogpp/src/QuadProg++.cc"

using namespace std;
using namespace Eigen;
using namespace std::chrono;

#define MAXBUFSIZE ((int)1e4)

int main(int argc, char* argv[])
{
  Manipulator iiwa = Manipulator::createKukaIIWA();

  bool contLoop = true;
  VectorXd q = VectorXd::Zero(7);
  VectorXd qtest(7);
  Box ground(Utils::trn(0, 0, 0), 2, 2, 0.01);

  qtest << -1.41073, 1.83578, 0.201129, -1.99335, 2.80249, -1.67804, 1.89293;

  iiwa.computeDistToObj(&ground, qtest);
  double dd;

  while (contLoop)
  {
    q = VectorXd::Zero(7);
    for (int i = 0; i < 7; i++)
    {
      q[i] = Utils::rand(iiwa.qMin[i], iiwa.qMax[i]);
    }

    dd = iiwa.computeDistToObj(&ground, q).getClosest().distance;

    if (dd > 0.001)
    {
      FreeConfigResult fcr = iiwa.checkFreeConfig({}, q);
      contLoop = fcr.errorType != FreeConfigResult::ErrorType::autoCollision;
    }

    // cout << fcr.errorType;
  }

  DistanceRobotObjResult dor = iiwa.computeDistToObj(&ground, q);

  cout << q.transpose();
}