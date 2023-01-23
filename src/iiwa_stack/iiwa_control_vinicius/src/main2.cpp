#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>
#include <chrono>

#include "libraries/utils.cpp"
#include "libraries/distance.cpp"
#include "libraries/robot.cpp"

using namespace std;
using namespace Eigen;
using namespace std::chrono;






int main(int argc, char *argv[])
{
    std::cout << "Ola, tudo bem!" << std::endl;

    IOFormat OctaveFmt(4, 0, " ", "\n", "", "", "[", "]");

    Manipulator iiwa = Manipulator::createKukaIIWA();

    FKResult fkres = iiwa.fk();
    Matrix4d htm;
    Vector3d rpy;

    cout<<fkres.htmDH[0].format(OctaveFmt)<<std::endl<<std::endl;


    for(int i=0; i<7; i++)
    {
        for(int j=0; j < iiwa.links[i].colObjs.size(); j++)
        {
            htm = fkres.htmDH[i] * iiwa.links[i].colObjs[j]->htm;
            
            rpy = Utils::rpy(htm);
            cout<<"("<<i<<","<<j<<") x: "<<htm(0,3)<<", y:"<<htm(1,3)<<", z:"<<htm(2,3)<<", roll: "<<rpy[0]<<", pitch: "<<rpy[1]<<", yaw: "<<rpy[2]<<std::endl<<std::endl;
            cout<<htm<<std::endl<<std::endl;
        }
    }



}