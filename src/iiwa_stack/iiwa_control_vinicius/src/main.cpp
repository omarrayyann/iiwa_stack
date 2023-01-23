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


//#include "quadprogpp/src/QuadProg++.hh"
//#include "quadprogpp/src/QuadProg++.cc"


using namespace std;
using namespace Eigen;
using namespace std::chrono;






int main(int argc, char *argv[])
{
    std::cout << "Ola, tudo bem!" << std::endl;

    IOFormat OctaveFmt(4, 0, " ", "\n", "", "", "[", "]");

    VectorXd q = VectorXd(6);
    MatrixXd M = Matrix4d::Zero();
    

    Manipulator kr5 = Manipulator::createKukaKR5();



    //Test the distance computing functions
    Box box(Utils::trn(2,0,0) * Utils::rotx(1), 0.1, 0.2, 0.3);
    Cylinder cylinder(Utils::trn(0,-1,0) * Utils::roty(1), 0.3, 0.4);
    Sphere ball(Utils::trn(0,0,-1), 0.5);

    Vector3d pointTest;
    pointTest << 0,0,0;

    DistanceStruct ds1 = GeometricPrimitives::computeDist(&box, &cylinder);
    DistanceStruct ds2 = GeometricPrimitives::computeDist(&box, &ball);
    DistanceStruct ds3 = GeometricPrimitives::computeDist(&ball, &cylinder);

    std::cout << "TEST THE DISTANCE COMPUTATIONS: "<< std::endl<< std::endl;

    std::cout <<  "Box htm: " << std::endl << box.htm.format(OctaveFmt) << std::endl<< std::endl;
    std::cout <<  "Cylinder htm: " << std::endl <<  cylinder.htm.format(OctaveFmt) << std::endl<< std::endl;
    std::cout <<  "Ball htm: " << std::endl <<  ball.htm.format(OctaveFmt) << std::endl<< std::endl;

    std::cout <<  "Projection at box: " << std::endl <<  box.projection(pointTest) << std::endl<< std::endl;
    std::cout <<  "Projection at cylinder " << std::endl <<  cylinder.projection(pointTest) << std::endl<< std::endl;
    std::cout <<  "Projection at ball: " << std::endl <<  ball.projection(pointTest) << std::endl<< std::endl;

    std::cout <<  "Distance between box and cylinder: " << ds1.distance << std::endl<< std::endl;
    std::cout <<  "Distance between box and ball: " << ds2.distance << std::endl<< std::endl;
    std::cout <<  "Distance between ball and cylinder: " << ds3.distance << std::endl<< std::endl;

    //Testing the collision of the robot with objects
    Box boxTest(Utils::trn(2,0,0) * Utils::rotx(M_PI/2), 0.1, 0.2, 0.3);
    Cylinder cylTest(Utils::trn(1,1,0) * Utils::roty(1) * Utils::rotz(1), 0.5, 0.7);
    DistanceRobotObjResult dro;
    

    q << 1, 1, 0, 0, 5, 6;
    kr5.setConfig(q);
    dro = kr5.computeDistToObj(&boxTest);

    std::cout << "TEST THE DISTANCE BETWEEN THE ROBOT AND ENVIRONMENT: "<< std::endl<< std::endl;


    std::cout << "FOR THE BOX: "<< std::endl<< std::endl;

    for(int i=0; i<dro.noElements; i++)
    {
        std::cout << "Distance between link "<< dro[i].linkNumber <<" object "<< dro[i].colObjLinkNumber<<": "<< dro[i].distance <<std::endl;
        std::cout << "Jacobian: "<< dro[i].jacDist.transpose().format(OctaveFmt) <<std::endl;
    }

    dro = kr5.computeDistToObj(&cylTest);
    std::cout << std::endl;
    std::cout << "FOR THE CYLINDER: "<< std::endl<< std::endl;

    for(int i=0; i<dro.noElements; i++)
    {
        std::cout << "Distance between link "<< dro[i].linkNumber <<" object "<< dro[i].colObjLinkNumber<<": "<< dro[i].distance <<std::endl;
        std::cout << "Jacobian: "<< dro[i].jacDist.transpose().format(OctaveFmt) <<std::endl;
    }

    //TEST AUTO COLLISION COMPUTATIONS
    DistanceRobotAutoResult dra;

    dra = kr5.computeDistAuto();


    //TEST MANY DISTANCE COMPUTATIONS

    auto start = high_resolution_clock::now();

    int N=1000;
    for(int i=0; i<N; i++)
    {
        kr5.setConfig(Utils::randVec(6));
        double x = Utils::rand(0.5,4);
        double y = Utils::rand(0.5,4);
        double z = Utils::rand(0.5,4);
        double a = Utils::rand();
        double b = Utils::rand();
        double c = Utils::rand();
        Box boxRand(Utils::trn(x,y,z) * Utils::rotx(a) * Utils::rotx(y) * Utils::rotx(z), 0.3, 0.5, 0.2);

        dro = kr5.computeDistToObj(&boxTest);

    }

    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>(stop - start);
    cout << "Time spent:" << (duration.count()/N) << endl;


    

}