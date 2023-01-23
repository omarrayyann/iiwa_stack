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

    
    vector<VectorXd> points;
    VectorXd p(2);
    double theta;

    //Test closed curve
    cout<<"TEST FOR THE CLOSED CURVE"<<std::endl<<std::endl;

    for(int i=0; i<200; i++)
    {
        theta = 2*M_PI*((double) i)/200.0;
        p<<0.3*cos(theta), 0.3*sin(theta);
        points.push_back(p);
    }

    double dt = 0.01;

    VectorXd v;
    p = VectorXd::Random(2);

    for(int j=0; j < 500; j++)
    {
        v = Utils::vectorField(p, points, 3, false);
        p+= v*dt;
    }

    //Test open curve
    cout<<"TEST FOR THE OPEN CURVE"<<std::endl<<std::endl;
    points.clear();

     for(int i=0; i<200; i++)
    {
        theta = ((double) i)/200.0;
        p<<theta, theta*theta;
        points.push_back(p);
    }   

    p<<0,-0.5;
    for(int j=0; j < 500; j++)
    {
        v = Utils::vectorField(p, points, 3, true);
        p+= v*dt;
        //cout<<v.norm()<<std::endl;
        cout<<p[0]<<", "<<p[1]<<std::endl;
    }



    

}