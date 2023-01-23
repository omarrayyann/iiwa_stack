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

#include "libraries/utils.cpp"
#include "libraries/distance.cpp"
#include "libraries/robot.cpp"

//#include "quadprogpp/src/QuadProg++.hh"
//#include "quadprogpp/src/QuadProg++.cc"

using namespace std;
using namespace Eigen;
using namespace std::chrono;

#define MAXBUFSIZE ((int)1e4)



string printMatrix(Matrix4d mat)
{

    string str="np.matrix([";
    str+="["+std::to_string(mat(0,0))+","+std::to_string(mat(0,1))+","+std::to_string(mat(0,2))+","+std::to_string(mat(0,3))+"],";
    str+="["+std::to_string(mat(1,0))+","+std::to_string(mat(1,1))+","+std::to_string(mat(1,2))+","+std::to_string(mat(1,3))+"],";
    str+="["+std::to_string(mat(2,0))+","+std::to_string(mat(2,1))+","+std::to_string(mat(2,2))+","+std::to_string(mat(2,3))+"],";
    str+="["+std::to_string(mat(3,0))+","+std::to_string(mat(3,1))+","+std::to_string(mat(3,2))+","+std::to_string(mat(3,3))+"]])";

    return str;

}
int main(int argc, char *argv[])
{


    IOFormat OctaveFmt(4, 0, " ", "\n", "", "", "[", "]");
    ofstream foutput; 
    ifstream finput;
    finput.open ("/home/viniciusmg/PycharmProjects/uaibot_vinicius/test/cpp_code_debug/qout.txt");
    foutput.open ("/home/viniciusmg/PycharmProjects/uaibot_vinicius/test/cpp_code_debug/qout.txt",ios::trunc); 

    Manipulator iiwa = Manipulator::createKukaIIWAUAIBot();

    Sphere *obs1 = new Sphere(Utils::trn(0.4,0,1), 0.15);

    Matrix4d htmTarget = Utils::trn(0.5,0,-0.7) * iiwa.fk().htmTool * Utils::roty(M_PI/2);
    vector<GeometricPrimitives *> obstacles = {obs1};

    
    VectorXd q = VectorXd(7);

    double dt=0.01;

    ConstControlResult ccr;
    ConstControlResult *oldccr = NULL;
    string out;

    double totalTime = 0;
    double mind=1000;
    double t = 0;

    for(int i=0; i < 2000; i++)
    {
        ccr = iiwa.constControl(htmTarget, obstacles, oldccr);
        //oldccr = &ccr;
        
        if(ccr.feasible)
        {
            iiwa.setConfig(iiwa.q + ccr.action*dt);
            cout<<"Task value: "<<ccr.taskResult.task.transpose().format(OctaveFmt)<<std::endl;
            double d = iiwa.computeDistToObj(obs1).getClosest().distance;
            cout<<"Distance: "<<d<<std::endl;

            mind = min(mind,d);

            totalTime+=ccr.milisecondsSpent;

            //Print manipulator config

            out = "manip.add_ani_frame("+std::to_string(t)+", [";
            for(int i=0; i<6; i++)
                out+=std::to_string(iiwa.q[i])+", ";

            out+=std::to_string(iiwa.q[6]) + "])\n";

            foutput<<out;

            //Print all collision objects poses
            for(int a=0; a < 7; a++)
                for(int b=0; b < iiwa.links[a].colObjs.size(); b++)
                {
                    out = "manip.links["+std::to_string(a)+"].col_objects["+std::to_string(b)+"][0].add_ani_frame("+std::to_string(t)+",";
                    out+= printMatrix(iiwa.links[a].colObjs[b]->htm)+")\n";
                    foutput<<out;
                }
            


        }
        else
        {
            cout<<"UNFEASIBLE!!!!!!!!!!!";
        }
        t+=dt;
    }


    cout<<"Mean time :"<<totalTime/2000<<" microseconds"<<std::endl;
    cout<<"Minimum distance :"<<mind<<" m"<<std::endl;



}