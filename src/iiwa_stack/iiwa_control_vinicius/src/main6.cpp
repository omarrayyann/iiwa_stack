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

    
    Box *wall1 = new Box(Utils::trn(0.3,-0.5, 0.7), 0.05, 0.6, 1.4);
    Box *wall2 = new Box(Utils::trn(0.3, 0.5, 0.7), 0.05, 0.6, 1.4);
    Box *wall3 = new Box(Utils::trn(0.3, 0, 0.25), 0.05, 0.4, 0.5);
    Box *wall4 = new Box(Utils::trn(0.3, 0, 1.15), 0.05, 0.4, 0.5);

    vector<GeometricPrimitives *> obstacles = {wall1, wall2, wall3, wall4};


    Matrix4d htmTarget = Utils::trn(0.5, -0.3, 0.7) * Utils::rotx(M_PI / 2);
    

    
    VectorXd q = VectorXd(7);

    double dt=0.03;

    ConstControlResult ccr;
    ConstControlResult *oldccr = NULL;
    string out;

    double totalTime = 0;
    double mind=1000;
    double t = 0;
    int i = 0;
    bool continueLoop = true;


    while(continueLoop)
    {
        ccr = iiwa.constControl(htmTarget, obstacles, oldccr);
        oldccr = &ccr;
        
        if(ccr.feasible)
        {
            iiwa.setConfig(iiwa.q + ccr.action*dt);
            cout<<"Task value: "<<ccr.taskResult.task.transpose().format(OctaveFmt)<<std::endl;
            
            //double d = iiwa.computeDistToObj(obs1).getClosest().distance;
            //cout<<"Distance: "<<d<<std::endl;

            //mind = min(mind,d);

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
            
            i++;
            continueLoop = i<2000 && !(ccr.taskResult.maxErrorPos < 0.002 && ccr.taskResult.maxErrorOri < 5);
                
        }
        else
        {
            cout<<"UNFEASIBLE!!!!!!!!!!!"<<std::endl;
            continueLoop = false;
        }
        t+=dt;

         
    }



    cout<<"Mean time :"<<totalTime/i<<" miliseconds"<<std::endl;
    cout<<"Minimum distance :"<<mind<<" m"<<std::endl;



}