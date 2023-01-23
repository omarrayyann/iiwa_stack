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

MatrixXd readMatrix(const char *filename)
{
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // Read numbers from file into buffer.
    ifstream infile;
    infile.open(filename);
    while (!infile.eof())
    {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while (!stream.eof())
            stream >> buff[cols * rows + temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
    }

    infile.close();

    rows--;

    // Populate matrix with numbers.

    MatrixXd result(rows, cols-1);

    for (int i = 0; i < rows; i++)
    {
        cout<<"LINHA "<<(i+1)<<std::endl;
        for (int j = 0; j < cols-1; j++)
        {
            result(i, j) = buff[cols * i + j];
            cout<<"L"<<(i+1)<<" C"<<(j+1)<<":  "<<result(i, j)<<std::endl;
        }
    }
    cout<<std::endl<<std::endl;           

    return result;
};

VectorXd readVector(const char *filename)
{
    MatrixXd vtemp = readMatrix(filename);

    VectorXd v = VectorXd::Zero(vtemp.rows());

    for(int i=0; i < vtemp.rows(); i++)
    {
        v[i] = vtemp(i,0);
    }

    return v;
}

int main(int argc, char *argv[])
{

    MatrixXd H, Aeq, A;
    VectorXd f, beq, b, u, uf;

    // H = MatrixXd::Zero(2, 2);
    // f = VectorXd::Zero(2);
    // Aeq = MatrixXd::Zero(0, 2);
    // beq = VectorXd::Zero(0);
    // A = MatrixXd::Zero(3, 2);
    // b = VectorXd::Zero(3);

    // H << 4, -2, -2, 4;
    // f << 6, 0;
    // Aeq << 1, 1;
    // beq << 3;
    // A << 1, 0, 0, 1, 1, 1;
    // b << 0, 0, 2;



    Aeq = MatrixXd::Zero(0, 7);
    beq = VectorXd::Zero(0);

    cout<<"H:"<<std::endl;
    H = readMatrix("H.txt");
    cout<<"A:"<<std::endl;
    A = readMatrix("A.txt");
    cout<<"f:"<<std::endl;
    f = readVector("f.txt");
    cout<<"b:"<<std::endl;
    b = readVector("b.txt");
    cout<<"uf:"<<std::endl;
    uf = readVector("u.txt");


    u = Utils::solveQP(H, f, A, b);

    double error = 0;

    cout<<"Verification 1:"<<std::endl;

    for(int i=0; i < 7; i++)
        error+= (u[i]-uf[i])*(u[i]-uf[i]);
        
    cout<<sqrt(error)<<std::endl;


    cout << "OK!";
}