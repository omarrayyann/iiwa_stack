#include "utils.h"
#include "robot.h"
#include "distance.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>


using namespace std;
using namespace Eigen;



//Link

Matrix4d Link::dhMatrix(double q)
{
    Matrix4d dhMat = Matrix4d::Zero();

    double cost = this->jointType == 0 ? cos(q) : this->_cost;
    double sint = this->jointType == 0 ? sin(q) : this->_sint;
    double cosa = _cosa;
    double sina = _sina;
    double d = this->jointType == 1 ? q : this->dHd;
    double a = this->dHa;

    dhMat(0, 0) = cost;
    dhMat(0, 1) = -sint * cosa;
    dhMat(0, 2) = sint * sina;
    dhMat(0, 3) = a * cost;

    dhMat(1, 0) = sint;
    dhMat(1, 1) = cost * cosa;
    dhMat(1, 2) = -cost * sina;
    dhMat(1, 3) = a * sint;

    dhMat(2, 1) = sina;
    dhMat(2, 2) = cosa;
    dhMat(2, 3) = d;

    dhMat(3, 3) = 1;

    return dhMat;
}

Link::Link(int number, double theta, double d, double alpha, double a, int type, string strName)
{
    linkNumber = number;
    dHtheta = theta;
    dHd = d;
    dHalpha = alpha;
    dHa = a;
    jointType = type;
    name = strName;

    _cost = cos(theta);
    _sint = sin(theta);
    _cosa = cos(alpha);
    _sina = sin(alpha);
}



//DistanceLinkObj

DistanceLinkObj::DistanceLinkObj(){};

DistanceLinkObj DistanceRobotObj::operator [](int i) const
{
    return _structDistances[i];
}

DistanceLinkObj DistanceRobotObj::getClosest()
{
    DistanceLinkObj ds = this->_structDistances[0];
    double minDist = ds.distance;

    for(int i=0; i < this->noElements; i++)
    {
        if(this->_structDistances[i].distance < minDist)
        {
            minDist = this->_structDistances[i].distance;
            ds = this->_structDistances[i];
        }
    }

    return ds;
}

DistanceLinkObj* DistanceRobotObj::getItem(int indLink, int indColObjLink)
{
    for(int i=0; i < this->noElements; i++)
    {
        if(this->_structDistances[i].linkNumber == indLink && this->_structDistances[i].colObjLinkNumber == indColObjLink)
        {
            return &this->_structDistances[i];
        }
    }
    return NULL;
}

void DistanceRobotObj::addDistanceLinkObj(DistanceLinkObj dlo)
{
    this->_structDistances.push_back(dlo);
    this->noElements++;
}

//DistanceRobotObj

DistanceRobotObj::DistanceRobotObj(int noJoints)
{
    this->_noJoints = noJoints;
    this->noElements=0;
}

DistanceRobotObj::DistanceRobotObj()
{
    this->noElements=0;
}

MatrixXd DistanceRobotObj::getAllJacobians()
{
    MatrixXd jac(this->noElements, this->_noJoints);

    for(int i=0; i < this->noElements; i++)
    {
        for(int j=0; j < this->_noJoints; j++)
        {
            jac(i,j) = this->_structDistances[i].jacDist[j];
        }
    }

    return jac;

}

VectorXd DistanceRobotObj::getDistances()
{
    VectorXd dist(this->noElements);

    for(int i=0; i < this->noElements; i++)
    {
        dist[i] = this->_structDistances[i].distance;
    }

    return dist;            
}


//Manipulator

Manipulator::Manipulator(int no)
{
    noJoints = no;
    q = VectorXd::Zero(no);
    qBase = VectorXd::Zero(no);
    htmWorldToBase = Matrix4d::Identity();
    htmBaseToDH0 = Matrix4d::Identity();
    htmDHnToTool = Matrix4d::Identity();
}

Manipulator Manipulator::createKukaKR5()
{
    // Create empty manipulator
    Manipulator manip(6);

    // Create links
    double d_bs = 0.340;
    double d_se = 0.400;
    double d_ew = 0.400;
    double d_wf = 0.126;

    vector<Link> linksKuka;

    Link link1(1, 0, 0.335, -M_PI / 2, 0.075, 0,"j1");
    Link link2(2, 0, 0, 0, 0.365, 0,"j2");
    Link link3(3, 0, 0, M_PI / 2, 0.090, 0,"j3");
    Link link4(4, 0, -0.405, -M_PI / 2, 0, 0,"j4");
    Link link5(5, 0, 0, -M_PI / 2, 0, 0,"j5");
    Link link6(6, 0, 0.080, 0, 0, 0,"j6");

    linksKuka.push_back(link1);
    linksKuka.push_back(link2);
    linksKuka.push_back(link3);
    linksKuka.push_back(link4);
    linksKuka.push_back(link5);
    linksKuka.push_back(link6);

    manip.links = linksKuka;

    //Create collision objects
    FKResult fkres0 = manip.fk();

    //For the first link
    Cylinder *kr5C0_0 = new Cylinder(Utils::trn(-0.075, 0.170, 0) * Utils::roty(M_PI/2) * Utils::rotx(M_PI/2), 0.12, 0.33);
    manip.links[0].colObjs.push_back(kr5C0_0);
    //manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*kr5C0_0).htm);
    manip.links[0].htmCols.push_back((*kr5C0_0).htm);

    Cylinder *kr5C0_1 = new Cylinder(Utils::trn(0, 0, 0.030) * Utils::rotz(M_PI/2) * Utils::rotx(M_PI), 0.095, 0.30);
    manip.links[0].colObjs.push_back(kr5C0_1);
    //manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*kr5C0_1).htm);
    manip.links[0].htmCols.push_back((*kr5C0_1).htm);

    //For the second link
    Box *kr5C1_0 = new Box(Utils::trn(-0.200, 0.020, 0.12) * Utils::roty(M_PI/2), 0.1, 0.16, 0.5);
    manip.links[1].colObjs.push_back(kr5C1_0);
    //manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*kr5C1_0).htm);
    manip.links[1].htmCols.push_back((*kr5C1_0).htm);

    Cylinder *kr5C1_1 = new Cylinder(Utils::trn(0, 0, 0.040) * Utils::rotz(M_PI) * Utils::rotx(M_PI), 0.095, 0.28);
    manip.links[1].colObjs.push_back(kr5C1_1);
    //manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*kr5C1_1).htm);
    manip.links[1].htmCols.push_back((*kr5C1_1).htm);

    //For the third link
    Box *kr5C2_0 = new Box(Utils::trn(0, 0, -0.224) * Utils::rotz(-M_PI/2) * Utils::rotx(-M_PI/2), 0.143, 0.45, 0.12);
    manip.links[2].colObjs.push_back(kr5C2_0);
    //manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*kr5C2_0).htm);
    manip.links[2].htmCols.push_back((*kr5C2_0).htm);


    return manip;
}

Manipulator Manipulator::createKukaIIWA()
{
    // Create empty manipulator
    Manipulator manip(7);

    // Create links
    double d_bs = 0.340;
    double d_se = 0.400;
    double d_ew = 0.400;
    double d_wf = 0.152;

    vector<Link> linksKuka;

    Link link1(1, 0, d_bs, -M_PI / 2, 0, 0, "iiwa_joint_1");
    Link link2(2, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_2");
    Link link3(3, 0, d_se, M_PI / 2, 0, 0, "iiwa_joint_3");
    Link link4(4, 0, 0, -M_PI / 2, 0, 0, "iiwa_joint_4");
    Link link5(5, 0, d_ew, -M_PI / 2, 0, 0, "iiwa_joint_5");
    Link link6(6, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_6");
    Link link7(7, 0, d_wf, 0, 0, 0, "iiwa_joint_7");

    linksKuka.push_back(link1);
    linksKuka.push_back(link2);
    linksKuka.push_back(link3);
    linksKuka.push_back(link4);
    linksKuka.push_back(link5);
    linksKuka.push_back(link6);
    linksKuka.push_back(link7);

    manip.links = linksKuka;

    //Create collision objects
    FKResult fkres0 = manip.fk();


    //For the first link
    Cylinder *iiwaC0_0 = new Cylinder(Utils::trn(0, 0, 0.15), 0.1, 0.12);
    manip.links[0].colObjs.push_back(iiwaC0_0);
    manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_0).htm);
    manip.links[0].nameObjs.push_back("c0_0");

    Sphere *iiwaC0_1 = new Sphere(Utils::trn(0.01,0, 0.33), 0.12);
    manip.links[0].colObjs.push_back(iiwaC0_1);
    manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_1).htm);
    manip.links[0].nameObjs.push_back("c0_1");

    //For the second link
    Sphere *iiwaC1_0 = new Sphere(Utils::trn(0,0.03, 0.38), 0.1);
    manip.links[1].colObjs.push_back(iiwaC1_0);
    manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_0).htm);
    manip.links[1].nameObjs.push_back("c1_0");

    Cylinder *iiwaC1_1 = new Cylinder(Utils::trn(0,0, 0.54), 0.075, 0.125);
    manip.links[1].colObjs.push_back(iiwaC1_1);
    manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_1).htm);
    manip.links[1].nameObjs.push_back("c1_1");

    //For the third link
    Cylinder *iiwaC2_0 = new Cylinder(Utils::trn(0, 0, 0.66), 0.08, 0.1);
    manip.links[2].colObjs.push_back(iiwaC2_0);
    manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_0).htm);
    manip.links[2].nameObjs.push_back("c2_0");

    Sphere *iiwaC2_1 = new Sphere(Utils::trn(0,0.01,0.75), 0.11);
    manip.links[2].colObjs.push_back(iiwaC2_1);
    manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_1).htm);
    manip.links[2].nameObjs.push_back("c2_1");

    //For the fourth link
    Sphere *iiwaC3_0 = new Sphere(Utils::trn(0,-0.035,0.803), 0.08);
    manip.links[3].colObjs.push_back(iiwaC3_0);
    manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_0).htm);
    manip.links[3].nameObjs.push_back("c3_0");

    Cylinder *iiwaC3_1 = new Cylinder(Utils::trn(0,0, 0.935), 0.075, 0.125);
    manip.links[3].colObjs.push_back(iiwaC3_1);
    manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_1).htm);
    manip.links[3].nameObjs.push_back("c3_1");

    //For the fifth link
    Cylinder *iiwaC4_0 = new Cylinder(Utils::trn(0, 0, 1.052), 0.075, 0.11);
    manip.links[4].colObjs.push_back(iiwaC4_0);
    manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_0).htm);
    manip.links[4].nameObjs.push_back("c4_0");

    Box *iiwaC4_1 = new Box(Utils::trn(-0.12, -0.11, 1), 0.12, 0.04, 0.17);
    manip.links[4].colObjs.push_back(iiwaC4_1);
    manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_1).htm);
    manip.links[4].nameObjs.push_back("c4_1_1"); //ITS AN EXCEPTION NAME IN GAZEBO! CORRECT SOON!!!

    //For the sixth link
    Cylinder *iiwaC5_0 = new Cylinder(Utils::trn(0, 0, 1.203), 0.075, 0.21);
    manip.links[5].colObjs.push_back(iiwaC5_0);
    manip.links[5].htmCols.push_back(fkres0.htmDH[5].inverse() * (*iiwaC5_0).htm);
    manip.links[4].nameObjs.push_back("c5_0");    

    //Create joint limits (in rad)
    double dq = 0.087;

    VectorXd qMin(7);
    qMin << -2.967055+dq, -2.09435+dq, -2.967055+dq,  -2.09435+dq, -2.937055+dq, -2.09435+dq, -3.054325+dq;

    VectorXd qMax(7);
    qMax << 2.967055-dq, 2.09435-dq, 2.967055-dq, 2.09435-dq, 2.937055-dq, 2.09435-dq, 3.054325-dq;

    manip.qMin = qMin;
    manip.qMax = qMax;

    //Create joint velocity limits (in rad/s)
    VectorXd qDotMin(7);
    qDotMin << -1.482, -1.482, -1.740, -1.307, -2.268, -2.355, -2.356;

    VectorXd qDotMax(7);
    qDotMax << 1.482, 1.482, 1.740, 1.307, 2.268, 2.355, 2.356;

    manip.qDotMin = qDotMin;
    manip.qDotMax = qDotMax;

    return manip;
}

void Manipulator::setConfig(VectorXd q, Matrix4d customHtmWorldToBase)
{
    // Set the configuration
    this->q = q;

    FKResult fkres = this->fk(q, customHtmWorldToBase);

    // Update all primitive collision objects to the new pose
    for(int i=0; i < this->noJoints; i++)
    {
        for(int j=0; j < this->links[i].colObjs.size(); j++)
        {
            (*this->links[i].colObjs[j]).htm = fkres.htmDH[i] * this->links[i].htmCols[j];
        }
    }
}

FKResult Manipulator::fk(VectorXd q, Matrix4d customHtmWorldToBase)
{

    VectorXd q_int = q.rows() == 0 ? this->q : q;
    Matrix4d htmNow = customHtmWorldToBase(3, 3) == 0 ? this->htmWorldToBase : customHtmWorldToBase;
    htmNow = htmNow * this->htmBaseToDH0;

    vector<Matrix4d> listOfHtms;
    double theta, d, alpha, a;

    for (int i = 0; i < this->noJoints; i++)
    {
        htmNow = htmNow * links[i].dhMatrix(q_int[i]);
        listOfHtms.push_back(htmNow);
    }

    FKResult fkres;
    fkres.htmDH = listOfHtms;
    fkres.htmTool = htmNow * this->htmDHnToTool;

    return fkres;
}

FKResult Manipulator::jacGeo(VectorXd q, Matrix4d customHtmWorldToBase)
{
    // Compute the forward kinematics
    FKResult fkres = this->fk(q, customHtmWorldToBase);

    // Now compute the differential kinematics
    vector<MatrixXd> listOfJacs;

    int n = this->noJoints;

    for (int i = 0; i < n; i++)
    {
        MatrixXd newJac = MatrixXd::Zero(6, this->noJoints);
        listOfJacs.push_back(newJac);
    }

    Vector3d p_i, p_j_ant, z_j_ant, v, w;
    Matrix4d htmWorldToDH0 = customHtmWorldToBase(3, 3) == 0 ? this->htmWorldToBase : customHtmWorldToBase;
    htmWorldToDH0 = htmWorldToDH0 * this->htmBaseToDH0;

    for (int i = 0; i < n; i++)
    {
        p_i = fkres.htmDH[i].block<3, 1>(0, 3);
        for (int j = 0; j <= i; j++)
        {

            p_j_ant = (j == 0) ? htmWorldToDH0.block<3, 1>(0, 3) : fkres.htmDH[j - 1].block<3, 1>(0, 3);
            z_j_ant = (j == 0) ? htmWorldToDH0.block<3, 1>(0, 2) : fkres.htmDH[j - 1].block<3, 1>(0, 2);

            v = (this->links[j].jointType == 0) ? Utils::cross(z_j_ant, p_i - p_j_ant) : z_j_ant;
            w = (this->links[j].jointType == 0) ? z_j_ant : Vector3d::Zero();

            listOfJacs[i](0, j) = v(0);
            listOfJacs[i](1, j) = v(1);
            listOfJacs[i](2, j) = v(2);
            listOfJacs[i](3, j) = w(0);
            listOfJacs[i](4, j) = w(1);
            listOfJacs[i](5, j) = w(2);
        }
    }

    fkres.jacDH = listOfJacs;

    Vector3d p_ef = fkres.htmTool.block<3, 1>(0, 3);
    Vector3d p_n = fkres.htmDH[n - 1].block<3, 1>(0, 3);
    MatrixXd Jv = fkres.jacDH[n - 1].block(0, 0, 3, n);
    MatrixXd Jw = fkres.jacDH[n - 1].block(3, 0, 3, n);
    MatrixXd JvTool = Jv - Utils::S(p_ef - p_n) * Jw;
    MatrixXd jacGeoTool(6, n);
    jacGeoTool << JvTool, Jw;

    fkres.jacTool = jacGeoTool;

    return fkres;
}

TaskResult Manipulator::taskFunction(Matrix4d taskHtm, VectorXd q)
{
    int n = this->noJoints;
    FKResult fkres = this->jacGeo(q);

    Vector3d p_ef = fkres.htmTool.block<3, 1>(0, 3);
    Vector3d x_ef = fkres.htmTool.block<3, 1>(0, 0);
    Vector3d y_ef = fkres.htmTool.block<3, 1>(0, 1);
    Vector3d z_ef = fkres.htmTool.block<3, 1>(0, 2);

    Vector3d p_des = taskHtm.block<3, 1>(0, 3);
    Vector3d x_des = taskHtm.block<3, 1>(0, 0);
    Vector3d y_des = taskHtm.block<3, 1>(0, 1);
    Vector3d z_des = taskHtm.block<3, 1>(0, 2);

    Vector3d r_p = p_ef - p_des;
    Vector3d r_o;
    double e_x = 1 - x_des.transpose() * x_ef;
    double e_y = 1 - y_des.transpose() * y_ef;
    double e_z = 1 - z_des.transpose() * z_ef;
    r_o << e_x, e_y, e_z;

    VectorXd r(6, 1);
    r << r_p, r_o;

    MatrixXd jacr_p = fkres.jacTool.block(0, 0, 3, n);
    MatrixXd Jw = fkres.jacDH[n - 1].block(3, 0, 3, n);
    MatrixXd jacr_o_x = x_des.transpose() * Utils::S(x_ef) * Jw;
    MatrixXd jacr_o_y = y_des.transpose() * Utils::S(y_ef) * Jw;
    MatrixXd jacr_o_z = z_des.transpose() * Utils::S(z_ef) * Jw;

    MatrixXd jacr(6, n);
    jacr << jacr_p, jacr_o_x, jacr_o_y, jacr_o_z;

    TaskResult taskResult;

    taskResult.task = r;
    taskResult.jacTask = jacr;

    return taskResult;
}

VectorXd Manipulator::ik(Matrix4d desHtm, VectorXd q0, double pTol, double aTol, int noIterMax)
{
    int n = this->noJoints;

    VectorXd q = q0.rows() == 0 ? Utils::randVec(n, -M_PI, M_PI) : q0;

    bool continueLoop = true;
    TaskResult tr;
    double dt = 0.05;
    double error_pos, error_ori;

    int iter = 0;
    int localIter = 0;

    while (continueLoop)
    {
        tr = taskFunction(desHtm, q);
        MatrixXd H = Utils::pinv(MatrixXd::Ones(3, 6), 0.005);
        q = q - dt * (Utils::pinv(tr.jacTask, 0.005) * Utils::vecPow(tr.task, 0, 6, 0.5));

        error_pos = tr.task.block(0, 0, 3, 1).norm();
        error_ori = 0;
        for (int i = 0; i < 3; i++)
        {
            error_ori = max(error_ori, (180 / M_PI) * acos(1 - tr.task[i + 3]));
        }

        continueLoop = !(error_pos <= pTol && error_ori <= aTol) && (iter <= noIterMax);
        iter++;

        if (!(error_pos <= pTol && error_ori <= aTol) && localIter >= 500)
        {
            q = Utils::randVec(n, -M_PI, M_PI);
            localIter = 0;
        }
    }

    cout << iter << std::endl;
    return q;
}

DistanceRobotObj Manipulator::computeDist(GeometricPrimitives* obj, VectorXd q, Matrix4d customHtmWorldToBase, DistanceRobotObj *oldDistStruct, double tol, double maxDist)
{
    DistanceRobotObj drs(this->noJoints);
    
    FKResult fkres = this->jacGeo(q, customHtmWorldToBase);

    //Prepare the list of objects with the appropriate pose
    vector<vector<GeometricPrimitives *>> listObjsCopy;

    for(int i=0; i < this->noJoints; i++)
    {
        vector<GeometricPrimitives *> tempList;
        for(int j=0; j < this->links[i].colObjs.size(); j++)
        {
            GeometricPrimitives *colObjCopied = (*this->links[i].colObjs[j]).copy();
            (*colObjCopied).htm = fkres.htmDH[i] * this->links[i].htmCols[j];
            tempList.push_back(colObjCopied);
        }
        listObjsCopy.push_back(tempList);
    }

    //Compute all the distances
    Vector3d pointObj;
    DistanceLinkObj *dlo;
    

    for(int i=0; i < this->noJoints; i++)
    {
        for(int j=0; j < this->links[i].colObjs.size(); j++)
        {
            if(maxDist >= 10000 || Utils::distanceAABB(obj, this->links[i].colObjs[j]) <= maxDist)
            {
                if( oldDistStruct!=NULL )
                {
                    dlo = (*oldDistStruct).getItem(i,j);
                    if( dlo != NULL )
                        pointObj = (*dlo).witnessObj;
                    else
                        pointObj = Vector3d::Random();
                }
                else
                    pointObj = Vector3d::Random();

                DistanceStruct ds = GeometricPrimitives::computeDist(obj, this->links[i].colObjs[j], pointObj, tol);

                DistanceLinkObj dloNew;
                dloNew.colObjLinkNumber = j;
                dloNew.linkNumber = i;
                dloNew.distance = ds.distance;
                dloNew.witnessObj = ds.witnessA;
                dloNew.witnessColObjLink = ds.witnessB;
                
                MatrixXd jacWitnessColObjLink(3,this->noJoints);
                MatrixXd Jv = fkres.jacDH[i].block(0,0,3,this->noJoints);
                MatrixXd Jw = fkres.jacDH[i].block(3,0,3,this->noJoints);
                Vector3d pointCenter = fkres.htmDH[i].block<3,1>(0,3);
                jacWitnessColObjLink = Jv - Utils::S(dloNew.witnessColObjLink - pointCenter) * Jw;

                dloNew.jacDist = jacWitnessColObjLink.transpose() * (dloNew.witnessColObjLink - dloNew.witnessObj)/(0.000001+dloNew.distance);

                drs.addDistanceLinkObj(dloNew);

            }
        }
    }

    return drs;
}
