#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>
#include <ros/ros.h>
#include <ros/service.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>

#include "utils.cpp"
#include "distance.cpp"
#include "robot.cpp"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"


//Main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "iiwa_control_vinicius");
    ros::NodeHandle n;
    ros::Publisher arm_pub = n.advertise<trajectory_msgs::JointTrajectory>("/iiwa/PositionJointInterface_trajectory_controller/command",1000);
    ros::Rate loop_rate(100);
    

    ROS_INFO("Starting controller!");

    //Generate trajectory
    Manipulator iiwa = Manipulator::createKukaIIWA();

    double dt = 0.01;
    FKResult fkres = iiwa.jacGeo();
    Matrix4d htmTg = Utils::trn(0.5, 0, -0.5) * fkres.htmTool;
    TaskResult taskres;
    VectorXd qdot;
    trajectory_msgs::JointTrajectory traj;

    //Test messages
    IOFormat OctaveFmt(4, 0, " ", "\n", "", "", "[", "]");


    ROS_INFO_STREAM("Tool htm for base configuration:"<<std::endl);
    ROS_INFO_STREAM(std::endl<<fkres.htmTool.format(OctaveFmt));

    VectorXd newq(7);
    newq<<0, 0, 0, 0, 0, 3.14/2, 0;

    fkres = iiwa.fk(newq);

    ROS_INFO_STREAM("Tool htm for joint 6 moving configuration:"<<std::endl);
    ROS_INFO_STREAM(std::endl<<fkres.htmTool.format(OctaveFmt));

   
    traj.joint_names.resize(7);
    //traj.points.resize(1);

    traj.joint_names[0] = iiwa.links[0].name;
    traj.joint_names[1] = iiwa.links[1].name;
    traj.joint_names[2] = iiwa.links[2].name;
    traj.joint_names[3] = iiwa.links[3].name;
    traj.joint_names[4] = iiwa.links[4].name;
    traj.joint_names[5] = iiwa.links[5].name;
    traj.joint_names[6] = iiwa.links[6].name;

    //traj.points.resize(1000);

    int i=0;
    traj.header.stamp = ros::Time::now();
    double kp=0.5;

    while(i < 1000)
    {
        taskres = iiwa.taskFunction(htmTg);
        qdot = -kp * Utils::pinv(taskres.jacTask, 0.001)*(Utils::vecPow(taskres.task, 0, 6, 0.5));
        //qdot = -Utils::pinv(taskres.jacTask, 0.005)*(taskres.task);
        iiwa.setConfig(iiwa.q + dt*qdot);

        //Publish
        //traj.header.stamp = ros::Time::now();

        trajectory_msgs::JointTrajectoryPoint joint;

        for(int j=0; j<7; j++)
        {
            joint.positions.push_back(iiwa.q[j]);
        }

        //Begin safety test
        for(int j=0; j<7; j++)
        {
            if(qdot[j] > iiwa.qDotMax[j])
            {
                ROS_INFO_STREAM("JOINT "<<(j+1)<<" AT TIME "<<(i*dt)<<" VIOLATED UPPER VELOCITY LIMIT ");
                ROS_INFO_STREAM("VALUE "<<qdot[j]<<" RAD/S BUT MAXIMUM IS  "<<iiwa.qDotMax[j]<<std::endl);
            }
            if(qdot[j] < iiwa.qDotMin[j])
            {
                ROS_INFO_STREAM("JOINT "<<(j+1)<<" AT TIME "<<(i*dt)<<" VIOLATED LOWER VELOCITY LIMIT ");
                ROS_INFO_STREAM("VALUE "<<qdot[j]<<" RAD/S BUT MINIMUM IS  "<<iiwa.qDotMin[j]<<std::endl);
            }
            if(iiwa.q[j] > iiwa.qMax[j])
            {
                ROS_INFO_STREAM("JOINT "<<(j+1)<<" AT TIME "<<(i*dt)<<" VIOLATED UPPER POSITION LIMIT ");
                ROS_INFO_STREAM("VALUE "<<iiwa.q[j]<<" RAD BUT MAXIMUM IS  "<<iiwa.qMax[j]<<std::endl);
            }
            if(iiwa.q[j] < iiwa.qMin[j])
            {
                ROS_INFO_STREAM("JOINT "<<(j+1)<<" AT TIME "<<(i*dt)<<" VIOLATED LOWER POSITION LIMIT ");
                ROS_INFO_STREAM("VALUE "<<iiwa.q[j]<<" RAD BUT MINIMUM IS  "<<iiwa.qMin[j]<<std::endl);
            }
        }       

        //End safety test


        traj.points.push_back(joint);
        traj.points[i].time_from_start = ros::Duration(i*dt);

        //arm_pub.publish(traj);
        //ros::spinOnce();

        //ROS_INFO_STREAM("Step now: "<<i);
        //ROS_INFO_STREAM("Time now: "<<ros::Time::now());
        ROS_INFO_STREAM("Time stamp: "<<traj.points[i].time_from_start);
        //ROS_INFO_STREAM("Task: "<<taskres.task.transpose());

        //loop_rate.sleep();
        i++;
        
    }

    //Put the collision objects at the correct place

    //Come back to base configuration
    trajectory_msgs::JointTrajectoryPoint joint;

    for(int j=0; j<7; j++)
    {
        joint.positions.push_back(0);
    }
    traj.points.push_back(joint);
    traj.points[i].time_from_start = ros::Duration(i*dt+5);   

    //Publish
    if(ros::ok())
        arm_pub.publish(traj);


    //ros::spinOnce();
    //end trajectory generation




}