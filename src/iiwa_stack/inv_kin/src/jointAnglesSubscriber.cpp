#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointQuantity.h"
#include "std_msgs/Float32MultiArray.h"

void getVariablesFromConsole(float& x, float& y, float& z, float& eefPhi, float& eefTheta, float& armAngle);

using namespace std;

	    ros::Publisher pub;


void getNewJointAngles(const std_msgs::Float32MultiArray& msg){

    ROS_INFO("Recieved a new message");

    iiwa_msgs::JointPosition jointPosition;

    iiwa_msgs::JointQuantity quantity;

    quantity.a1 = (float)msg.data[0]*M_PI/180;
    quantity.a2 = (float)msg.data[1]*M_PI/180;
    quantity.a3 = (float)msg.data[2]*M_PI/180;
    quantity.a4 = (float)msg.data[3]*M_PI/180;
    quantity.a5 = (float)msg.data[4]*M_PI/180;
    quantity.a6 = (float)msg.data[5]*M_PI/180;
    quantity.a7 = (float)msg.data[6]*M_PI/180;

    jointPosition.position = quantity;

    ROS_INFO("Published successfully 123");

    pub.publish(jointPosition);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "jointAnglesSubscriber");

    ros::NodeHandle n;

    ROS_INFO("Started joint angles subscriber publisher");

    ros::Subscriber sub = n.subscribe("jointAnglesGoal", 10000, getNewJointAngles);

	pub = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 100);

    ros::spin();

}

