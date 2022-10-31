#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "iiwa_msgs/CartesianPose.h"
#include "iiwa_msgs/JointQuantity.h"
#include "std_msgs/Float32MultiArray.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

void getVariablesFromConsole(float& x, float& y, float& z, float& eefPhi, float& eefTheta, float& armAngle);

using namespace std;

	ros::Publisher pub;


void printState(const iiwa_msgs::CartesianPose& msg){

    ROS_INFO("Recieved a new message");

    std_msgs::Float32MultiArray messageArray;
    geometry_msgs::Point position = msg.poseStamped.pose.position;

    cout << "X: " <<  roundf(position.x * 1000) / 1000 << endl;
    cout << "Y: " <<  roundf(position.y * 1000) / 1000 << endl;
    cout << "Z: " <<  roundf(position.z * 1000) / 1000 << endl;

    messageArray.data.clear();
    messageArray.data = {(float)position.x, (float)position.y,  (float)position.z};
            
    pub.publish(messageArray);

    ROS_INFO("Sent a new message to the visualizer");

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "CartesianPoseReader");
    ros::NodeHandle n;

    pub = n.advertise<std_msgs::Float32MultiArray>("eefGoal", 100);

    ros::Subscriber sub = n.subscribe("/iiwa/state/CartesianPose", 10000, printState);

    ROS_INFO("Sent a new message to the visualizer");
    ros::spin();

}

