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
#include <string>

void getVariablesFromConsole(float& x, float& y, float& z, float& eefPhi, float& eefTheta, float& armAngle);

using namespace std;

	    ros::Publisher pub;


void printState(const iiwa_msgs::JointPosition& msg){

    ROS_INFO("Recieved a new message");

    iiwa_msgs::JointQuantity quantity = msg.position;

    cout << "Joint 1: " <<  to_string(roundf(quantity.a1* 1000)) << endl;
    cout << "Joint 2: " <<  to_string(roundf(quantity.a2* 1000)) << endl;
    cout << "Joint 3: " <<  to_string(roundf(quantity.a3* 1000)) << endl;
    cout << "Joint 4: " <<  to_string(roundf(quantity.a4* 1000)) << endl;
    cout << "Joint 5: " <<  to_string(roundf(quantity.a5* 1000)) << endl;
    cout << "Joint 6: " <<  to_string(roundf(quantity.a6* 1000)) << endl;
    cout << "Joint 7: " <<  to_string(roundf(quantity.a7* 1000)) << endl;



}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "jointAngleStateSubscriber");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/iiwa/state/JointPosition", 10000, printState);

    ros::spin();

}

