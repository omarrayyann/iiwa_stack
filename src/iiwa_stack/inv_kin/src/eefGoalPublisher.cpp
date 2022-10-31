#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;

void getVariablesFromConsole(float& x, float& y, float& z, float& eefPhi, float& eefTheta, float& armAngle);

int main(int argc, char **argv)
{
    
	ros::init(argc, argv, "eefGoalPublisher");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("eefGoalPosition", 100);

	while (ros::ok())
	{
		std_msgs::Float32MultiArray messageArray;

		messageArray.data.clear();


    float xPosition;
    float yPosition;
    float zPosition;
    float eefPhiOrientation;
    float eefThetaOrientation;
    float armAngle;
    
    getVariablesFromConsole(xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle);

		messageArray.data = {xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle};

		pub.publish(messageArray);

		ROS_INFO("Published new EEF position goal");

		sleep(1);
	}

}



void getVariablesFromConsole(float& x, float& y, float& z, float& eefPhi, float& eefTheta, float& armAngle){
    
    cout << "X position: ";
    while(!(cin >> x)){
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Value entered incorrect, please try again!" << endl;
        cout << "X position: ";
    }
    
    cout << "Y position: ";
    while(!(cin >> y)){
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Value entered incorrect, please try again!" << endl;
        cout << "Y position: ";
    }
    
    cout << "Z position: ";
    while(!(cin >> z)){
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Value entered incorrect, please try again!" << endl;
        cout << "Z position: ";
    }
    
    cout << "End Effecetor Phi orientation angle (degrees): ";
    while(!(cin >> eefPhi)){
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Value entered incorrect, please try again!" << endl;
        cout << "End Effecetor Phi orientation angle (degrees): ";
    }
    
    cout << "End Effecetor Theta orientation angle (degrees): ";
    while(!(cin >> eefTheta)){
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Value entered incorrect, please try again!" << endl;
        cout << "End Effecetor Theta orientation angle (degrees): ";
    }
    
    cout << "Arm Angle: ";
    while(!(cin >> armAngle)){
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Value entered incorrect, please try again!" << endl;
        cout << "Arm Angle: ";
    }
    
    
}
