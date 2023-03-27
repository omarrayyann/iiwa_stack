/*	program to test the firgelli actuator
	NEvangeliou, NYU Abu Dhabi
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#define RATE 0.5

//Keyboard function used to terminate the program on any keyboard button hit
int kbhit(void)
{
	struct termios oldt, newt;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	int ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)	return 1;
	return 0;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "surg_grip_pub");
	ros::NodeHandle nh;
	ROS_INFO("Start");
	ros::Publisher grip_ctrl = nh.advertise<std_msgs::Bool>("/surgrob/gripper_ctrl", 1);
	std_msgs::Bool message;
	message.data = false;
	while(!kbhit() && ros::ok())
	{
		message.data = !message.data;
		grip_ctrl.publish(message);
		ros::spinOnce();
		ros::Duration(2.0).sleep();
	}
	return 0;
}
