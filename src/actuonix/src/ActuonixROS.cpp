/*	program to test the firgelli actuator
	R. Treffers Starman Systems, LLC
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "firgelli.h"
#include <fcntl.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#define RATE 2
#define OPEN_MAX 60
#define CLOSE_MIN 15

Firgelli firgelli;

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

void gripper_ctrl(const std_msgs::BoolConstPtr in_){
	if(in_->data==true) {ROS_INFO("Gripper closing"); firgelli.WriteCode(0x20,CLOSE_MIN);}
	else {ROS_INFO("Gripper opening"); firgelli.WriteCode(0x20,OPEN_MAX);}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "surg_grip");
	ros::NodeHandle nh;
	ROS_INFO("Start");
	ros::Subscriber grip_ctrl = nh.subscribe<std_msgs::Bool>("/surgrob/gripper_ctrl", 1, gripper_ctrl);
	// the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(RATE);
  
	firgelli.SetDebug(0);
	firgelli.Open(1);
	ros::Duration(1.0).sleep();
	//firgelli.Info();
	
	
	ROS_INFO("LAC ready");
	firgelli.WriteCode(0x01,10); // Set deadzone thres
	firgelli.WriteCode(0x02,5); // Set retract thres
	firgelli.WriteCode(0x03,100); // Set extend thres
	firgelli.WriteCode(0x0C,10); // Set K_p
	firgelli.WriteCode(0x0D,1); // Set K_d
	ROS_INFO("LAC looping");
	while(!kbhit() && ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		// 	int value=firgelli.WriteCode(0x20,60);
		// 	printf("value=%d\n",value);
		// 	sleep(1);
		// 	int value2=firgelli.WriteCode(0x20,20);
		// 	printf("value=%d\n",value2);
		// 	sleep(1);
	}
	ROS_INFO("LAC exit");
	firgelli.WriteCode(0x20,CLOSE_MIN);
	return 0;
}
