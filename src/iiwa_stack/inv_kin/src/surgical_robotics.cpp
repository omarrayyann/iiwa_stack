
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <string>
#include <math.h>
#include <vector>
#include <fstream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <sstream>
#include <stack>
#include "iiwa_msgs/CartesianPose.h"
#include "iiwa_msgs/JointQuantity.h"
#include "std_msgs/Float32MultiArray.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointQuantity.h"
#include "std_msgs/Float32MultiArray.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>
#include <conio.h>
#include "robot.cpp"
#include "utils.h"
#include "distance.cpp"

using namespace std;

// Variable Declarations
#define pi 3.14159265
double deg2rad = pi / 180.0;
bool axis_limit_violation = false;
// vectors
vector<double> ex = {1, 0, 0};
vector<double> ey = {0, 1, 0};
vector<double> ez = {0, 0, 1};
vector<double> pw = {0, 0, 0};
vector<double> psw = {0, 0, 0};
vector<double> pc = {0, 0, 0};
// angles [deg]
double phi1 = 0;
double phi1_1 = 0;
double phi1_2 = 0;
double phi2 = 0;
double phi2_1 = 0;
double phi2_2 = 0;
double phi3 = 0;
double phi3_1 = 0;
double phi3_2 = 0;
double phi4 = 0;
double phi4_1 = 0;
double phi4_2 = 0;
double phi5 = 0;
double phi5_1 = 0;
double phi5_2 = 0;
double phi6 = 0;
double phi6_1 = 0;
double phi6_2 = 0;
double phi7 = 0;
double phi7_1 = 0;
double phi7_2 = 0;
// old angles [rad]
double phi1_old = 0;
double phi2_old = 0;
double phi3_old = 0;
double phi4_old = 0;
double phi5_old = 0;
double phi6_old = 0;
double phi7_old = 0;
// angles constraints
double phi1_max = 2.875;
double phi2_max = 2.0;
double phi3_max = 2.875;
double phi4_max = 2.0;
double phi5_max = 2.875;
double phi6_max = 2.0;
double phi7_max = 2.96;
float stickLength = 0;
double armAng = 0;
// Parameters ----------------------------------------------------------------
// limb length [mm], total: 1330mm (with zimmer group gripper), else: 1266mm
double d_bs = 360;
double d_se = 420;
double d_ew = 400;
// double d_wf = 152;  // without gripper (x,y,z output of smartPad is without gripper)
double d_wf = 639;                         // with 487 gripper (x,y,z output of smartPad is without gripper)
vector<double> p_shoulder = {0, 0, d_bs};  // position of shoulder
vector<double> vec_eef = {0, 0, 0};        // vector base (x=y=z=0) to tcp tip
vector<double> vec_wf_tmp = {0, 0, d_wf};  // vector wrist endeffector [wf]
vector<float> testing = {0, 0, 0};
bool run = true;
bool reachedSentPoint = false;
bool started = false;
vector<float> sentPoint;
vector<float> origin;
vector<vector<float>> points;
int pointIndex = 0;
ros::Publisher pub;
ros::Publisher pub2;

// Functions Prototypes
bool inv_kin_kuka(double X, double Y, double Z, double eef_phi, double eef_theta, double armAng_in, float* jointAngles);
void inv_kin_kuka_angle_calc(double X, double Y, double Z, double eef_phi, double eef_theta, double armAng);
double adapt_elbow_position(double X, double Y, double Z, double eef_phi, double eef_theta, double armAng);
vector<double> Mat_vec_prod_Rx(double theta, vector<double> input_vector);
vector<double> Mat_vec_prod_Ry(double theta, vector<double> input_vector);
vector<double> Mat_vec_prod_Rz(double theta, vector<double> input_vector);
vector<double> Vector_substraction(vector<double> input_vector, vector<double> input_vector2);
vector<double> Vector_addition(vector<double> input_vector, vector<double> input_vector2);
vector<double> Vector_division(vector<double> input_vector, double divider);
vector<double> Vector_multi(vector<double> input_vector, double value);
vector<double> Vector_cross(vector<double> input_vector, vector<double> input_vector2);
double Vector_scalar(vector<double> input_vector, vector<double> input_vector2);
vector<double> Elbow_Position(double armAng, double R);
vector<float> startingPosition;
vector<float> touch_origin;
float xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle;
float* jointAngles;
vector<float> angles;
float currentX = 0.0;
float currentY = 0.0;
float currentZ = 0.0;
float currentPhi = 0.0;
float currentTheta = 0.0;
float final_theta;
float final_phi;

using namespace std;

bool publishNewEEF(ros::Publisher jointAnglesPublisher, ros::Publisher xyzPublisher, float xPosition, float yPosition,
                   float zPosition, float eefPhiOrientation, float eefThetaOrientation, float armAngle)
{
  std_msgs::Float32MultiArray messageArray;
  float* jointAngles = new float[7];

  if (inv_kin_kuka(xPosition, yPosition, zPosition, eefPhiOrientation, eefThetaOrientation, armAngle, jointAngles))
  {
    // safety

    if (abs(jointAngles[0]) > 165 || abs(jointAngles[1]) > 115 || abs(jointAngles[2]) > 165 ||
        abs(jointAngles[3]) > 115 || abs(jointAngles[4]) > 165 || abs(jointAngles[5]) > 115 ||
        abs(jointAngles[6]) > 170)
    {
      ROS_INFO("DID NOT SEND ANGLES - SAFETY");

      return false;
    }

    messageArray.data.clear();

    iiwa_msgs::JointPosition jointPosition;

    iiwa_msgs::JointQuantity quantity;

    quantity.a1 = jointAngles[0] * M_PI / 180;
    quantity.a2 = jointAngles[1] * M_PI / 180;
    quantity.a3 = jointAngles[2] * M_PI / 180;
    quantity.a4 = jointAngles[3] * M_PI / 180;
    quantity.a5 = jointAngles[4] * M_PI / 180;
    quantity.a6 = jointAngles[5] * M_PI / 180;
    quantity.a7 = jointAngles[6] * M_PI / 180;

    Manipulator manip = Manipulator::createKukaIIWA();
    VectorXd q(7);
    q << jointAngles[0] * M_PI / 180, jointAngles[1] * M_PI / 180, jointAngles[2] * M_PI / 180,
        jointAngles[3] * M_PI / 180, jointAngles[4] * M_PI / 180, jointAngles[5] * M_PI / 180,
        jointAngles[6] * M_PI / 180;
    testing[0] = (float)manip.fk(q).htmTool(0, 2);
    testing[1] = (float)manip.fk(q).htmTool(1, 2);
    testing[2] = (float)manip.fk(q).htmTool(2, 2);

    jointPosition.position = quantity;

    jointAnglesPublisher.publish(jointPosition);
    messageArray.data.clear();
    messageArray.data = {xPosition, yPosition, zPosition};

    xyzPublisher.publish(messageArray);

    delete[] jointAngles;
    return true;
  }
  else
  {
    ROS_INFO("Could not come up with joint angles required");
    delete[] jointAngles;

    return false;
  }
}

void moved_kuka(const iiwa_msgs::JointPosition msg)
{
  angles.clear();
  angles.push_back(0);
  angles.push_back(msg.position.a1);
  angles.push_back(msg.position.a2);
  angles.push_back(msg.position.a3);
  angles.push_back(msg.position.a4);
  angles.push_back(msg.position.a5);
  angles.push_back(msg.position.a6);
  angles.push_back(msg.position.a7);

  Manipulator manip = Manipulator::createKukaIIWA();
  VectorXd q(7);
  q << msg.position.a1, msg.position.a2, msg.position.a3, msg.position.a4, msg.position.a5, msg.position.a6,
      msg.position.a7;

  FKResult fkResult = manip.fk(q);

  currentX = manip.fk(q).htmTool(0, 3);
  currentY = manip.fk(q).htmTool(1, 3);
  currentZ = manip.fk(q).htmTool(2, 3);

  float x_diff = 1000 * (manip.fk(q).htmTool(0, 3) - manip.fk(q).htmDH[4](0, 3)) / d_wf;
  float y_diff = 1000 * (manip.fk(q).htmTool(1, 3) - manip.fk(q).htmDH[4](1, 3)) / d_wf;
  float z_diff = 1000 * (manip.fk(q).htmTool(2, 3) - manip.fk(q).htmDH[4](2, 3)) / d_wf;

  currentPhi = (atan2(y_diff, x_diff)) * 180 / M_PI;

  currentTheta = acos(z_diff) * 180 / M_PI;

  vector<double> position_elbow_2 = {manip.fk(q).htmDH[2](0, 3), manip.fk(q).htmDH[2](1, 3),
                                     manip.fk(q).htmDH[2](2, 3)};
  vector<double> position_shoulder_2 = {manip.fk(q).htmDH[0](0, 3), manip.fk(q).htmDH[0](1, 3),
                                        manip.fk(q).htmDH[0](2, 3)};

  // psw
  vector<double> shoulder_to_wrist = {manip.fk(q).htmDH[4](0, 3), manip.fk(q).htmDH[4](1, 3),
                                      manip.fk(q).htmDH[4](2, 3)};

  double psw_length_2 =
      sqrt(pow(shoulder_to_wrist.at(0), 2) + pow(shoulder_to_wrist.at(1), 2) + pow(shoulder_to_wrist.at(2), 2));

  double pc_length_2 = abs(420.0 * (pow(400.0, 2) - pow(420.0, 2) - pow(psw_length_2, 2)) / (840.0 * psw_length_2));

  vector<double> pc_2 = Vector_division(psw, (psw_length_2 / pc_length_2));

  double alpha_2 = asin(pc_length_2 / d_se);
  double alpha2_2 = (alpha_2 * 180) / pi;
  double R_2 = cos(alpha_2) * d_se;

  vector<double> pc_unit_2 =
      Vector_division(pc_2, (sqrt(pow(pc_2.at(0), 2) + pow(pc_2.at(1), 2) + pow(pc_2.at(2), 2))));
  vector<double> zero_axis_2 = {1, 0, 0};  // arbitrary axis a, (here ArmAngle = 0)

  double u_tmp_2 = Vector_scalar(zero_axis_2, pc_unit_2);
  vector<double> vec_u_tmp_1_2 = Vector_multi(pc_unit_2, u_tmp_2);
  vector<double> vec_u_tmp_2_2 = Vector_substraction(zero_axis_2, vec_u_tmp_1_2);
  double vec_u_tmp_len_2 =
      sqrt(pow(vec_u_tmp_2_2.at(0), 2) + pow(vec_u_tmp_2_2.at(1), 2) + pow(vec_u_tmp_2_2.at(2), 2));
  vector<double> vec_u_2 = Vector_division(vec_u_tmp_2_2, vec_u_tmp_len_2);

  vector<double> vec_v_2 = Vector_cross(vec_u_2, pc);
  double vec_b_length_2 = sqrt(pow(vec_v_2.at(0), 2) + pow(vec_v_2.at(1), 2) + pow(vec_v_2.at(2), 2));
  vec_v_2 = Vector_division(vec_v_2, vec_b_length_2);  // unit vector

  float x_first = position_elbow_2.at(0) - pc_2.at(0) - position_shoulder_2.at(0);
  float y_first = position_elbow_2.at(1) - pc_2.at(1) - position_shoulder_2.at(1);
  float z_first = position_elbow_2.at(2) - pc_2.at(2) - position_shoulder_2.at(2);

  float arm_angle_computed =
      acos(x_first / pow(pow(vec_v_2.at(0), 2) + pow(vec_u_2.at(0), 2), 0.5)) + atan2(vec_v_2.at(0) / vec_u_2.at(0));

  cout << "ARM ANGLE COMPUTED: " << arm_angle_computed * 180 / M_PI << endl;
  ;
}

void moved_touch_joints(const sensor_msgs::JointState msg)
{
  vector<float> joints = {0,
                          msg.position[0],
                          msg.position[1],
                          msg.position[2] - M_PI / 2,
                          msg.position[3],
                          msg.position[4],
                          msg.position[5]};

  Manipulator manip = Manipulator::createGeoTouch();
  VectorXd q(5);
  q << msg.position[0], msg.position[1], msg.position[2], msg.position[3], -1 * msg.position[4];

  VectorXd unit_vector(4);
  unit_vector << (float)manip.fk(q).htmTool(0, 0), (float)manip.fk(q).htmTool(1, 0), (float)manip.fk(q).htmTool(2, 0),
      1;

  unit_vector *= -1;

  VectorXd corrected_unit_vector = Utils::rotz(-M_PI / 2) * Utils::rotx(M_PI / 2) * unit_vector;

  float x_diff = corrected_unit_vector[0];
  float y_diff = corrected_unit_vector[1];
  float z_diff = corrected_unit_vector[2];

  final_phi = (M_PI + atan2(y_diff, x_diff)) * 180 / M_PI;
  final_theta = acos(z_diff) * 180 / M_PI;
}

void moved_touch(const geometry_msgs::PoseStamped msg)
{
  float x_pos = msg.pose.position.x;
  float y_pos = msg.pose.position.y;
  float z_pos = msg.pose.position.z;

  if (touch_origin.empty())
  {
    touch_origin.push_back(x_pos);
    touch_origin.push_back(y_pos);
    touch_origin.push_back(z_pos);
    touch_origin.push_back(final_phi);
    touch_origin.push_back(final_theta);
    cout << "   Touch 3D Starting Position Set as:" << endl;
    cout << "   X: " << 1000 * touch_origin.at(0) << endl;
    cout << "   Y: " << 1000 * touch_origin.at(1) << endl;
    cout << "   Z: " << 1000 * touch_origin.at(2) << endl;
    cout << "   Theta: " << final_theta << endl;
    cout << "   Phi: " << final_phi << endl;
    cout << endl << "   STARTED MOTION SUCCESSFULLY" << endl;
  }
  else
  {
    float x_dif = 1000 * (x_pos - touch_origin.at(0)) / 5;
    float y_dif = 1000 * (y_pos - touch_origin.at(1)) / 5;
    float z_dif = 1000 * (z_pos - touch_origin.at(2)) / 5;

    x_dif /= 5;
    y_dif /= 5;
    z_dif /= 5;

    // publishNewEEF(pub, pub2, y_dif + startingPosition.at(0), -x_dif + startingPosition.at(1),
    //               z_dif + startingPosition.at(2), final_phi, final_theta, armAngle);
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "surgical_roboitcs");
  ros::NodeHandle n;
  pub = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 100);
  pub2 = n.advertise<std_msgs::Float32MultiArray>("eefGoals", 100);

  origin.push_back(450.0);
  origin.push_back(0.0);
  origin.push_back(170.0);

  clrscr();

  cout << "    ____                  _           _   ____       _           _   _" << endl;
  cout << "   / ___| _   _ _ __ __ _(_) ___ __ _| | |  _ \\ ___ | |__   ___ | |_(_) ___ ___" << endl;
  cout << "   \\___ \\| | | | '__/ _` | |/ __/ _` | | | |_) / _ \\| '_ \\ / _ \\| __| |/ __/ __| " << endl;
  cout << "    ___) | |_| | | | (_| | | (_| (_| | | |  _ < (_) | |_) | (_) | |_| | (__\\__ \\ " << endl;
  cout << "   |____/ \\__,_|_|  \\__, |_|\\___\\__,_|_| |_| \\_\\___/|_.__/ \\___/ \\__|_|\\___|___/ " << endl;
  cout << "                    |___/                                    1.0 by omar rayyan" << endl;
  cout << endl << endl << endl;
  cout << "   Would you like to set KUKA's origin's position physically (1) or manually (2)?" << endl;
  cout << "   Choice: ";
  string choice;
  cin >> choice;
  ros::Subscriber sub2;
  ros::Subscriber sub4;

  while (1)
  {
    if (choice == "1")
    {
      cout << "   Move the KUKA robot to the initial position, hit enter to confirm position";
      sub2 = n.subscribe("/iiwa/state/JointPosition", 1000, moved_kuka);
      char c = getch();
      ros::spinOnce();
      startingPosition.push_back(currentX);
      startingPosition.push_back(currentY);
      startingPosition.push_back(currentZ);
      startingPosition.push_back(currentPhi);
      startingPosition.push_back(currentTheta);
      break;
    }
    else if (choice == "2")
    {
      cout << "X: ";
      cin >> currentX;
      cout << "Y: ";
      cin >> currentY;
      cout << "Z: ";
      cin >> currentZ;
      cout << "Phi: ";
      cin >> currentPhi;
      cout << "Theta: ";
      cin >> currentTheta;
      cout << "Arm Angle: ";
      cin >> armAngle;
      startingPosition.push_back(currentX);
      startingPosition.push_back(currentY);
      startingPosition.push_back(currentZ);
      startingPosition.push_back(currentPhi);
      startingPosition.push_back(currentTheta);
      startingPosition.push_back(armAngle);

      break;
    }
    else
    {
      cout << "Unexpected Input, Try again." << endl;
      cout << "Would you like to set KUKA's origin's position physically (1) or manually (2)?" << endl;
      cout << "Choice: ";
      cin >> choice;
    }
  }
  sub4 = n.subscribe("/phantom/joint_states", 1000, moved_touch_joints);

  cout << endl << endl << "   KUKA Starting Position Set as: " << endl;

  cout << "   X: " << startingPosition.at(0) << endl;
  cout << "   Y: " << startingPosition.at(1) << endl;
  cout << "   Z: " << startingPosition.at(2) << endl;
  cout << "   Phi: " << startingPosition.at(3) << endl;
  cout << "   Theta: " << startingPosition.at(4) << endl;

  cout << endl;

  cout << "   Move the Touch 3D Geomagic to the initial position, hit enter to START ";
  char c = getch();

  cout << endl << endl;

  ros::Subscriber sub = n.subscribe("/phantom/pose", 1000, moved_touch);
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();
  }
}

bool inv_kin_kuka(double X, double Y, double Z, double eef_phi, double eef_theta, double armAng_in, float* jointAngles)
{
  // cout << "Commanded: \nPosition (X,Y,Z) [mm]: [" << X << ", " << Y << ", " << Z << "]" << endl;
  // cout << "EEF Orientation (Phi, Theta) [deg]: [" << eef_phi << ", " << eef_theta << "]" << endl;
  // cout << "Entered arm Angle [deg]: " << armAng << endl;
  // cout << "-------------------------------------" << endl;

  // calc joint values
  inv_kin_kuka_angle_calc(X, Y, Z, eef_phi, eef_theta, armAng_in);

  // check for joint violation
  if (abs(phi4) >= phi4_max)
  {
    cout << "Wrist to close to shoulder!" << endl;
    return false;
  }

  else if (abs(phi1) >= phi1_max | abs(phi2) >= phi2_max | abs(phi3) >= phi3_max | abs(phi4) >= phi4_max |
           abs(phi5) >= phi5_max | abs(phi6) >= phi6_max | abs(phi7) >= phi7_max)
  {
    armAng = adapt_elbow_position(X, Y, Z, eef_phi, eef_theta, armAng_in);
    // return false;
  }
  // store calculated joint values for next round
  phi1_old = phi1;
  phi2_old = phi2;
  phi3_old = phi3;
  phi4_old = phi4;
  phi5_old = phi5;
  phi6_old = phi6;
  phi7_old = phi7;

  // print out results ---------------------------------------------------------------------------
  // cout << "Angles [deg]: " << endl;
  // cout << "(new) Arm Angle: " << armAng << endl;
  // cout << "Phi 1: " << phi1_1 << "\nPhi 2: " << phi2_2 << "\nPhi 3: " << phi3_2 << endl;
  // cout << "Phi 4: " << phi4_2 << "\nPhi 5: " << phi5_2 << "\nPhi 6: " << phi6_2 << "\nPhi 7: " << phi7_2 << endl;
  cout << "-------------------------------------" << endl;

  if (isnan(phi1_1) || isnan(phi2_2) || isnan(phi3_2) || isnan(phi4_2) || isnan(phi5_2) || isnan(phi6_2) ||
      isnan(phi7_2))
  {
    return false;
  }
  jointAngles[0] = (float)phi1_1;
  jointAngles[1] = (float)phi2_2;
  jointAngles[2] = (float)phi3_2;
  jointAngles[3] = (float)phi4_2;
  jointAngles[4] = (float)phi5_2;
  jointAngles[5] = (float)phi6_2;
  jointAngles[6] = (float)phi7_2;

  return true;
}

double adapt_elbow_position(double X, double Y, double Z, double eef_phi, double eef_theta, double armAng)
{
  double armAng_entered = armAng;  // store enter value by user [deg]
  double armAng_1 = armAng;
  double armAng_2 = armAng;
  double loop_count = 0;

  // first solution
  while (abs(phi1) >= phi1_max | abs(phi2) >= phi2_max | abs(phi3) >= phi3_max | abs(phi4) >= phi4_max |
         abs(phi5) >= phi5_max | abs(phi6) >= phi6_max | abs(phi7) >= phi7_max)
  {
    // use redundancy circle to avoid joint limits
    armAng = armAng + 0.010;
    loop_count += 0.01;

    // calc new angle values
    inv_kin_kuka_angle_calc(X, Y, Z, eef_phi, eef_theta, armAng);

    cout << "!!!ADAPTION!!! " << endl;
    cout << "(new) Arm Angle: " << armAng << endl;
    // cout << "Phi 1: " << phi1_1 << "\nPhi 2: " << phi2_2 << "\nPhi 3: " << phi3_2 << endl;
    // cout << "Phi 4: " << phi4_2 << "\nPhi 5: " << phi5_2 << "\nPhi 6: " << phi6_2 << "\nPhi 7: " << phi7_2 << endl;
    // cout << "-------------------------------------" << endl;

    // if there is no solution
    if (loop_count > 360)
    {
      cout << "Target not reachable (impossible pose)" << endl;
      return 0;
    }
  }

  armAng_1 = armAng;  // new arm angle

  // store final values
  phi1_1 = phi1;
  phi2_1 = phi2;
  phi3_1 = phi3;
  phi4_1 = phi4;
  phi5_1 = phi5;
  phi6_1 = phi6;
  phi7_1 = phi7;

  // calc difference to previous angle values [deg]
  double phi1_diff = phi1_1 - phi1_old;
  double phi2_diff = phi2_1 - phi2_old;
  double phi3_diff = phi3_1 - phi3_old;
  double phi4_diff = phi4_1 - phi4_old;
  double phi5_diff = phi5_1 - phi5_old;
  double phi6_diff = phi6_1 - phi6_old;
  double phi7_diff = phi7_1 - phi7_old;
  double phi_diff_plus = abs(phi1_diff) + abs(phi2_diff) + abs(phi3_diff) + abs(phi4_diff) + abs(phi5_diff) +
                         abs(phi6_diff) + abs(phi7_diff);

  // second solution------------------------------------------------------
  // reset to init values (data entered by user)
  armAng = armAng_entered;
  loop_count = 0;
  inv_kin_kuka_angle_calc(X, Y, Z, eef_phi, eef_theta, armAng);

  while (abs(phi1) >= phi1_max | abs(phi2) >= phi2_max | abs(phi3) >= phi3_max | abs(phi4) >= phi4_max |
         abs(phi5) >= phi5_max | abs(phi6) >= phi6_max | abs(phi7) >= phi7_max)
  {
    // use redundancy circle to avoid joint limits
    armAng = armAng - 0.01;
    loop_count += 0.01;

    // calc new angle values
    inv_kin_kuka_angle_calc(X, Y, Z, eef_phi, eef_theta, armAng);

    // if there is no solution
    if (loop_count > 360)
    {
      cout << "Target not reachable (impossible pose)" << endl;
      return 0;
    }
  }

  armAng_2 = armAng;  // new arm angle

  // calc difference to previous angle values [deg]
  phi1_diff = phi1_2 - phi1_old;
  phi2_diff = phi2_2 - phi2_old;
  phi3_diff = phi3_2 - phi3_old;
  phi4_diff = phi4_2 - phi4_old;
  phi5_diff = phi5_2 - phi5_old;
  phi6_diff = phi6_2 - phi6_old;
  phi7_diff = phi7_2 - phi7_old;
  double phi_diff_minus = abs(phi1_diff) + abs(phi2_diff) + abs(phi3_diff) + abs(phi4_diff) + abs(phi5_diff) +
                          abs(phi6_diff) + abs(phi7_diff);

  //---------------------------------------------------------------------

  // find solution with least difference to previous joint values
  if (phi_diff_plus < phi_diff_minus)  // first solution is better
  {
    phi1 = phi1_1;
    phi2 = phi2_1;
    phi3 = phi3_1;
    phi4 = phi4_1;
    phi5 = phi5_1;
    phi6 = phi6_1;
    phi7 = phi7_1;
    armAng = armAng_1;  // update arm angle
  }
  else  // second solution is better
  {
    phi1 = phi1_2;
    phi2 = phi2_2;
    phi3 = phi3_2;
    phi4 = phi4_2;
    phi5 = phi5_2;
    phi6 = phi6_2;
    phi7 = phi7_2;
    armAng = armAng_2;  // update arm angle
  }

  return armAng;  // return updated arm angle
}

void inv_kin_kuka_angle_calc(double X, double Y, double Z, double eef_phi, double eef_theta, double armAng)
{
  // Check if target is in workspace (circle with radius 800mm) ----------------------------------
  vec_eef = {X, Y, Z};

  // vector wrist endeffector [wf]
  vector<double> vec_wf_tmp_2 = Mat_vec_prod_Ry(eef_theta * deg2rad, vec_wf_tmp);
  vector<double> vec_wf = Mat_vec_prod_Rz(eef_phi * deg2rad, vec_wf_tmp_2);

  // vector base to wrist, global position of wrist
  pw = Vector_substraction(vec_eef, vec_wf);
  // cout << "Vec_eef: [" << vec_eef.at(0) << " " << vec_eef.at(1) << " " << vec_eef.at(2) << "]" << endl;
  // cout << "vec_wf: [" << vec_wf.at(0) << " " << vec_wf.at(1) << " " << vec_wf.at(2) << "]" << endl;
  // cout << "Wrist position (X,Y,Z) [mm]: [" << pw.at(0) << " " << pw.at(1) << " " << pw.at(2) << "]" << endl;

  // vector shoulder to wrist
  psw = Vector_substraction(pw, p_shoulder);

  // check if length of vector psw greater then "d_se + d_ew - 1" (-1 to avoid singularity)
  if (sqrt(pow(psw.at(0), 2) + pow(psw.at(1), 2) + pow(psw.at(2), 2)) > (d_se + d_ew - 1))
  {
    cout << "target out of workspace!" << endl;
    return;  // abort calculation
  }

  // Elbow Position ------------------------------------------------------------------------------
  /*This only works for d_se = b, otherwise use formula for general triangle (Tafelwerk P.26)*/

  // length of vector shoulder to center of circle

  double psw_length = sqrt(pow(psw.at(0), 2) + pow(psw.at(1), 2) + pow(psw.at(2), 2));

  // double pc_length = sqrt(pow(pc.at(0), 2) + pow(pc.at(1), 2) + pow(pc.at(2), 2));

  double pc_length = abs(420.0 * (pow(400.0, 2) - pow(420.0, 2) - pow(psw_length, 2)) / (840.0 * psw_length));

  pc = Vector_division(psw, (psw_length / pc_length));

  // circle radius
  double alpha = asin(pc_length / d_se);
  double alpha2 = (alpha * 180) / pi;
  double R = cos(alpha) * d_se;

  // elbow position
  vector<double> Pe = Elbow_Position(armAng, R);

  // Target Center Axis --------------------------------------------------------------------------
  vector<double> vec_wf_unit =
      Vector_division(vec_wf, (sqrt(pow(vec_wf.at(0), 2) + pow(vec_wf.at(1), 2) + pow(vec_wf.at(2), 2))));
  vector<double> zero_axis = {1, 0, 0};  // arbitrary axis a, (here ArmAngle = 0)

  // find orthogonal vector to vec_wf
  //= zero_axis-(zero_axis'*pc_unit)*pc_unit;
  double u_tmp = Vector_scalar(zero_axis, vec_wf_unit);
  vector<double> vec_u_tmp_1 = Vector_multi(vec_wf_unit, u_tmp);
  vector<double> vec_u_tmp_2 = Vector_substraction(zero_axis, vec_u_tmp_1);
  double vec_u_tmp_len = sqrt(pow(vec_u_tmp_2.at(0), 2) + pow(vec_u_tmp_2.at(1), 2) + pow(vec_u_tmp_2.at(2), 2));
  vector<double> vec_u_tmp_3 = Vector_division(vec_u_tmp_2, vec_u_tmp_len);

  // rotate found orthogonal vector (around z-axis needed only)
  vector<double> vec_u = Mat_vec_prod_Rz(eef_phi * deg2rad, vec_u_tmp_3);

  vector<double> vec_v = Vector_cross(vec_u, vec_wf_unit);
  double vec_b_length = sqrt(pow(vec_v.at(0), 2) + pow(vec_v.at(1), 2) + pow(vec_v.at(2), 2));
  vector<double> Tcb = Vector_multi(Vector_division(vec_v, vec_b_length), (-1));  // unit vector

  // Angle Calculation----------------------------------------------------------------------------
  // phi 1
  phi1 = atan2(Pe.at(1), Pe.at(0));  // atan2 is only needed here!?
  if (abs(phi1) >= 2 * pi)           // map rotation to 360° circle
    phi1 = 2 * pi - phi1;
  // phi1 = round(phi1*10000)/10000; //round to 2 decimal places
  // phi1 = phi1 - pi;
  if (abs(phi1) == 0)  // avoid negative 0
    phi1 = 0;
  phi1_1 = (phi1 * 180) / pi;
  // phi1_1 = round(phi1_1*10000)/10000; //round to 2 decimal places

  // phi 2 ---------------------------------------------------------------------------------------
  phi2 = acos((Pe.at(2) - d_bs) / d_se);
  if (abs(phi2) >= 2 * pi)  // map rotation to 360° circle
    phi2 = 2 * pi - phi2;
  // phi2 = round(phi2*10000)/10000; //round to 2 decimal places
  if (abs(phi2) == 0)  // avoid negative 0
    phi2 = 0;
  phi2_2 = (phi2 * 180) / pi;
  // phi2_2 = round(phi2_2*10000)/10000; //round to 2 decimal places

  // phi 3 ---------------------------------------------------------------------------------------

  phi3 = atan2(
      (pw.at(1) * cos(phi1) - pw.at(0) * sin(phi1)),
      (d_bs * sin(phi2) - pw.at(2) * sin(phi2) + pw.at(0) * cos(phi1) * cos(phi2) + pw.at(1) * cos(phi2) * sin(phi1)));

  phi3 = phi3 - pi;  // otherwise wrist position wrong!?

  // if (abs(phi3) >= 2*pi) //map rotation to 360° circle
  //    phi3 = 2*pi - phi3;

  if (phi3 <= (-1) * 2 * pi)
    phi3 = 2 * pi + phi3;
  else if (phi3 > 2 * pi)
    phi3 = 2 * pi - phi3;

  // phi3 = round(phi3*10000)/10000; //round to 2 decimal places
  if (abs(phi3) == 0)  // avoid negative 0
    phi3 = 0;
  phi3_2 = (phi3 * 180) / pi;
  // phi3_2 = round(phi3_2*10000)/10000; //round to 2 decimal places

  // phi 4 ---------------------------------------------------------------------------------------
  phi4 = acos((pw.at(2) * cos(phi2) - d_bs * cos(phi2) + pw.at(0) * cos(phi1) * sin(phi2) +
               pw.at(1) * sin(phi1) * sin(phi2) - d_se) /
              d_ew);

  if (abs(phi4) >= 2 * pi)  // map rotation to 360° circle
    phi4 = 2 * pi - phi4;
  // phi4 = round(phi4*10000)/10000; //round to 2 decimal places
  if (abs(phi4) == 0)  // avoid negative 0
    phi4 = 0;
  phi4_2 = (phi4 * 180) / pi;
  // phi4_2 = round(phi4_2*10000)/10000; //round to 2 decimal places

  // phi 5 ---------------------------------------------------------------------------------------
  phi5 = atan2((Y * cos(phi1) * cos(phi3) - X * cos(phi3) * sin(phi1) + Z * sin(phi2) * sin(phi3) -
                d_bs * sin(phi2) * sin(phi3) - X * cos(phi1) * cos(phi2) * sin(phi3) -
                Y * cos(phi2) * sin(phi1) * sin(phi3)),
               (Z * cos(phi2) * sin(phi4) - d_se * sin(phi4) - d_bs * cos(phi2) * sin(phi4) +
                Y * cos(phi1) * cos(phi4) * sin(phi3) - Z * cos(phi3) * cos(phi4) * sin(phi2) +
                X * cos(phi1) * sin(phi2) * sin(phi4) - X * cos(phi4) * sin(phi1) * sin(phi3) +
                d_bs * cos(phi3) * cos(phi4) * sin(phi2) + Y * sin(phi1) * sin(phi2) * sin(phi4) +
                X * cos(phi1) * cos(phi2) * cos(phi3) * cos(phi4) + Y * cos(phi2) * cos(phi3) * cos(phi4) * sin(phi1)));
  if (abs(phi5) >= 2 * pi)  // map rotation to 360° circle
    phi5 = 2 * pi - phi5;
  // phi5 = round(phi5*10000)/10000; //round to 2 decimal places
  if (abs(phi5) == 0)  // avoid negative 0
    phi5 = 0;
  phi5_2 = (phi5 * 180) / pi;
  // phi5_2 = round(phi5_2*10000)/10000; //round to 2 decimal places

  // phi 6 ---------------------------------------------------------------------------------------
  phi6 = acos((Z * cos(phi2) * cos(phi4) - d_se * cos(phi4) - d_bs * cos(phi2) * cos(phi4) +
               X * cos(phi1) * cos(phi4) * sin(phi2) + Y * cos(phi4) * sin(phi1) * sin(phi2) -
               Y * cos(phi1) * sin(phi3) * sin(phi4) + Z * cos(phi3) * sin(phi2) * sin(phi4) +
               X * sin(phi1) * sin(phi3) * sin(phi4) - d_bs * cos(phi3) * sin(phi2) * sin(phi4) -
               X * cos(phi1) * cos(phi2) * cos(phi3) * sin(phi4) - Y * cos(phi2) * cos(phi3) * sin(phi1) * sin(phi4) -
               d_ew) /
              d_wf);
  if (abs(phi6) >= 2 * pi)  // map rotation to 360° circle
    phi6 = 2 * pi - phi6;
  // phi6 = round(phi6*10000)/10000; //round to 2 decimal places
  if (abs(phi6) == 0)  // avoid negative 0
    phi6 = 0;
  phi6_2 = (phi6 * 180) / pi;
  // phi6_2 = round(phi6_2*10000)/10000; //round to 2 decimal places

  // phi 7 ---------------------------------------------------------------------------------------
  phi7 =
      atan((-1) *
           (Tcb.at(1) * cos(phi1) * cos(phi3) * cos(phi6) * sin(phi5) - Tcb.at(2) * cos(phi2) * cos(phi4) * sin(phi6) +
            Tcb.at(2) * cos(phi2) * cos(phi5) * cos(phi6) * sin(phi4) -
            Tcb.at(0) * cos(phi1) * cos(phi4) * sin(phi2) * sin(phi6) -
            Tcb.at(0) * cos(phi3) * cos(phi6) * sin(phi1) * sin(phi5) -
            Tcb.at(1) * cos(phi4) * sin(phi1) * sin(phi2) * sin(phi6) +
            Tcb.at(1) * cos(phi1) * sin(phi3) * sin(phi4) * sin(phi6) -
            Tcb.at(2) * cos(phi3) * sin(phi2) * sin(phi4) * sin(phi6) +
            Tcb.at(2) * cos(phi6) * sin(phi2) * sin(phi3) * sin(phi5) -
            Tcb.at(0) * sin(phi1) * sin(phi3) * sin(phi4) * sin(phi6) +
            Tcb.at(1) * cos(phi2) * cos(phi3) * sin(phi1) * sin(phi4) * sin(phi6) -
            Tcb.at(1) * cos(phi2) * cos(phi6) * sin(phi1) * sin(phi3) * sin(phi5) +
            Tcb.at(1) * cos(phi5) * cos(phi6) * sin(phi1) * sin(phi2) * sin(phi4) +
            Tcb.at(1) * cos(phi1) * cos(phi4) * cos(phi5) * cos(phi6) * sin(phi3) -
            Tcb.at(2) * cos(phi3) * cos(phi4) * cos(phi5) * cos(phi6) * sin(phi2) +
            Tcb.at(0) * cos(phi1) * cos(phi2) * cos(phi3) * sin(phi4) * sin(phi6) -
            Tcb.at(0) * cos(phi1) * cos(phi2) * cos(phi6) * sin(phi3) * sin(phi5) +
            Tcb.at(0) * cos(phi1) * cos(phi5) * cos(phi6) * sin(phi2) * sin(phi4) -
            Tcb.at(0) * cos(phi4) * cos(phi5) * cos(phi6) * sin(phi1) * sin(phi3) +
            Tcb.at(0) * cos(phi1) * cos(phi2) * cos(phi3) * cos(phi4) * cos(phi5) * cos(phi6) +
            Tcb.at(1) * cos(phi2) * cos(phi3) * cos(phi4) * cos(phi5) * cos(phi6) * sin(phi1)) /
           (Tcb.at(1) * cos(phi1) * cos(phi3) * cos(phi5) - Tcb.at(0) * cos(phi3) * cos(phi5) * sin(phi1) +
            Tcb.at(2) * cos(phi5) * sin(phi2) * sin(phi3) - Tcb.at(2) * cos(phi2) * sin(phi4) * sin(phi5) -
            Tcb.at(0) * cos(phi1) * cos(phi2) * cos(phi5) * sin(phi3) -
            Tcb.at(1) * cos(phi2) * cos(phi5) * sin(phi1) * sin(phi3) -
            Tcb.at(1) * cos(phi1) * cos(phi4) * sin(phi3) * sin(phi5) +
            Tcb.at(2) * cos(phi3) * cos(phi4) * sin(phi2) * sin(phi5) -
            Tcb.at(0) * cos(phi1) * sin(phi2) * sin(phi4) * sin(phi5) +
            Tcb.at(0) * cos(phi4) * sin(phi1) * sin(phi3) * sin(phi5) -
            Tcb.at(1) * sin(phi1) * sin(phi2) * sin(phi4) * sin(phi5) -
            Tcb.at(0) * cos(phi1) * cos(phi2) * cos(phi3) * cos(phi4) * sin(phi5) -
            Tcb.at(1) * cos(phi2) * cos(phi3) * cos(phi4) * sin(phi1) * sin(phi5)));
  if (abs(phi7) >= 2 * pi)  // map rotation to 360° circle
    phi7 = 2 * pi - phi7;
  // phi7 = round(phi7*10000)/10000; //round to 2 decimal places
  if (abs(phi7) == 0)  // avoid negative 0
    phi7 = 0;
  phi7_2 = (phi7 * 180) / pi;
  // phi7_2 = round(phi7_2*10000)/10000; //round to 2 decimal places
}

vector<double> Mat_vec_prod_Rx(double theta, vector<double> input_vector)
{
  vector<double> tmp_vec = {0, 0, 0};
  double Rx[3][3] = {{1, 0, 0}, {cos(theta), 0, sin(theta)}, {0, -sin(theta), cos(theta)}};

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      tmp_vec.at(i) += Rx[i][j] * input_vector.at(j);
    }
  }

  return tmp_vec;
}

vector<double> Mat_vec_prod_Ry(double theta, vector<double> input_vector)
{
  vector<double> tmp_vec = {0, 0, 0};
  double Ry[3][3] = {{0, cos(theta), -sin(theta)}, {0, 1, 0}, {0, -sin(theta), cos(theta)}};

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      tmp_vec.at(i) += Ry[i][j] * input_vector.at(j);
    }
  }

  return tmp_vec;
}

vector<double> Mat_vec_prod_Rz(double theta, vector<double> input_vector)
{
  vector<double> tmp_vec = {0, 0, 0};
  double Rz[3][3] = {{cos(theta), -sin(theta), 0}, {sin(theta), cos(theta), 0}, {0, 0, 1}};

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      tmp_vec.at(i) += Rz[i][j] * input_vector.at(j);
    }
  }

  return tmp_vec;
}

vector<double> Vector_substraction(vector<double> input_vector, vector<double> input_vector2)
{
  vector<double> tmp_vec = {0, 0, 0};

  tmp_vec.at(0) = input_vector.at(0) - input_vector2.at(0);
  tmp_vec.at(1) = input_vector.at(1) - input_vector2.at(1);
  tmp_vec.at(2) = input_vector.at(2) - input_vector2.at(2);

  return tmp_vec;
}

vector<double> Vector_addition(vector<double> input_vector, vector<double> input_vector2)
{
  vector<double> tmp_vec = {0, 0, 0};

  tmp_vec.at(0) = input_vector.at(0) + input_vector2.at(0);
  tmp_vec.at(1) = input_vector.at(1) + input_vector2.at(1);
  tmp_vec.at(2) = input_vector.at(2) + input_vector2.at(2);

  return tmp_vec;
}

vector<double> Vector_division(vector<double> input_vector, double divider)
{
  vector<double> tmp_vec = {0, 0, 0};

  tmp_vec.at(0) = input_vector.at(0) / divider;
  tmp_vec.at(1) = input_vector.at(1) / divider;
  tmp_vec.at(2) = input_vector.at(2) / divider;

  return tmp_vec;
}

vector<double> Vector_multi(vector<double> input_vector, double value)
{
  vector<double> tmp_vec = {0, 0, 0};

  tmp_vec.at(0) = input_vector.at(0) * value;
  tmp_vec.at(1) = input_vector.at(1) * value;
  tmp_vec.at(2) = input_vector.at(2) * value;

  return tmp_vec;
}

vector<double> Vector_cross(vector<double> input_vector, vector<double> input_vector2)
{
  vector<double> tmp_vec = {0, 0, 0};

  tmp_vec.at(0) = input_vector.at(1) * input_vector2.at(2) - input_vector.at(2) * input_vector2.at(1);
  tmp_vec.at(1) = input_vector.at(2) * input_vector2.at(0) - input_vector.at(0) * input_vector2.at(2);
  tmp_vec.at(2) = input_vector.at(0) * input_vector2.at(1) - input_vector.at(1) * input_vector2.at(0);

  return tmp_vec;
}

double Vector_scalar(vector<double> input_vector, vector<double> input_vector2)
{
  double scalar = input_vector.at(0) * input_vector2.at(0) + input_vector.at(1) * input_vector2.at(1) +
                  input_vector.at(2) * input_vector2.at(2);

  return scalar;
}

vector<double> Elbow_Position(double armAng, double R)
{
  // vectors of circle (all orthogonal, unit vectors), see pdf: Real-Time Inverse Kinematics...P.12
  vector<double> pc_unit = Vector_division(pc, (sqrt(pow(pc.at(0), 2) + pow(pc.at(1), 2) + pow(pc.at(2), 2))));
  vector<double> zero_axis = {1, 0, 0};  // arbitrary axis a, (here ArmAngle = 0)

  //= zero_axis-(zero_axis'*pc_unit)*pc_unit;
  double u_tmp = Vector_scalar(zero_axis, pc_unit);
  vector<double> vec_u_tmp_1 = Vector_multi(pc_unit, u_tmp);
  vector<double> vec_u_tmp_2 = Vector_substraction(zero_axis, vec_u_tmp_1);
  double vec_u_tmp_len = sqrt(pow(vec_u_tmp_2.at(0), 2) + pow(vec_u_tmp_2.at(1), 2) + pow(vec_u_tmp_2.at(2), 2));
  vector<double> vec_u = Vector_division(vec_u_tmp_2, vec_u_tmp_len);

  vector<double> vec_v = Vector_cross(vec_u, pc);
  double vec_b_length = sqrt(pow(vec_v.at(0), 2) + pow(vec_v.at(1), 2) + pow(vec_v.at(2), 2));
  vec_v = Vector_division(vec_v, vec_b_length);  // unit vector

  // elbow position
  // Formula: Pe = R * (cos(armAng * deg2rad) * u + sin(armAng * deg2rad) * v) + pc + p_shoulder;
  vector<double> vec_tmp_1 = Vector_multi(vec_u, cos(armAng * deg2rad));  // cos(armAng * deg2rad) * u
  vector<double> vec_tmp_2 = Vector_multi(vec_v, sin(armAng * deg2rad));  // sin(armAng * deg2rad) * v
  vector<double> vec_tmp_3 = Vector_addition(vec_tmp_1, vec_tmp_2);       //... + ...
  vector<double> vec_tmp_4 = Vector_multi(vec_tmp_3, R);                  // R * ...
  vector<double> vec_tmp_5 = Vector_addition(vec_tmp_4, pc);              // ... + pc
  vector<double> vec_tmp_6 = Vector_addition(vec_tmp_5, p_shoulder);      //... + p_shoudler

  // cout << "Elbow position (X,Y,Z) [mm]: [" << vec_tmp_6.at(0) << " " << vec_tmp_6.at(1) << " " << vec_tmp_6.at(2)
  // <<
  // "]" << endl; cout << "-------------------------------------" << endl;

  return vec_tmp_6;
}
