/*
 *  kinematicsTest.cpp
 *  Descriotion:
 *
 *  Created on: date, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#include <ros/ros.h>
#include "quadruped_model/quadrupedkinematics.h"
//#include "geometry_msgs/Twist.h"
//#include "gazebo_msgs/ModelStates.h"
#include "rbdl/Model.h"
using namespace std;
using namespace quadruped_model;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematicsTest");
  ros::NodeHandle nh;
  Eigen::MatrixXd jacobian;
  QuadrupedKinematics QK;
//  QK.LoadRobotDescriptionFromFile("/home/hitstar/catkin_ws/src/quadruped_locomotion-dev/quadruped_model/urdf/simpledog.urdf");
  JointPositionsLimb joints(0,0,0);
  Pose result;
  HipPoseInBase test_enum;
  test_enum[LimbEnum::RF_LEG] = Pose(Position(0,0,0), RotationQuaternion());
  if(QK.FowardKinematicsSolve(joints, LimbEnum::RF_LEG, result))
  {
    EulerAnglesZyx eular_zyx(result.getRotation());
    cout<<"Kinematics solve result:"<<endl<<result.getPosition()<<endl<<
          "Rotation: "<<endl<<"Roll: "<<eular_zyx.roll()<<endl<<"Pitch: "<<
          eular_zyx.pitch()<<endl<<"Yaw: "<<eular_zyx.yaw()<<endl;
  }
  if(QK.InverseKinematicsSolve(result.getPosition(),LimbEnum::RF_LEG,joints,joints))
  {
    cout<<"Inverse Kinematics solve results: "<<endl<<"joint1 = "<<joints(0)<<endl
       <<"joint2 = "<<joints(1)<<endl<<"joint3 = "<<joints(2)<<endl;
  }

  QK.AnalysticJacobian(joints, LimbEnum::RF_LEG, jacobian);
  cout<<"jacobian : "<<endl<<jacobian<<endl;

  ROS_INFO("Hello world!");
}
