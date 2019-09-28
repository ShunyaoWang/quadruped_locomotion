/*
 *  test.cpp
 *  Descriotion:
 *
 *  Created on: Aug 31, 2017
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/leg_motion/Footstep.hpp"
#include "free_gait_core/free_gait_core.hpp"
//#include "free_gait_core/executor/AdapterDummy.hpp"
//#include "free_gait_ros/StepRosConverter.hpp"
#include "free_gait_msgs/Footstep.h"
#include "free_gait_ros/free_gait_ros.hpp"
#include "pluginlib/class_loader.h"


using namespace free_gait;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "step_test");
  ros::NodeHandle nh("~");
  AdapterRos adapterRos_(nh, free_gait::AdapterRos::AdapterType::Preview);
  pluginlib::ClassLoader<AdapterBase> adapter_loader("free_gait_ros", "free_gait::AdapterBase");//package_name, base_class
  std::unique_ptr<AdapterBase> adapter;
  adapter.reset(adapter_loader.createUnmanagedInstance("free_gait_ros/AdapterDummy"));//create an instance of free_gait_ros/AdapterDummy


  JointPositionsLeg joints(0,0,0);
  Position end = adapter->getPositionBaseToFootInBaseFrame(LimbEnum::LF_LEG,joints);
  cout<<end<<endl;
  adapter->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(end,LimbEnum::LF_LEG,joints);
  cout<<joints<<endl;

  State state;
  StepParameters parameters;
  StepCompleter completer(parameters, *adapter);
  StepComputer computer;
  Executor executor(completer, computer, *adapter, state);
  BatchExecutor batch_executor(executor);
  cout<<"======================"<<endl;
  executor.initialize();
  cout<<"======================"<<endl;
  StepRosConverter converter(*adapter);
  StateRosPublisher rosPublisher(nh, *adapter);

  std::vector<Step> steps;
  Step step;
  // Basemotion
  step.setId("00");
  free_gait_msgs::BaseAuto baseauto_msg;
  baseauto_msg.height = 0.4;
  baseauto_msg.average_angular_velocity = 0.0;
  baseauto_msg.average_linear_velocity = 0.0;
  baseauto_msg.support_margin = 0.0;
  baseauto_msg.ignore_timing_of_leg_motion = false;
  BaseAuto baseauto;
  converter.fromMessage(baseauto_msg, baseauto);
  step.addBaseMotion(baseauto);
  steps.push_back(step);

  Step step1;
  step1.setId("01");
  free_gait_msgs::BaseTarget base_target_msg;
  base_target_msg.target.header.frame_id = "odom";
  base_target_msg.target.pose.position.x = 0.1;
  base_target_msg.target.pose.position.y = 0;
  base_target_msg.target.pose.position.z = 0.4;
  base_target_msg.target.pose.orientation.w = 1;
  base_target_msg.target.pose.orientation.x = 0;
  base_target_msg.target.pose.orientation.y = 0;
  base_target_msg.target.pose.orientation.z = 0;
  base_target_msg.average_angular_velocity = 0.1;
  base_target_msg.average_linear_velocity = 0.1;
  base_target_msg.ignore_timing_of_leg_motion = false;
  BaseTarget baseTarget;
  converter.fromMessage(base_target_msg, baseTarget);
  step1.addBaseMotion(baseTarget);
  steps.push_back(step1);

  Step step2;
  step2.setId("02");
  // Footstep
  free_gait_msgs::Footstep footstep_msg;
  geometry_msgs::PointStamped target;
  geometry_msgs::Vector3Stamped surface_normal;
  target.point.x = 0.5;
  target.point.y = 0.25;
  target.point.z = 0.0;
  target.header.frame_id = "odom";
  surface_normal.vector.x = 0.0;
  surface_normal.vector.y = 0.0;
  surface_normal.vector.z = 1.0;
//  footstep.updateStartPosition(start);
  footstep_msg.name = "LF_LEG";
  footstep_msg.target = target;
  footstep_msg.average_velocity = 0.15;
  footstep_msg.profile_height = 0.15;
  footstep_msg.profile_type = "triangle";
  footstep_msg.ignore_contact = false;
  footstep_msg.ignore_for_pose_adaptation = false;
  footstep_msg.surface_normal = surface_normal;

  Footstep footstep(LimbEnum::LF_LEG);
  converter.fromMessage(footstep_msg, footstep);
  step2.addLegMotion(footstep);
  // Basemotion
//  free_gait_msgs::BaseAuto baseauto_msg;
  baseauto_msg.height = 0.4;
  baseauto_msg.average_angular_velocity = 0.0;
  baseauto_msg.average_linear_velocity = 0.0;
  baseauto_msg.support_margin = 0.0;
  baseauto_msg.ignore_timing_of_leg_motion = false;
//  BaseAuto baseauto;
  converter.fromMessage(baseauto_msg, baseauto);
  step2.addBaseMotion(baseauto);
  // in to step queue
  steps.push_back(step2);

  Step step3;
  step3.setId("03");
  // Footstep
//  free_gait_msgs::Footstep footstep_msg;
  //  Position start(0.0, 0.0, 0.0);
//  Position target(0.3, 0.0, 0.0);
//  Vector sufaceNormal(0, 0, 1);
//  geometry_msgs::PointStamped target;
//  geometry_msgs::Vector3Stamped surface_normal;
  target.point.x = -0.2;
  target.point.y = -0.25;
  target.point.z = 0.0;
  target.header.frame_id = "odom";
  surface_normal.vector.x = 0.0;
  surface_normal.vector.y = 0.0;
  surface_normal.vector.z = 1.0;
//  footstep.updateStartPosition(start);
  footstep_msg.name = "RH_LEG";
  footstep_msg.target = target;
  footstep_msg.average_velocity = 0.15;
  footstep_msg.profile_height = 0.15;
  footstep_msg.profile_type = "triangle";
  footstep_msg.ignore_contact = false;
  footstep_msg.ignore_for_pose_adaptation = false;
  footstep_msg.surface_normal = surface_normal;

  Footstep footstep2(LimbEnum::RH_LEG);
  converter.fromMessage(footstep_msg, footstep2);
  step3.addLegMotion(footstep2);
  // Basemotion
//  free_gait_msgs::BaseAuto baseauto_msg;
  baseauto_msg.height = 0.4;
  baseauto_msg.average_angular_velocity = 0.0;
  baseauto_msg.average_linear_velocity = 0.0;
  baseauto_msg.support_margin = 0.0;
  baseauto_msg.ignore_timing_of_leg_motion = false;
//  BaseAuto baseauto;
  converter.fromMessage(baseauto_msg, baseauto);
  step3.addBaseMotion(baseauto);
  steps.push_back(step3);

  free_gait_msgs::BaseTrajectory base_trajectory_msg;

  BaseTrajectory baseTrajectory;
  converter.fromMessage(base_trajectory_msg, baseTrajectory);


  executor.getQueue().add(steps);
  executor.setPreemptionType(Executor::PreemptionType::PREEMPT_IMMEDIATE);
  double dt = 0.01;
  double time = 0.0;
  ros::Time t_start = ros::Time::now();
  ros::Rate rate(100);
  while (!executor.getQueue().empty()) {
    executor.advance(dt, true);
    time = time + dt;
    rosPublisher.publish(adapter->getState());
    rate.sleep();
  }
  ros::Time t_end = ros::Time::now();
cout<<"end time : "<<time<<endl;
cout<<"Real time costs is : "<<t_end-t_start<<endl;
//  footstep.setTargetPosition("map", target);
//  footstep.setProfileHeight(0.05);
//  footstep.setProfileType("triangle");
//  footstep.setAverageVelocity(0.15);



}
