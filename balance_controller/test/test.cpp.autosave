/*
 *  test.cpp
 *  Descriotion: test the VMC algorithm
 *
 *  Created on: Mar 15, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

// gtest
#include <gtest/gtest.h>

#include "ros/ros.h"

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/free_gait_core.hpp"
#include "balance_controller/contact_force_distribution/ContactForceDistributionBase.hpp"
#include "balance_controller/contact_force_distribution/ContactForceDistribution.hpp"
#include "balance_controller/motion_control/MotionControllerBase.hpp"
#include "balance_controller/motion_control/VirtualModelController.hpp"
#include "balance_controller/ros_controler/robot_state_gazebo_ros_control_plugin.hpp"
#include "balance_controller/ros_controler/gazebo_state_hardware_interface.hpp"
#include "pluginlib/class_loader.h"
#include "balance_controller/ros_controler/robot_state_interface.hpp"

using namespace free_gait;
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "balance_controller_test_node");
  ros::NodeHandle nh("~");
  boost::shared_ptr<pluginlib::ClassLoader<gazebo_ros_control::RobotHWSim> > robot_hw_sim_loader_;
  boost::shared_ptr<pluginlib::ClassLoader<controller_interface::ControllerBase>> controller_loader;
  boost::shared_ptr<gazebo_ros_control::RobotHWSim> robot_hw_sim_;
  robot_hw_sim_loader_.reset
    (new pluginlib::ClassLoader<gazebo_ros_control::RobotHWSim>
      ("balance_controller",
        "gazebo_ros_control::RobotHWSim"));
  robot_hw_sim_ = robot_hw_sim_loader_->createInstance("balance_controller/SimRobotStateHardwareInterface");
  controller_loader.reset(new pluginlib::ClassLoader<controller_interface::ControllerBase>(
                            "balance_controller", "controller_interface::ControllerBase"));
  boost::shared_ptr<controller_interface::ControllerBase> controller;
  controller = controller_loader->createInstance("balance_controller/RosBalanceController");

  hardware_interface::RobotStateHandle::Data robot_data;
  double* pos_ptr = 0;
  double pos[3];
  pos_ptr = pos;
  pos_ptr[0] = 1.0;
  pos_ptr[1] = 2.0;
  pos_ptr[2] = 3.0;
  ROS_INFO("Got here");
  robot_data.joint_position_read = pos;

  std::shared_ptr<free_gait::State> robot_state;
  robot_state.reset(new free_gait::State);
  robot_state->Initialize();
  kindr::EulerAnglesZyxPD rotation(0.5, 0.0, 0.0);
  free_gait::Stance support_stance = Stance({
                                            {LimbEnum::LF_LEG, Position(0.3, 0.35, -0.2)},
                                            {LimbEnum::RF_LEG, Position(0.3, -0.35, -0.2)},
                                            {LimbEnum::RH_LEG, Position(-0.3, -0.35, -0.2)},
                                            {LimbEnum::LH_LEG, Position(-0.3, 0.35, -0.2)} });

  // current state
  robot_state->setPoseBaseToWorld(Pose(Position(0,0,0.2),RotationQuaternion(rotation)));
  robot_state->setBaseStateFromFeedback(LinearVelocity(0,0,0), LocalAngularVelocity(0,0,0));

  robot_state->setSupportFootStance(support_stance);
//!  set current joint position
  JointPositions all_joint_positions;
  JointPositionsLeg joint_position_limb;
  std::cout<<"size rows X columns "<<all_joint_positions.toImplementation().size()<<std::endl;
//  robot_state->InverseKinematicsSolve(support_stance.at(LimbEnum::LF_LEG),LimbEnum::LF_LEG,
//                                      joint_position_limb, joint_position_limb);
  robot_state->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(support_stance.at(LimbEnum::LF_LEG), LimbEnum::LF_LEG,
                                                                      joint_position_limb);
  all_joint_positions.setSegment<3>(QD::getLimbStartIndexInJ(LimbEnum::LF_LEG),joint_position_limb);
//  robot_state->InverseKinematicsSolve(support_stance.at(LimbEnum::RF_LEG),LimbEnum::RF_LEG,
//                                      joint_position_limb, joint_position_limb);
  robot_state->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(support_stance.at(LimbEnum::RF_LEG), LimbEnum::RF_LEG,
                                                                      joint_position_limb);
  all_joint_positions.setSegment<3>(QD::getLimbStartIndexInJ(LimbEnum::RF_LEG),joint_position_limb);
//  robot_state->InverseKinematicsSolve(support_stance.at(LimbEnum::RH_LEG),LimbEnum::RH_LEG,
//                                      joint_position_limb, joint_position_limb);
  robot_state->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(support_stance.at(LimbEnum::RH_LEG), LimbEnum::RH_LEG,
                                                                      joint_position_limb);
  all_joint_positions.setSegment<3>(QD::getLimbStartIndexInJ(LimbEnum::RH_LEG),joint_position_limb);
//  robot_state->InverseKinematicsSolve(support_stance.at(LimbEnum::LH_LEG),LimbEnum::LH_LEG,
//                                      joint_position_limb, joint_position_limb);
  robot_state->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(support_stance.at(LimbEnum::LH_LEG), LimbEnum::LH_LEG,
                                                                      joint_position_limb);
  all_joint_positions.setSegment<3>(QD::getLimbStartIndexInJ(LimbEnum::LH_LEG),joint_position_limb);
  std::cout<<"all_joint_positions is : "<<all_joint_positions<<std::endl;
  robot_state->setCurrentLimbJoints(all_joint_positions);
  //!set support leg and surface normal
  for(auto leg : support_stance)
    {
      robot_state->setSupportLeg(leg.first, true);
      robot_state->setSurfaceNormal(leg.first, Vector(0,0,1));
    }

  //desire state
  robot_state->setPositionWorldToBaseInWorldFrame(Position(0,0,0.1));
  robot_state->setOrientationBaseToWorld(RotationQuaternion(rotation));
  robot_state->setLinearVelocityBaseInWorldFrame(LinearVelocity(0,0,0));
  robot_state->setAngularVelocityBaseInBaseFrame(LocalAngularVelocity(0,0,0));


  auto CFD = std::shared_ptr<balance_controller::ContactForceDistribution>(new balance_controller::ContactForceDistribution(nh, robot_state));

  if(!CFD->loadParameters()){
      ROS_INFO("CFD load parameters failed");
    }
  auto VMC = std::shared_ptr<balance_controller::VirtualModelController>(new balance_controller::VirtualModelController(nh, robot_state, CFD));
//  balance_controller::VirtualModelController VMC(nh, robot_state, CFD);
  if(!VMC->loadParameters()){
      ROS_INFO("VMC load parameters failed");
    }
  if(!VMC->compute())
    {
      ROS_INFO("VMC compute failed");
    }
  std::cout<<*VMC<<std::endl;
  for(int i = 0;i<4;i++){
      std::cout<<"Contact force for leg "<<i<<" is :"<<CFD->getLegInfo(static_cast<free_gait::LimbEnum>(i)).desiredContactForce_<<std::endl;
    }

  std::cout<<"Computed Virtual Force: "<<VMC->getDesiredVirtualForceInBaseFrame()<<std::endl
          <<"Compute Virtual Torque : "<<VMC->getDesiredVirtualTorqueInBaseFrame()<<std::endl;
  std::cout<<"Computed Joint Torque : "<<std::endl<<robot_state->getAllJointEfforts()<<std::endl;
  return 0;
}




