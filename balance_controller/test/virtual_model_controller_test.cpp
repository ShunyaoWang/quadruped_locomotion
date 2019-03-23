/*
 *  filename.cpp
 *  Descriotion:
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

TEST(virtual_model_controller, QPOptimization)
{
  ros::NodeHandle nh("~");
  std::shared_ptr<free_gait::State> robot_state;
  robot_state.reset(new free_gait::State);
  kindr::EulerAnglesZyxPD rotation(0.5, 0.0, 0.0);
  robot_state->setPoseBaseToWorld(Pose(Position(0,0,0.4),RotationQuaternion(rotation)));

  auto CFD = std::shared_ptr<balance_controller::ContactForceDistribution>(new balance_controller::ContactForceDistribution(nh, robot_state));
  ASSERT_TRUE(CFD->loadParameters());
  balance_controller::VirtualModelController VMC(nh, robot_state, CFD);
  ASSERT_TRUE(VMC.loadParameters());

}
