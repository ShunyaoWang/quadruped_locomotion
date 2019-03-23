/*
 * FootstepTest.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/leg_motion/Footstep.hpp"
#include "free_gait_core/free_gait_core.hpp"
#include "AdapterDummy.hpp"
#include "free_gait_ros/StepRosConverter.hpp"
#include "free_gait_msgs/Footstep.h"
// gtest
#include <gtest/gtest.h>

using namespace free_gait;

//TEST(footstep, triangleLowLongStep)
//{
//  Footstep footstep(LimbEnum::LF_LEG);
//  Position start(0.0, 0.0, 0.0);
//  Position target(0.3, 0.0, 0.0);
//  footstep.updateStartPosition(start);
//  footstep.setTargetPosition("map", target);
//  double height = 0.05;
//  footstep.setProfileHeight(0.05);
//  footstep.setProfileType("triangle");
//  footstep.setAverageVelocity(0.15);
//  ASSERT_TRUE(footstep.compute(true));

//  for (double time = 0.0; time < footstep.getDuration(); time += 0.001) {
//    Position position = footstep.evaluatePosition(time);
//    EXPECT_LT(position.z(), height + 0.001);
//    EXPECT_GT(position.x(), start.x() - 0.001);
//    EXPECT_LT(position.x(), target.x() + 0.001);
//  }
//}

//TEST(footstep, trianglehighLong)
//{
//  Footstep footstep(LimbEnum::LF_LEG);
//  Position start(0.0, 0.0, 0.0);
//  Position target(0.3, 0.0, 0.3);
//  footstep.updateStartPosition(start);
//  footstep.setTargetPosition("map", target);
//  double height = 0.05;
//  footstep.setProfileHeight(0.05);
//  footstep.setProfileType("triangle");
//  footstep.setAverageVelocity(0.15);
//  ASSERT_TRUE(footstep.compute(true));

//  for (double time = 0.0; time < footstep.getDuration(); time += 0.001) {
//    Position position = footstep.evaluatePosition(time);
//    EXPECT_LT(position.z(), target.z() + height + 0.01);
//    EXPECT_GT(position.x(), start.x() - 0.001);
//    EXPECT_LT(position.x(), target.x() + 0.001);
//  }
//}

TEST(executor, footstepExecute)
{
  AdapterDummy adapter;
  State state;
  StepParameters parameters;
  StepCompleter completer(parameters, adapter);
  StepComputer computer;
  Executor executor(completer, computer, adapter, state);
  StepRosConverter converter(adapter);
  std::vector<Step> steps;
  Step step;
  step.setId("01");
  // Footstep
  free_gait_msgs::Footstep footstep_msg;
  //  Position start(0.0, 0.0, 0.0);
//  Position target(0.3, 0.0, 0.0);
//  Vector sufaceNormal(0, 0, 1);
  geometry_msgs::PointStamped target;
  geometry_msgs::Vector3Stamped surface_normal;
  target.point.x = 0.3;
  target.point.y = 0.0;
  target.point.z = 0.0;
  surface_normal.vector.x = 0.0;
  surface_normal.vector.y = 0.0;
  surface_normal.vector.z = 1.0;
//  footstep.updateStartPosition(start);
  footstep_msg.name = "LF_LEG";
  footstep_msg.target = target;
  footstep_msg.average_velocity = 0.15;
  footstep_msg.profile_height = 0.05;
  footstep_msg.profile_type = "triangle";
  footstep_msg.ignore_contact = false;
  footstep_msg.ignore_for_pose_adaptation = false;
  footstep_msg.surface_normal = surface_normal;

  Footstep footstep(LimbEnum::LF_LEG);
  converter.fromMessage(footstep_msg, footstep);
  step.addLegMotion(footstep);
  // Basemotion
  free_gait_msgs::BaseAuto baseauto_msg;
  baseauto_msg.height = 0.4;
  baseauto_msg.average_angular_velocity = 0.0;
  baseauto_msg.average_linear_velocity = 0.0;
  baseauto_msg.support_margin = 0.0;
  baseauto_msg.ignore_timing_of_leg_motion = false;
  BaseAuto baseauto;
  converter.fromMessage(baseauto_msg, baseauto);
  step.addBaseMotion(baseauto);
  // in to step queue
  steps.push_back(step);

  executor.getQueue().add(steps);
  executor.setPreemptionType(Executor::PreemptionType::PREEMPT_IMMEDIATE);
  double dt = 0.001;
  ASSERT_TRUE(executor.advance(dt, false));
//  footstep.setTargetPosition("map", target);
//  footstep.setProfileHeight(0.05);
//  footstep.setProfileType("triangle");
//  footstep.setAverageVelocity(0.15);



}
