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

// gtest
#include <gtest/gtest.h>

using namespace free_gait;

TEST(footstep, triangleLowLongStep)
{
  Footstep footstep(LimbEnum::LF_LEG);
  Position start(0.0, 0.0, 0.0);
  Position target(0.3, 0.0, 0.0);
  footstep.updateStartPosition(start);
  footstep.setTargetPosition("map", target);
  double height = 0.05;
  footstep.setProfileHeight(0.05);
  footstep.setProfileType("triangle");
  footstep.setAverageVelocity(0.15);
  ASSERT_TRUE(footstep.compute(true));

  for (double time = 0.0; time < footstep.getDuration(); time += 0.001) {
    Position position = footstep.evaluatePosition(time);
    EXPECT_LT(position.z(), height + 0.001);
    EXPECT_GT(position.x(), start.x() - 0.001);
    EXPECT_LT(position.x(), target.x() + 0.001);
  }
}

TEST(footstep, trianglehighLong)
{
  Footstep footstep(LimbEnum::LF_LEG);
  Position start(0.0, 0.0, 0.0);
  Position target(0.3, 0.0, 0.3);
  footstep.updateStartPosition(start);
  footstep.setTargetPosition("map", target);
  double height = 0.05;
  footstep.setProfileHeight(0.05);
  footstep.setProfileType("triangle");
  footstep.setAverageVelocity(0.15);
  ASSERT_TRUE(footstep.compute(true));

  for (double time = 0.0; time < footstep.getDuration(); time += 0.001) {
    Position position = footstep.evaluatePosition(time);
    EXPECT_LT(position.z(), target.z() + height + 0.01);
    EXPECT_GT(position.x(), start.x() - 0.001);
    EXPECT_LT(position.x(), target.x() + 0.001);
  }
}

//TEST(executor, footstepExecute)
//{
//  AdapterDummy adapter;
//  State state;
//  StepParameters parameters;
//  StepCompleter completer(parameters, adapter);
//  StepComputer computer;
//  Executor executor(completer, computer, adapter, state);
//  StepRosConverter converter(adapter);
//  std::vector<Step> steps;
//  Step step;
//  step.setId("01");
//  // Footstep
//  Footstep footstep(LimbEnum::LF_LEG);
////  Position start(0.0, 0.0, 0.0);
//  Position target(0.3, 0.0, 0.0);
//  Vector sufaceNormal(0, 0, 1);
////  footstep.updateStartPosition(start);
//  footstep.setTargetPosition("map", target);
//  footstep.setProfileHeight(0.05);
//  footstep.setProfileType("triangle");
//  footstep.setAverageVelocity(0.15);
//  footstep.surfaceNormal_.reset(new Vector(sufaceNormal));


//}
