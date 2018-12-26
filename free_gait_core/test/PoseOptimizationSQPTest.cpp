/*
 * PoseOptimizationSQPTest.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "free_gait_core/TypeDefs.hpp"
#include "qp_solver/pose_optimization/PoseOptimizationSQP.hpp"
#include "qp_solver/pose_optimization/PoseOptimizationObjectiveFunction.hpp"
#include "qp_solver/pose_optimization/poseparameterization.h"
#include "AdapterDummy.hpp"

#include <grid_map_core/Polygon.hpp>
#include <gtest/gtest.h>
#include <kindr/Core>
#include <kindr/common/gtest_eigen.hpp>
//#include <numopt_common/ParameterizationIdentity.hpp>
//#include <numopt_sqp/SQPFunctionMinimizer.hpp>
//#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>

#include <cmath>

using namespace free_gait;

TEST(PoseOptimizationSQP, PoseParameterization)
{
  PoseParameterization result, p;
  p.setRandom(p.getParams());
  Eigen::VectorXd dp = Eigen::VectorXd::Random(p.getLocalSize());
  result.plus(result.getParams(), p.getParams(), dp);
  Position translation = result.getPosition() - p.getPosition();
  RotationVector rotation(result.getOrientation().boxMinus(p.getOrientation()));
  kindr::assertNear(dp.head(3), translation.vector(), 1e-3, KINDR_SOURCE_FILE_POS);
  kindr::assertNear(dp.tail(3), rotation.vector(), 1e-3, KINDR_SOURCE_FILE_POS);
}

TEST(PoseOptimizationSQP, ObjectiveFunction)
{
  AdapterDummy adapter;
//  State state;
//  state.setPoseBaseToWorld(Pose(Position(0.0, 0.0, 0.1), RotationQuaternion()));
//  state.setRandom();
//  adapter.setInternalDataFromState(state);

  PoseOptimizationObjectiveFunction objective;
  Stance nominalStance;
  nominalStance[LimbEnum::LF_LEG] = Position(0.3, 0.2, -0.5);
  nominalStance[LimbEnum::RF_LEG] = Position(0.3, -0.2, -0.5);
  nominalStance[LimbEnum::LH_LEG] = Position(-0.3, 0.2, -0.5);
  nominalStance[LimbEnum::RH_LEG] = Position(-0.3, -0.2, -0.5);
  objective.setNominalStance(nominalStance);
  Stance currentStance;
  currentStance[LimbEnum::LF_LEG] = Position(0.3, 0.2, 0.0);
  currentStance[LimbEnum::RF_LEG] = Position(0.3, -0.2, 0.0);
  currentStance[LimbEnum::LH_LEG] = Position(-0.3, 0.2, 0.0);
  currentStance[LimbEnum::RH_LEG] = Position(-0.3, -0.2, 0.0);
  objective.setStance(currentStance);

  // Define support region.
  grid_map::Polygon supportRegion;
  std::vector<Position> footholdsOrdered;
  getFootholdsCounterClockwiseOrdered(currentStance, footholdsOrdered);
  for (auto foothold : footholdsOrdered) {
    supportRegion.addVertex(foothold.vector().head<2>());
  }
  objective.setSupportRegion(supportRegion);
  PoseParameterization params;

  params.setIdentity(params.getParams());
//  std::cout<<"Get Here"<<std::endl;
  double value;
  objective.computeValue(value, params);
  // Each leg has 0.5 m error.
//  std::cout<<"first value is : "<<value<<std::endl;
  EXPECT_EQ(4 * std::pow(0.5, 2), value);

  Eigen::VectorXd gradient;
  objective.getLocalGradient(gradient, params);
  std::cerr << gradient << std::endl;

  params.setPose(Pose(Position(0.0, 0.0, 0.1), RotationQuaternion()));
  objective.computeValue(value, params);
  // Each leg has 0.4 m error.
  EXPECT_EQ(4 * std::pow(0.4, 2), value);

  objective.getLocalGradient(gradient, params);
  std::cerr << gradient << std::endl;

  params.setPose(Pose(Position(2.0, 1.0, 0.0), RotationQuaternion()));
  supportRegion.removeVertices();
  footholdsOrdered.clear();
  for (auto& stance : currentStance) {
    stance.second += Position(2.0, 1.0, 0.0);
  }
  getFootholdsCounterClockwiseOrdered(currentStance, footholdsOrdered);
  for (auto foothold : footholdsOrdered) {
    supportRegion.addVertex(foothold.vector().head<2>());
  }
  objective.setSupportRegion(supportRegion);
  objective.setStance(currentStance);
  objective.computeValue(value, params);
  // Each leg has 0.4 m error.
  EXPECT_EQ(4 * std::pow(0.5, 2), value);

  objective.getLocalGradient(gradient, params);
  std::cerr << gradient << std::endl;
}

TEST(PoseOptimizationSQP, OptimizationSquareUp)
{
  AdapterDummy adapter;
  PoseOptimizationSQP optimization(adapter);
  PoseOptimizationBase::LimbLengths minLimbLenghts_, maxLimbLenghts_;
  optimization.setNominalStance(Stance({
    {LimbEnum::LF_LEG, Position(0.3, 0.2, -0.4)},
    {LimbEnum::RF_LEG, Position(0.3, -0.2, -0.4)},
    {LimbEnum::LH_LEG, Position(-0.3, 0.2, -0.4)},
    {LimbEnum::RH_LEG, Position(-0.3, -0.2, -0.4)} }));

  Stance stance;
  stance[LimbEnum::LF_LEG] = Position(0.3, 0.2, -0.1);
  stance[LimbEnum::RF_LEG] = Position(0.3, -0.2, -0.1);
  stance[LimbEnum::LH_LEG] = Position(-0.3, 0.2, -0.1);
  stance[LimbEnum::RH_LEG] = Position(-0.3, -0.2, -0.1);
  optimization.setStance(stance);
  optimization.setSupportStance(stance);
    // Define support region.
  grid_map::Polygon supportRegion;
  std::vector<Position> footholdsOrdered;
  getFootholdsCounterClockwiseOrdered(stance, footholdsOrdered);
  for (auto foothold : footholdsOrdered) {
    supportRegion.addVertex(foothold.vector().head<2>());
  }
  optimization.setSupportRegion(supportRegion);
  // Define min./max. leg lengths.
  for (const auto& limb : adapter.getLimbs()) {
    minLimbLenghts_[limb] = 0.2; // TODO Make as parameters.
    maxLimbLenghts_[limb] = 0.565; // Foot leaves contact. // 0.6
  }
  optimization.setLimbLengthConstraints(minLimbLenghts_, maxLimbLenghts_);

  Pose result = Pose(Position(0.0, 0.0, 0.3), RotationQuaternion());
  ASSERT_TRUE(optimization.optimize(result));

  Position expectedPosition(0.0, 0.0, 0.3);
  RotationMatrix expectedOrientation; // Identity.
  kindr::assertNear(expectedOrientation.matrix(), RotationMatrix(result.getRotation()).matrix(), 1e-2, KINDR_SOURCE_FILE_POS);
  kindr::assertNear(expectedPosition.vector(), result.getPosition().vector(), 1e-3, KINDR_SOURCE_FILE_POS);

  // Translation only.
  result.setIdentity();
  expectedPosition += Position(0.3, 0.2, 0.0);
  result = Pose(expectedPosition, RotationQuaternion());
  Stance translatedStance(stance);
  for (auto& stance : translatedStance) {
    stance.second += Position(expectedPosition.x(), expectedPosition.y(), 0.0);
  }
  footholdsOrdered.clear();
  supportRegion.removeVertices();
  getFootholdsCounterClockwiseOrdered(translatedStance, footholdsOrdered);
  for (auto foothold : footholdsOrdered) {
    supportRegion.addVertex(foothold.vector().head<2>());
  }
  optimization.setSupportRegion(supportRegion);

  optimization.setStance(translatedStance);
  ASSERT_TRUE(optimization.optimize(result));
  kindr::assertNear(expectedOrientation.matrix(), RotationMatrix(result.getRotation()).matrix(), 1e-2, KINDR_SOURCE_FILE_POS);
  kindr::assertNear(expectedPosition.vector(), result.getPosition().vector(), 1e-3, KINDR_SOURCE_FILE_POS);

  // Add yaw rotation.
  for (double yawRotation = 0.0; yawRotation <= 45.0; yawRotation += 10.0) {
    std::cerr << "yaw Rotation ====" << yawRotation << std::endl;
//    result.setIdentity();

    expectedOrientation = EulerAnglesZyx(yawRotation/180.0 * M_PI, 0.0, 0.0);
    result = Pose(expectedPosition, RotationQuaternion());
    Stance rotatedStance(stance);
    for (auto& stance : rotatedStance) {
      stance.second = expectedOrientation.rotate(stance.second);
      stance.second += Position(expectedPosition.x(), expectedPosition.y(), 0.0);
    }

    footholdsOrdered.clear();
    supportRegion.removeVertices();
    getFootholdsCounterClockwiseOrdered(rotatedStance, footholdsOrdered);
    for (auto foothold : footholdsOrdered) {
      supportRegion.addVertex(foothold.vector().head<2>());
    }
    optimization.setSupportRegion(supportRegion);


    optimization.setStance(rotatedStance);
    ASSERT_TRUE(optimization.optimize(result));
    kindr::assertNear(expectedOrientation.matrix(), RotationMatrix(result.getRotation()).matrix(), 1e-2, KINDR_SOURCE_FILE_POS);
    kindr::assertNear(expectedPosition.vector(), result.getPosition().vector(), 1e-2, KINDR_SOURCE_FILE_POS);
  }
}
