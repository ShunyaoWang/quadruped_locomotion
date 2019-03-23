/*
 * PoseOptimizationFunctionConstraints.hpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/executor/AdapterBase.hpp"
#include "free_gait_core/executor/State.hpp"
#include "qp_solver/pose_optimization/PoseConstraintsChecker.hpp"
#include "qp_solver/quadraticproblemsolver.h"
#include "qp_solver/pose_optimization/poseparameterization.h"
//#include <numopt_common/NonlinearFunctionConstraints.hpp>
#include <grid_map_core/Polygon.hpp>

#include <map>

namespace free_gait {

class PoseOptimizationFunctionConstraints : public qp_solver::LinearFunctionConstraints//public numopt_common::NonlinearFunctionConstraints
{
 public:
  using LimbLengths = PoseConstraintsChecker::LimbLengths;
  typedef std::map<LimbEnum, Position> LegPositions;

  PoseOptimizationFunctionConstraints();
  virtual ~PoseOptimizationFunctionConstraints();

  void setStance(const Stance& stance);
  void setSupportRegion(const grid_map::Polygon& supportRegion);

  void setLimbLengthConstraints(const LimbLengths& minLimbLenghts,
                                const LimbLengths& maxLimbLenghts);

  void setPositionsBaseToHip(const LegPositions& positionBaseToHipInBaseFrame);

  void setCenterOfMass(const Position& centerOfMassInBaseFrame);

  bool getGlobalBoundConstraintMinValues(Eigen::VectorXd& values);
  bool getGlobalBoundConstraintMaxValues(Eigen::VectorXd& values);

  //! d <= c(p) <= f
  bool getInequalityConstraintValues(Eigen::VectorXd& values,
                                     const PoseParameterization& p,
                                     bool newParams = true);
  bool getInequalityConstraintMinValues(Eigen::VectorXd& d);
  bool getInequalityConstraintMaxValues(Eigen::VectorXd& f);

  bool getLocalInequalityConstraintJacobian(Eigen::MatrixXd& jacobian,
                                            const PoseParameterization& params, bool newParams = true);

 private:
  void updateNumberOfInequalityConstraints();

  Stance stance_;
  Stance supportStance_;
  LegPositions positionsBaseToHipInBaseFrame_;
  Position centerOfMassInBaseFrame_;

  size_t nSupportRegionInequalityConstraints_;
  grid_map::Polygon supportRegion_;

  size_t nLimbLengthInequalityConstraints_;
  Eigen::VectorXd limbLengthInequalityConstraintsMinValues_;
  Eigen::VectorXd limbLengthInequalityConstraintsMaxValues_;

  //! A from A*x <= b.
  Eigen::MatrixXd supportRegionInequalityConstraintGlobalJacobian_;
  //! b from A*x <= b.
  Eigen::VectorXd supportRegionInequalityConstraintsMaxValues_;
};

} /* namespace */
