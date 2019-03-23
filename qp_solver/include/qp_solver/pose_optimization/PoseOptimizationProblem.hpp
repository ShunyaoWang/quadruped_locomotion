/*
 * PoseOptimizationProblem.hpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "qp_solver/pose_optimization/PoseOptimizationObjectiveFunction.hpp"
#include "qp_solver/pose_optimization/PoseOptimizationFunctionConstraints.hpp"

//#include <numopt_common/ConstrainedNonlinearProblem.hpp>

namespace free_gait {

class PoseOptimizationProblem //: public numopt_common::ConstrainedNonlinearProblem
{
 public:
  PoseOptimizationProblem(std::shared_ptr<PoseOptimizationObjectiveFunction> objectiveFunction,
                          std::shared_ptr<PoseOptimizationFunctionConstraints> functionConstraints);
  virtual ~PoseOptimizationProblem();
  std::shared_ptr<PoseOptimizationObjectiveFunction> objectiveFunction_;
  std::shared_ptr<PoseOptimizationFunctionConstraints> functionConstraints_;
};

} /* namespace */
