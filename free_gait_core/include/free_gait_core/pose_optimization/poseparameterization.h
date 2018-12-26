/*
 * PoseParameterization.hpp
 *
 *  Created on: Mar 22, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"

#include "qp_solver/quadraticproblemsolver.h"

namespace free_gait {

class PoseParameterization //: public numopt_common::Parameterization
{
 public:
  PoseParameterization();
  virtual ~PoseParameterization();

//  PoseParameterization(const PoseParameterization& other);

  qp_solver::QuadraticProblemSolver::parameters& getParams();
  const qp_solver::QuadraticProblemSolver::parameters& getParams() const;

  bool plus(qp_solver::QuadraticProblemSolver::parameters& result,
            const qp_solver::QuadraticProblemSolver::parameters& p,
            const qp_solver::QuadraticProblemSolver::Delta& dp) const;

  bool getTransformMatrixLocalToGlobal(Eigen::MatrixXd& matrix,
                                       const qp_solver::QuadraticProblemSolver::parameters& params) const;

  bool getTransformMatrixGlobalToLocal(Eigen::MatrixXd& matrix,
                                       const qp_solver::QuadraticProblemSolver::parameters& params) const;

  int getGlobalSize() const;
  static const size_t getGlobalSizeStatic();
  int getLocalSize() const;

  bool setRandom(qp_solver::QuadraticProblemSolver::parameters& p) const;
  bool setIdentity(qp_solver::QuadraticProblemSolver::parameters& p) const;

//  Parameterization* clone() const;

  const Pose getPose() const;
  void setPose(const Pose& pose);
  const Position getPosition() const;
  const RotationQuaternion getOrientation() const;

 private:
  const static size_t nTransGlobal_ = 3;
  const static size_t nRotGlobal_ = 4;
  const static size_t nTransLocal_ = 3;
  const static size_t nRotLocal_ = 3;

  //! Global state vector with position and orientation.
  qp_solver::QuadraticProblemSolver::parameters params_;
};

} /* namespace free_gait */
