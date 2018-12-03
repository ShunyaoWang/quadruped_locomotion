/*
 * PoseParameterization.cpp
 *
 *  Created on: Mar 22, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "qp_solver/pose_optimization/poseparameterization.h"

namespace free_gait {

PoseParameterization::PoseParameterization()
{
  setIdentity(params_);
}

PoseParameterization::~PoseParameterization()
{
}

//PoseParameterization::PoseParameterization(const PoseParameterization& other)
//    : params_(other.params_)
//{
//}

qp_solver::QuadraticProblemSolver::parameters& PoseParameterization::getParams()
{
  return params_;
}

const qp_solver::QuadraticProblemSolver::parameters& PoseParameterization::getParams() const
{
  return params_;
}

bool PoseParameterization::plus(qp_solver::QuadraticProblemSolver::parameters& result,
                                const qp_solver::QuadraticProblemSolver::parameters& p,
                                const qp_solver::QuadraticProblemSolver::Delta& dp) const
{
  // Position.
    result.head(nTransGlobal_) = p.head(nTransGlobal_) + dp.head(nTransLocal_);

  //  // Orientation.
  ////  result.tail(nRotGlobal_) = RotationQuaternion(p.tail(nRotGlobal_)).boxPlus(
  ////      dp.tail(nRotLocal_)).vector();
//  Eigen::VectorXd p1(7);
//    result.tail(nRotGlobal_) = RotationQuaternion(p1.tail(nRotGlobal_)).boxPlus(dp.tail(nRotLocal_)).vector());
      result.tail(nRotGlobal_) = RotationQuaternion(p.tail(nRotGlobal_)).boxPlus(dp.tail(nRotLocal_)).vector();
    return true;
}

bool PoseParameterization::getTransformMatrixLocalToGlobal(Eigen::MatrixXd& matrix,
                                                           const qp_solver::QuadraticProblemSolver::parameters& params) const
{
  Eigen::MatrixXd denseMatrix(Eigen::MatrixXd::Zero(getGlobalSize(), getLocalSize()));
  denseMatrix.topLeftCorner(nTransGlobal_, nTransLocal_).setIdentity();
  denseMatrix.bottomRightCorner(nRotGlobal_, nRotLocal_) =
      0.5 * RotationQuaternion(params.tail(nRotGlobal_))
           .getLocalQuaternionDiffMatrix().transpose();
  matrix = denseMatrix.sparseView();
  return true;
}

bool PoseParameterization::getTransformMatrixGlobalToLocal(
    Eigen::MatrixXd& matrix, const qp_solver::QuadraticProblemSolver::parameters& params) const
{
  Eigen::MatrixXd denseMatrix(Eigen::MatrixXd::Zero(getLocalSize(), getGlobalSize()));
  denseMatrix.topLeftCorner(nTransLocal_, nTransGlobal_).setIdentity();
  denseMatrix.bottomRightCorner(nRotLocal_, nRotGlobal_) = 2.0
      * RotationQuaternion(params.tail(nRotGlobal_)).getLocalQuaternionDiffMatrix();
  matrix = denseMatrix.sparseView();
  return true;
}

int PoseParameterization::getGlobalSize() const
{
  return nTransGlobal_ + nRotGlobal_;
}

const size_t PoseParameterization::getGlobalSizeStatic()
{
  return nTransGlobal_ + nRotGlobal_;
}

int PoseParameterization::getLocalSize() const
{
  return nTransLocal_ + nRotLocal_;
}

bool PoseParameterization::setRandom(qp_solver::QuadraticProblemSolver::parameters& p) const
{
  p.resize(getGlobalSize());
  p.head(nTransGlobal_).setRandom();
  RotationQuaternion randomQuaternion;
  randomQuaternion.setIdentity(); //TODO(shunyao): set random
  p.tail(nRotGlobal_) = randomQuaternion.vector();
  return true;
}

bool PoseParameterization::setIdentity(qp_solver::QuadraticProblemSolver::parameters& p) const
{
  p.resize(getGlobalSize());
  p.head(nTransGlobal_).setZero();
  RotationQuaternion identityQuaternion;
  identityQuaternion.setIdentity();
  std::cout<<identityQuaternion.vector()(0)<<std::endl;
  p.tail(nRotGlobal_) = identityQuaternion.vector();
  return true;
}

//numopt_common::Parameterization* PoseParameterization::clone() const
//{
//  Parameterization* clone = new PoseParameterization(*this);
//  return clone;
//}

const Pose PoseParameterization::getPose() const
{
  return Pose(getPosition(), getOrientation());
}

void PoseParameterization::setPose(const Pose& pose)
{
  params_.head(nTransGlobal_) = pose.getPosition().vector();
  params_.tail(nRotGlobal_) = pose.getRotation().vector();
}

const Position PoseParameterization::getPosition() const
{
  return Position(params_.head(nTransGlobal_));
}

const RotationQuaternion PoseParameterization::getOrientation() const
{
  return RotationQuaternion(params_.tail(nRotGlobal_));
}

} /* namespace */
