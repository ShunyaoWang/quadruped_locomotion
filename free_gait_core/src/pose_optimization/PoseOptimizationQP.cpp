/*
 * PoseOptimizationQP.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/pose_optimization/PoseOptimizationQP.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <kindr/Core>
//#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>
//#include <numopt_common/ParameterizationIdentity.hpp>

using namespace Eigen;
using namespace std;
namespace free_gait {

PoseOptimizationQP::PoseOptimizationQP(const AdapterBase& adapter)
    : PoseOptimizationBase(adapter),
      nStates_(3),
      nDimensions_(3)
{
//  solver_.reset(new numopt_quadprog::ActiveSetFunctionMinimizer());
}

PoseOptimizationQP::~PoseOptimizationQP()
{
}

//PoseOptimizationQP::PoseOptimizationQP(const PoseOptimizationQP& other)
//    : PoseOptimizationBase(other),
//      nStates_(other.nStates_),
//      nDimensions_(other.nDimensions_)
//{
////  solver_.reset(new numopt_quadprog::ActiveSetFunctionMinimizer());
//}

bool PoseOptimizationQP::optimize(Pose& pose)
{
  checkSupportRegion();

  state_.setPoseBaseToWorld(pose); //TODO(Shunyao): fix bug in state class
  adapter_.setInternalDataFromState(state_);
//  adapter_.setInternalDataFromState(state_, false, true, false, false); // To guide IK.
//  updateJointPositionsInState(state_);
//  adapter_.setInternalDataFromState(state_, false, true, false, false);

  // Compute center of mass.
  const Position centerOfMassInBaseFrame(
      adapter_.transformPosition(adapter_.getWorldFrameId(), adapter_.getBaseFrameId(),
                                 adapter_.getCenterOfMassInWorldFrame()));
  std::cout<<"COM is :"<<centerOfMassInBaseFrame<<std::endl;

  // Problem definition:
  // min Ax - b, Gx <= h, x is base center (x,y,z),minimaze foothold offsets(I_r_F_hat - I_r_F)
  unsigned int nFeet = stance_.size();
  MatrixXd A = MatrixXd::Zero(nDimensions_ * nFeet, nStates_);
  VectorXd b = VectorXd::Zero(nDimensions_ * nFeet);
  Matrix3d R = RotationMatrix(pose.getRotation()).matrix();
  std::cout<<"QP optimization stance :"<<stance_<<std::endl;
  unsigned int i = 0;
  for (const auto& footPosition : stance_) {
    A.block(nDimensions_ * i, 0, nStates_, A.cols()) << Matrix3d::Identity();
    b.segment(nDimensions_ * i, nDimensions_) << footPosition.second.vector() - R * nominalStanceInBaseFrame_[footPosition.first].vector();
    ++i;
  }

//  std::cout << "R: " << std::endl << R << std::endl;
//  std::cout << "A: " << std::endl << A << std::endl;
//  std::cout << "b: " << std::endl << b << std::endl;

  // Inequality constraints.
  Eigen::MatrixXd G;
  Eigen::VectorXd hp;
  supportRegion_.convertToInequalityConstraints(G, hp);
  Eigen::VectorXd h = hp - G * (R * centerOfMassInBaseFrame.vector()).head(2);
  G.conservativeResize(Eigen::NoChange,3); // Add column corresponding to Z position
  G.col(2).setZero(); // No constraints in Z position

//  std::cout << "G: " << std::endl << G << std::endl;
//  std::cout << "hp: " << std::endl << hp << std::endl;
//  std::cout << "h: " << std::endl << h << std::endl;

  // Formulation as QP:
  // min 1/2 x'Px + q'x + r
  Eigen::MatrixXd P = 2 * A.transpose() * A;
  Eigen::VectorXd q = -2 * A.transpose() * b;

//  MatrixXd r = b.transpose() * b; // Not used.

  // Cost function.
  //auto costFunction = std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(new numopt_common::QuadraticObjectiveFunction());
  auto costFunction = std::shared_ptr<qp_solver::QuadraticObjectiveFunction>(new qp_solver::QuadraticObjectiveFunction());

  Eigen::MatrixXd P_sparse = P.sparseView();
  costFunction->setGlobalHessian(P_sparse);
  costFunction->setLinearTerm(q);

  // Constraints.
  //auto constraints = std::shared_ptr<numopt_common::LinearFunctionConstraints>(new numopt_common::LinearFunctionConstraints());
  auto constraints = std::shared_ptr<qp_solver::LinearFunctionConstraints>(new qp_solver::LinearFunctionConstraints());
  Eigen::MatrixXd G_sparse = G.sparseView();
  Eigen::MatrixXd Aeq(P.cols(), 1);
  Eigen::VectorXd beq(1);
  constraints->setGlobalInequalityConstraintJacobian(G_sparse);
  //constraints->setInequalityConstraintMinValues(std::numeric_limits<double>::lowest() * numopt_common::Vector::Ones(h.size()));
  constraints->setInequalityConstraintMaxValues(h);
  constraints->setGlobalEqualityConstraintJacobian(Aeq.setZero());
  constraints->setEqualityConstraintMaxValues(beq.setZero());
  // BUG(shunyao,29/11/18) only the inequality max value is equal to Matlab,
  // considering the -CI ?(fixed, optimization object different, here only for position(x,y,z))

//  cout<<"Hessian Matrix is: "<<endl<<P_sparse<<endl;
//  cout<<"Jacobian Vector is: "<<endl<<q<<endl;
//  cout<<"Inequality Constraints Jacobian is: "<<endl<<G<<endl;
//  cout<<"Equality Constraints Jacobian is: "<<endl<<Aeq<<endl;
//  cout<<"Inequality Constraints value vector is: "<<endl<<h<<endl;
//  cout<<"Equality Constraints value vector is: "<<endl<<beq<<endl;

  // Solve.
  /*numopt_common::QuadraticProblem problem(costFunction, constraints);

  Eigen::VectorXd x;
  numopt_common::ParameterizationIdentity params(x.size());
  params.getParams() = x;
  double cost = 0.0;
  if (!solver_->minimize(&problem, params, cost)) return false;
  x = params.getParams();
  std::cout << "x: " << std::endl << x << std::endl;*/
  Eigen::VectorXd params(P.cols());
  if (!solver_->minimize(*costFunction, *constraints, params)) return false;
  // Return optimized pose.
  std::cout << "quadratic solution is : " << std::endl << params << std::endl;
  pose.getPosition().vector() = params;
  return true;
}

} /* namespace */
