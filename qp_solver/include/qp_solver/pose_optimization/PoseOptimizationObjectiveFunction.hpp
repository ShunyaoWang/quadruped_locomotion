/*
 * PoseOptimizationObjectiveFunction.hpp
 *
 *  Created on: Mar 22, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "free_gait_core/executor/AdapterBase.hpp"
#include "free_gait_core/executor/State.hpp"

#include <grid_map_core/Polygon.hpp>
#include "qp_solver/quadraticproblemsolver.h"
//#include <numopt_common/NonlinearObjectiveFunction.hpp>
#include "qp_solver/pose_optimization/poseparameterization.h"
#include <memory>
using namespace qp_solver;
namespace free_gait {

class PoseOptimizationObjectiveFunction : public qp_solver::QuadraticObjectiveFunction//public numopt_common::NonlinearObjectiveFunction
{
 public:
  /*!
   * Initialize the pose optimization objective function.
   * Note: The robot model behind the adapter is internally changed, make sure to give it
   * an adapter only used for this optimization.
   * @param adapter the Free Gait adapter.
   */
  PoseOptimizationObjectiveFunction();
  virtual ~PoseOptimizationObjectiveFunction();

  /*!
   * Set the positions of the feet (stance) of the robot in world coordinate system.
   * @param stance the feet positions.
   */
  void setStance(const Stance& stance);
  const Stance& getStance() const;

  /*!
   * Define the desired leg configuration by specifying the desired feet positions
   * relative to the base.
   * @param nominalStanceInBaseFrame the desired feet positions in base frame.
   */
  void setNominalStance(const Stance& nominalStanceInBaseFrame);
  const Stance& getNominalStance() const;

  void setInitialPose(const Pose& pose);
  void setSupportRegion(const grid_map::Polygon& supportRegion);
  void setCenterOfMass(const Position& centerOfMassInBaseFrame);

  /*! This method computes the objective value
   * @param value       function value
   * @param p           parameters
   * @param newParams   true if this class has already seen the parameters
   * @return  true if successful
   */
  bool computeValue(double& value, const PoseParameterization& params,
                    bool newParams = true);


  /*! Computes the local gradient of the objective function.
   * @param gradient    gradient (column vector)
   * @param p           parameters
   * @param newParams   true if this class has already seen the parameters
   * @returns true if successful
   */
  bool getLocalGradient(Eigen::VectorXd& gradient, const PoseParameterization& params,
                        bool newParams = true);

  /*! Computes the local nxn Hessian matrix of the objective function.
   * Note that this function does not need to update the list of triplets.
   *
   * @param hessian     Hessian matrix
   * @param p           parameters
   * @param newParams   true if this class has already seen the parameters
   * @returns true if successful
   */
  bool getLocalHessian(Eigen::MatrixXd& hessian, const PoseParameterization& params,
                       bool newParams = true);

 private:
  Stance stance_;
  Stance nominalStanceInBaseFrame_;
  Pose initialPose_;
  grid_map::Polygon supportRegion_;
  Position centerOfMassInBaseFrame_;
  const double comWeight_; // w_2
};

} /* namespace free_gait */
