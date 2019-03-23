/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Péter Fankhauser, Christian Gehring, C. Dario Bellicoso, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*!
* @file     ContactForceDistribution.hpp
* @author   Péter Fankhauser, Christian Gehring
* @date     Aug 6, 2013
* @brief
*/
#pragma once

#include "ContactForceDistributionBase.hpp"
//#include "loco/common/LegBase.hpp"
//#include "loco/common/TorsoBase.hpp"
#include <qp_solver/quadraticproblemsolver.h>
#include "free_gait_core/pose_optimization/PoseOptimizationBase.hpp"
#include <Eigen/SparseCore>
#include "tinyxml.h"

namespace balance_controller {

//! This class distributes a virtual force and torque on the base as forces to the leg contact points.
/*!
 * Based on 'Control of Dynamic Gaits for a Quadrupedal Robot', C. Gehring, ICRA, 2013.
 *
 * The optimization problem is formulated as:
 *
 * [ I    I   ...] [f1] = [F] ==> A*x = b
 * [r1x  r2x  ...] [f2]   [T]
 *                 [ .]
 *                 [ .]
 *                 [ .]
 */
class ContactForceDistribution : public ContactForceDistributionBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct LegInfo
  {
    bool isPartOfForceDistribution_;
    bool isLoadConstraintActive_;
    int indexInStanceLegList_;
    int startIndexInVectorX_;
    Force desiredContactForce_;
    //! for logging
    Vector firstDirectionOfFrictionPyramidInWorldFrame_;
    //! for logging
    Vector secondDirectionOfFrictionPyramidInWorldFrame_;
    //! for logging
    Vector normalDirectionOfFrictionPyramidInWorldFrame_;
    //! Assumed friction coefficient (mu).
    double frictionCoefficient_;

  };

 public:

  /*!
   * Constructor.
   */
  ContactForceDistribution(const ros::NodeHandle& node_handle,
                           std::shared_ptr<free_gait::State> robot_state);

  /*!
   * Destructor.
   */
  virtual ~ContactForceDistribution();

  /*!
   * Loads the parameters. Has to be done before using this class.
   * @return true if successful.
   */
  virtual bool loadParameters(const TiXmlHandle& handle);
  /*!
   * \brief loadParameters from ros parameters server
   * \return
   */
  virtual bool loadParameters();
  /*!
   * Adds class data to the logger (optional).
   * @return true if successful.
   */
  bool addToLogger();

  /*!
   * Computes the contact force distribution of the virtual force and torque.
   * @param virtualForce the desired virtual force on the base (in base frame).
   * @param virtualTorque the desired virtual torque on the base (in base frame).
   * @return true if successful.
   */
  virtual bool computeForceDistribution(const Force& virtualForceInBaseFrame,
                                const Torque& virtualTorqueInBaseFrame);

  /*!
   * Gets the distributed net forces and torques that act on the base, i.e.
   * this force and torque are computed from the distributed contact forces and
   * should ideally be equal to the desired net forces and torques.
   * @param[out] netForce.
   * @param[out] netTorque.
   * @return true if net force and torque can be calculated, false otherwise.
   */
   bool getNetForceAndTorqueOnBase(Force& netForce, Torque& netTorque);

   double getGroundForceWeight() const;
   double getMinimalNormalGroundForce() const;
   double getVirtualForceWeight(int index) const;

   const Vector& getFirstDirectionOfFrictionPyramidInWorldFrame(free_gait::LimbEnum leg) const;
   const Vector& getSecondDirectionOfFrictionPyramidInWorldFrame(free_gait::LimbEnum leg) const;
   const Vector& getNormalDirectionOfFrictionPyramidInWorldFrame(free_gait::LimbEnum leg) const;
   double getFrictionCoefficient(free_gait::LimbEnum leg) const;
//   double getFrictionCoefficient(const free_gait::LimbEnum leg) const;
   double getFrictionCoefficient(int index) const;

   const LegInfo& getLegInfo(free_gait::LimbEnum leg) const;

   virtual bool setToInterpolated(const ContactForceDistributionBase& contactForceDistribution1, const ContactForceDistributionBase& contactForceDistribution2, double t);

//   const LegGroup* getLegs() const;
 private:

  //! Number of legs in stance phase
  int nLegsInForceDistribution_;
  int footDof_;
  //! Number of variables to optimize (size of x, n = nTranslationalDofPerFoot_ * nLegsInStance_)
  int n_;

  //! Diagonal elements of the weighting matrix for the desired virtual forces and torques (for S).
  Eigen::Matrix<double, nElementsVirtualForceTorqueVector_, 1> virtualForceWeights_;

  //! Diagonal element of the weighting matrix for the ground reaction forces (regularizer, for W).
  double groundForceWeight_;
  //! Minimal normal ground force (F_min^n, in N).
  double minimalNormalGroundForce_;

  //! Stacked contact forces (in base frame)
  Eigen::VectorXd x_;
  //! The matrix A in the optimization formulation (nElementsInStackedVirtualForceTorqueVector_ x n).
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_;
  //! The vector b in the optimization formulation (stacked vector of desired net virtual forces and torques).
  Eigen::VectorXd b_;
  //! Weighting matrix for the ground reaction forces (regularizer).
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> W_;
  //! Weighting matrix for the desired virtual forces and torques.
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> S_;
  //! Force inequality constraint matrix
  Eigen::SparseMatrix<double, Eigen::RowMajor> D_;
  //! Upper and lower limits vectors of force inequality constraint
  Eigen::VectorXd d_, f_;
  //! Force equality constraint matrix
  Eigen::SparseMatrix<double, Eigen::RowMajor> C_;
  //! Vector of force equality constraint
  Eigen::VectorXd c_;

  std::vector<free_gait::LimbEnum> limbs_;

  std::unique_ptr<qp_solver::QuadraticProblemSolver> solver_;
  std::unique_ptr<qp_solver::QuadraticObjectiveFunction> cost_function_;
  std::unique_ptr<qp_solver::LinearFunctionConstraints> constraints_;

  ros::NodeHandle node_handle_;

public:
//  std::map<LegBase*, LegInfo> legInfos_;
  std::map<free_gait::LimbEnum, LegInfo> legInfos_;
private:


  /*!
   * Reads foot contact flags and includes user leg load settings from changeLegLoad().
   * @return true if successful.
   */
  bool prepareLegLoading();

  /*!
   * Prepare matrices for the optimization problem.
   * @return true if successful
   */
  bool prepareOptimization(const Force& virtualForce,
                           const Torque& virtualTorque);


  bool addMinimalForceConstraints();

  bool addFrictionConstraints();

  bool addDesiredLegLoadConstraints();

  /*!
   * Solve optimization
   * @return true if successful
   */
  bool solveOptimization();

 /*!
  * Calculate the joint torques from the desired contact forces
  * with Jacobi transpose.
  */
  bool computeJointTorques();

  bool resetOptimization();

  /*!
   * Update the data that is recorded by the logger.
   * @return true if successful, false if it is not logging.
   */
  bool updateLoggerData();
};

} /* namespace loco */
