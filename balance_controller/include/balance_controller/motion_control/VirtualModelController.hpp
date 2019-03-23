/*******************************************************************************************
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
* @file     VirtualModelController.hpp
* @author   Péter Fankhauser, Christian Gehring
* @date     Aug 6, 2013
* @brief
*/
#ifndef LOCO_VIRTUALMODELCONTROLLER_HPP_
#define LOCO_VIRTUALMODELCONTROLLER_HPP_

// Motion Controller
#include "balance_controller/motion_control/MotionControllerBase.hpp"
// Contact force distribution
#include "balance_controller/contact_force_distribution/ContactForceDistributionBase.hpp"
// Locomotion controller commons
//#include "loco/common/TypeDefs.hpp"
#include "free_gait_core/TypeDefs.hpp"
// Eigen
#include <Eigen/Core>
// Cout
#include <iostream>
#include "tinyxml.h"

#include "tinyxml.h"

namespace balance_controller {

class VirtualModelController : public MotionControllerBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  /*!
   * Constructor.
   * @param robotModel the reference to the robot
   */
  VirtualModelController(const ros::NodeHandle& node_handle,
                         std::shared_ptr<free_gait::State> robot_state,
                         std::shared_ptr<ContactForceDistributionBase> contactForceDistribution);
  /*!
   * Destructor.
   */
  virtual ~VirtualModelController();

  /*!
   * Load parameters.
   * @return true if successful
   */
  virtual bool loadParameters(const TiXmlHandle& handle);
  /*!
   * \brief loadParameters from parameters server
   * \return
   */
  virtual bool loadParameters();

  /*!
   * Add data to logger (optional).
   * @return true if successful.
   */
  bool addToLogger();

  /*!
   * Computes the joint torques from the desired base pose.
   * @return true if successful.
   */
  bool compute();


  friend std::ostream& operator << (std::ostream& out, const VirtualModelController& motionController);


  Force getDesiredVirtualForceInBaseFrame() const;
  Torque getDesiredVirtualTorqueInBaseFrame() const;
  void getDistributedVirtualForceAndTorqueInBaseFrame(Force& netForce, Torque& netTorque) const;

  const Eigen::Vector3d& getProportionalGainTranslation() const;
  const Eigen::Vector3d& getDerivativeGainTranslation() const;
  const Eigen::Vector3d& getFeedforwardGainTranslation() const;

  const Eigen::Vector3d& getProportionalGainRotation() const;
  const Eigen::Vector3d& getDerivativeGainRotation() const;
  const Eigen::Vector3d& getFeedforwardGainRotation() const;

  void setProportionalGainTranslation(const Eigen::Vector3d& gains);
  void setDerivativeGainTranslation(const Eigen::Vector3d& gains);
  void setFeedforwardGainTranslation(const Eigen::Vector3d& gains);

  void setProportionalGainRotation(const Eigen::Vector3d& gains);
  void setDerivativeGainRotation(const Eigen::Vector3d& gains);
  void setFeedforwardGainRotation(const Eigen::Vector3d& gains);

  void setGainsHeading(double kp, double kd, double kff);
  void setGainsLateral(double kp, double kd, double kff);
  void setGainsVertical(double kp, double kd, double kff);
  void setGainsRoll(double kp, double kd, double kff);
  void setGainsPitch(double kp, double kd, double kff);
  void setGainsYaw(double kp, double kd, double kff);

  void getGainsHeading(double& kp, double& kd, double& kff);
  void getGainsLateral(double& kp, double& kd, double& kff);
  void getGainsVertical(double& kp, double& kd, double& kff);
  void getGainsRoll(double& kp, double& kd, double& kff);
  void getGainsPitch(double& kp, double& kd, double& kff);
  void getGainsYaw(double& kp, double& kd, double& kff);

  double getGravityCompensationForcePercentage() const;
  void setGravityCompensationForcePercentage(double percentage);
  const ContactForceDistributionBase& getContactForceDistribution() const;

  /*! Sets the parameters to the interpolated ones between motionController1 and controller2.
   * @param motionController1     If the interpolation parameter is 0, then the parameter set is equal to the one of motionController1.
   * @param motionController2     If the interpolation parameter is 1, then the parameter set is equal to the one of motionController2.
   * @param t                     interpolation parameter in [0, 1]
   * @return                      true if successful
   */
  virtual bool setToInterpolated(const MotionControllerBase& motionController1, const MotionControllerBase& motionController2, double t);

 private:
  std::shared_ptr<ContactForceDistributionBase> contactForceDistribution_;

  //! Base position error in base frame.
  Position positionErrorInControlFrame_;
  //! Base orientation error vector (rotation vector) in base frame.
  Eigen::Vector3d orientationError_;
  //! Base linear velocity error in base frame.
  LinearVelocity linearVelocityErrorInControlFrame_;
  //! Base angular velocity error in base frame.
  LocalAngularVelocity angularVelocityErrorInControlFrame_;

  //! Force on torso to compensate for gravity (in base frame).
  Force gravityCompensationForceInBaseFrame_;
  //! Torque on torso to compensate for gravity (in base frame).
  Torque gravityCompensationTorqueInBaseFrame_;

  //! Proportional gain vector (k_p)  for translational error (force).
  Eigen::Vector3d proportionalGainTranslation_;
  //! Derivative gain vector (k_d) for translational error (force).
  Eigen::Vector3d derivativeGainTranslation_;
  //! Feedforward gain vector (k_ff) for translational error (force).
  Eigen::Vector3d feedforwardGainTranslation_;

  //! Proportional (k_p), derivative (k_d), and feedforward gain vector (k_ff) for rotational error (torque).
  Eigen::Vector3d proportionalGainRotation_;
  Eigen::Vector3d derivativeGainRotation_;
  Eigen::Vector3d feedforwardGainRotation_;

  double gravityCompensationForcePercentage_;

  //! Desired virtual force on base in base frame (B_F_B^d).
  Force virtualForceInBaseFrame_;
  //! Desired virtual torque on base in base frame (B_T_B^d).
  Torque virtualTorqueInBaseFrame_;
  std::vector<free_gait::LimbEnum> limbs_;

  ros::NodeHandle node_handle_;
 private:
  /*!
   * Compute error between desired an actual robot pose and twist.
   * @return true if successful.
   */
  bool computeError();

  /*!
   * Computes the gravity compensation force and torque.
   * @return true if successful.
   */
  bool computeGravityCompensation();

  //! Compute virtual force based on the form:
  //! F = k_p*(q_d-q) + k_d*(q_dot_d-q_dot) + k_ff*(...).
  //! In base frame.
  bool computeVirtualForce();

  //! Compute virtual torque based on the form:
  //! T = k_p*(q_d-q) + k_d*(q_dot_d-q_dot) + k_ff*(...).
  //! In base frame.
  bool computeVirtualTorque();

  /*!
   * Check if parameters are loaded.
   * @return true if parameters are loaded
   */
  bool isParametersLoaded() const;
};

} /* namespace loco */
#endif /* VIRTUALMODELCONTROLLER_H_ */
