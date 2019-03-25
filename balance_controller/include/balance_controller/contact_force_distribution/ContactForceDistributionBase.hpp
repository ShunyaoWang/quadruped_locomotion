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
* @file     ContactForceDistributionBase.hpp
* @author   Péter Fankhauser, Christian Gehring
* @date     Aug 6, 2013
* @brief
*/
#pragma once

#include <Eigen/Core>
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/free_gait_core.hpp"
#include "balance_controller/math.hpp"

//#include "loco/common/TorsoBase.hpp"
//#include "loco/common/LegGroup.hpp"
//#include "loco/common/TerrainModelBase.hpp"
#include "tinyxml.h"

#include "ros/ros.h"

#include <memory>

namespace balance_controller {

//! This class distributes a virtual force and torque on the base as forces to the leg contact points.
class ContactForceDistributionBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Constructor.
   */
  ContactForceDistributionBase(const ros::NodeHandle& node_handle, std::shared_ptr<free_gait::State> robot_state);

  /*!
   * Destructor.
   */
  virtual ~ContactForceDistributionBase();

  /*!
   * Loads the parameters. Has to be done before using this class.
   * @return true if successful.
   */
  virtual bool loadParameters(const TiXmlHandle& handle) = 0;
  /*!
   * \brief loadParameters from ros parameters server
   * \return
   */
  virtual bool loadParameters() = 0;
  /*!
   * Adds class data to the logger (optional).
   * @return true if successful.
   */
  virtual bool addToLogger() = 0;

  /*!
   * Computes the contact force distribution of the virtual force and torque.
   * @param virtualForce the desired virtual force on the base (in base frame).
   * @param virtualTorque the desired virtual torque on the base (in base frame).
   * @return true if successful.
   */
  virtual bool computeForceDistribution(const Force& virtualForceInBaseFrame,
                                        const Torque& virtualTorqueInBaseFrame) = 0;

  /*!
   * Gets the distributed net forces and torques that act on the base, i.e.
   * this force and torque are computed from the distributed contact forces and
   * should ideally be equal to the desired net forces and torques.
   * @param[out] netForce.
   * @param[out] netTorque.
   * @return true if net force and torque can be calculated, false otherwise.
   */
   virtual bool getNetForceAndTorqueOnBase(Force& netForce, Torque& netTorque) = 0;

   /*! Computes an interpolated version of the two force distributions passed in as parameters.
    *  if t is 0, the current setting is set to contactForceDistribution1, 1 -> contactForceDistribution2, and values in between
    *  correspond to interpolated parameter set.
    * @param contactForceDistribution1
    * @param contactForceDistribution2
    * @param t interpolation parameter
    * @returns true if successful
    */
   virtual bool setToInterpolated(const ContactForceDistributionBase& contactForceDistribution1, const ContactForceDistributionBase& contactForceDistribution2, double t);

 protected:
  constexpr static int nLegs_ = 4; // TODO move to robotModel
  constexpr static int nTranslationalDofPerFoot_ = 3; // TODO move to robotModel
  constexpr static int nElementsVirtualForceTorqueVector_ = 6;

//  std::shared_ptr<TorsoBase> torso_;
//  std::shared_ptr<LegGroup> legs_;
//  std::shared_ptr<loco::TerrainModelBase> terrain_;
  std::shared_ptr<free_gait::State> robot_state_;

  ros::NodeHandle node_handle_;
  //! True if parameters are successfully loaded.
  bool isParametersLoaded_;

  //! True if data is logged.
  bool isLogging_;

  //! True if a force distribution was computed successfully.
  bool isForceDistributionComputed_;

  /*!
   * Check if parameters are loaded.
   * @return true if parameters are loaded.
   */
  virtual bool checkIfParametersLoaded() const;

  /*!
   * Check if force distribution is computed.
   * @return true if force distribution is computed.
   */
  virtual bool checkIfForceDistributionComputed() const;

  /*!
   * Update the data that is recorded by the logger.
   * @return true if successful, false if it is not logging.
   */
  virtual bool updateLoggerData() = 0;

};

} /* namespace balance_controller */
