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
* @file     MotionControllerBase.hpp
* @author   Péter Fankhauser, Christian Gehring
* @date     March 6, 2014
* @brief
*/
#ifndef LOCO_MOTIONCONTROLLERBASE_H_
#define LOCO_MOTIONCONTROLLERBASE_H_

// Shared pointers
#include <memory>
// Locomotion controller common
//#include "loco/common/LegGroup.hpp"
//#include "loco/common/TorsoBase.hpp"
// Contact force distribution
#include "balance_controller/contact_force_distribution/ContactForceDistributionBase.hpp"
// Parameters
#include "tinyxml.h"

#include "ros/ros.h"

namespace balance_controller {

class MotionControllerBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Constructor.
   */
//  MotionControllerBase(std::shared_ptr<LegGroup> legs, std::shared_ptr<TorsoBase> torso);
  MotionControllerBase(const ros::NodeHandle& node_handle, std::shared_ptr<free_gait::State> robot_state);

  /*!
   * Destructor.
   */
  virtual ~MotionControllerBase();

  /*!
   * Load parameters.
   * @return true if successful
   */
  virtual bool loadParameters(const TiXmlHandle& handle) = 0;
  /*!
   * \brief loadParameters from ros parameters server
   * \return
   */
  virtual bool loadParameters() = 0;
  /*!
   * Add data to logger (optional).
   * @return true if successful
   */
  virtual bool addToLogger() = 0;

  /*!
   * Computes the joint torques from the desired base pose.
   * @return true if successful
   */
  virtual bool compute() = 0;

  /*! Sets the parameters to the interpolated ones between motionController1 and controller2.
   * @param motionController1     If the interpolation parameter is 0, then the parameter set is equal to the one of motionController1.
   * @param motionController2     If the interpolation parameter is 1, then the parameter set is equal to the one of motionController2.
   * @param t                     interpolation parameter in [0, 1]
   * @return                      true if successful
   */
  virtual bool setToInterpolated(const MotionControllerBase& motionController1, const MotionControllerBase& motionController2, double t);

 protected:
//  std::shared_ptr<LegGroup> legs_;
//  std::shared_ptr<TorsoBase> torso_;

  ros::NodeHandle node_handle_;

  std::shared_ptr<free_gait::State> robot_state_;

  //! True if parameters are successfully loaded.
  bool isParametersLoaded_;

  //! True if data is logged.
  bool isLogging_;

  /*!
   * Check if parameters are loaded.
   * @return true if parameters are loaded.
   */
  virtual bool checkIfParametersLoaded() const;
};

} /* namespace balance_controller */
#endif /* MOTIONCONTROLLERBASE_H_ */
