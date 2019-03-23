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
* @file     ContactForceDistributionBase.cpp
* @author   Péter Fankhauser, Christian Gehring
* @date     Aug 6, 2013
* @brief
*/
#include "balance_controller/contact_force_distribution/ContactForceDistributionBase.hpp"

using namespace std;
using namespace Eigen;

namespace balance_controller {

ContactForceDistributionBase::ContactForceDistributionBase(const ros::NodeHandle& node_handle,
                                                           std::shared_ptr<free_gait::State> robot_state)
: robot_state_(robot_state),
  node_handle_(node_handle)
  /*torso_(torso),
  legs_(legs),
  terrain_(terrain)*/
{
  isParametersLoaded_ = false;
  isLogging_ = false;
  isForceDistributionComputed_ = false;
}

ContactForceDistributionBase::~ContactForceDistributionBase()
{

}

bool ContactForceDistributionBase::checkIfParametersLoaded() const
{
  if (isParametersLoaded_) return true;
  cout << "Contact force distribution parameters are not loaded." << endl; // TODO use warning output
  return false;
}

bool ContactForceDistributionBase::checkIfForceDistributionComputed() const
{
  if (isForceDistributionComputed_) return true;
//  cout << "Contact force distribution is not computed yet or was unsuccessful." << endl; // TODO use warning output
  return false;
}

bool ContactForceDistributionBase::setToInterpolated(const ContactForceDistributionBase& contactForceDistribution1, const ContactForceDistributionBase& contactForceDistribution2, double t) {
  return false;
}

} /* namespace loco */
