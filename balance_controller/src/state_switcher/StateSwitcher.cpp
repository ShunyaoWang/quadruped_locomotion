/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, C. Dario Bellicoso, Christian Gehring, Péter Fankhauser, Stelian Coros
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
/*
 * StateSwitcher.cpp
 *
 *  Created on: Oct 5, 2014
 *      Author: C. Dario Bellicoso
 */

#include "state_switcher/StateSwitcher.hpp"
#include <iostream>

const bool STATESWITCHER_DEBUG = false;

namespace balance_controller {

StateSwitcher::StateSwitcher(): StateSwitcherBase() {
  currentState_ = States::Init;
  oldState_ = States::Init;
}


StateSwitcher::~StateSwitcher() {

}


bool StateSwitcher::initialize(double dt) {
  currentState_ = States::Init;
  oldState_ = States::Init;

  return true;
}


StateSwitcher::States StateSwitcher::getState() {
  return currentState_;
}


void StateSwitcher::setState(States state) {
  if (STATESWITCHER_DEBUG) {
    std::cout << "Setting state from: " << getStateName(currentState_) << " to: " << getStateName(state) << std::endl;
  }
  oldState_ = currentState_;
  currentState_ = state;
}


std::string StateSwitcher::getStateName(StateSwitcher::States state) {
  switch(state) {
    case(States::Init):                     return std::string("Init");
    case(States::StanceNormal):             return std::string("StanceNormal");
    case(States::StanceSlipping):           return std::string("StanceSlipping");
    case(States::StanceLostContact):        return std::string("StanceLostContact");
    case(States::SwingNormal):              return std::string("SwingNormal");
    case(States::SwingLateLiftOff):         return std::string("SwingLateLiftOff");
    case(States::SwingEarlyTouchDown):      return std::string("SwingEarlyTouchDown");
    case(States::SwingBumpedIntoObstacle):  return std::string("SwingBumpedIntoObstacle");
    default:                                return std::string("unknown state");
  }
}

} /* namespace loco */

