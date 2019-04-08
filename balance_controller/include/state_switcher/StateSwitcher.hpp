/*******************************************************************************************
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
 * StateSwitcher.hpp
 *
 *  Created on: Oct 5, 2014
 *      Author: C. Dario Bellicoso
 */

#ifndef LOCO_STATESWITCHER_HPP_
#define LOCO_STATESWITCHER_HPP_

#include "state_switcher/StateSwitcherBase.hpp"
#include <string>

namespace balance_controller {

class StateSwitcher: public StateSwitcherBase {
 public:

  /* state_
   * 0: stance phase normal condition
   * 1: swing phase normal condition
   * 2: stance, but slipping
   * 3: stance, but lost contact / not yet touch-down
   * 4: swing, but late lift-off
   * 5: late swing, but early touch-down
   * 6: middle swing, but bumped into obstacle while swinging
   */
  enum States {
    Init,
    StanceNormal,
    StanceSlipping,
    StanceLostContact,
    SwingNormal,
    SwingLateLiftOff,
    SwingEarlyTouchDown,
    SwingBumpedIntoObstacle,
    SwingLatelyTouchDown
  };

  StateSwitcher();
  virtual ~StateSwitcher();

  virtual bool initialize(double dt);

  virtual void setState(States state);
  virtual States getState();

 protected:
  States currentState_;
  States oldState_;

  virtual std::string getStateName(States state);

};

} /* namespace loco */

#endif /* LOCO_STATESWITCHER_HPP_ */
