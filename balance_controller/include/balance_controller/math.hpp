/*******************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, Péter Fankhauser, C. Dario Bellicoso, Stelian Coros
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
 * math.hpp
 *
 *  Created on: Feb 23, 2014
 *      Author: gech
 */

#ifndef LOCO_MATH_HPP_
#define LOCO_MATH_HPP_

#include <cmath>
#include <Eigen/Core>
#include "free_gait_core/TypeDefs.hpp"
//#include "loco/common/TypeDefs.hpp"

namespace balance_controller {

inline void boundToRange(double* v, double min, double max){
  if (*v < min) *v = min;
  if (*v > max) *v = max;
}

inline double boundToRange(double v, double min, double max){
  if (v < min) return min;
  if (v > max) return max;
  return v;
}

/*
  if v < min, this method returns 0. If v > max, it returns 1. For everything else it returns some interpolated value;
*/
inline double mapTo01Range(double v, double min, double max){
  double t = v;
  if (fabs(min - max) < 0.000000001) return 1;
  boundToRange(&t, min, max);
  t = (t-min)/(max-min);
  return t;
}

inline double mapToUniformRange(double v, double min, double max){
  double t = v;
  if (fabs(min - max) < 0.000000001) return 1;
  t = (t-min)/(max-min);
  return t;
}

inline double linearlyInterpolate(double v1, double v2, double t1, double t2, double t){
  if (v1 == v2)
    return v2;
  return (t-t1)/(t2-t1) * v2 + (t2-t)/(t2-t1) * v1;
}

inline Eigen::Vector3d linearlyInterpolate(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, double t1, double t2, double t) {
  return (t-t1)/(t2-t1) * v2 + (t2-t)/(t2-t1) * v1;
}

inline Position linearlyInterpolate(const Position& v1, const Position& v2, double t1, double t2, double t) {
  return (t-t1)/(t2-t1) * v2 + (t2-t)/(t2-t1) * v1;
}

} // namespace loco

#endif /* LOCO_MATH_HPP_ */
