/*
 * PoseConstraintsChecker.cpp
 *
 *  Created on: Aug 31, 2017
 *      Author: Péter Fankhauser
 */

#include "free_gait_core/pose_optimization/PoseConstraintsChecker.hpp"

namespace free_gait {

PoseConstraintsChecker::PoseConstraintsChecker(const AdapterBase& adapter)
    : PoseOptimizationBase(adapter),
      centerOfMassTolerance_(0.0),
      legLengthTolerance_(0.0)
{
}

PoseConstraintsChecker::~PoseConstraintsChecker()
{
}

void PoseConstraintsChecker::setTolerances(const double centerOfMassTolerance, const double legLengthTolerance)
{
  centerOfMassTolerance_ = centerOfMassTolerance;
  legLengthTolerance_ = legLengthTolerance;
}

bool PoseConstraintsChecker::check(const Pose& pose)
{
  // TODO(Shunyao): findout the useness of those lines
//! state_ is a protected member in poseOptimizationBase, set it to AdapterBase,
//! implemented in AdapterDummy
    state_.setPoseBaseToWorld(pose); // this member function is in the quadrupedModel class, which is a parent class of State
//  adapter_.setInternalDataFromState(state_, false, true, false, false); // To guide IK.
    adapter_.setInternalDataFromState(state_); // To guide IK.
//  if (!updateJointPositionsInState(state_)) {
//    return false;
//  }
//  adapter_.setInternalDataFromState(state_, false, true, false, false);

  // Check center of mass.
  grid_map::Polygon supportRegionCopy(supportRegion_);
  supportRegionCopy.offsetInward(centerOfMassTolerance_);
  if (!supportRegion_.isInside(adapter_.getCenterOfMassInWorldFrame().vector().head(2))) {
    std::cout<<"Center of mass: "<<adapter_.getCenterOfMassInWorldFrame()<<" is out of support region"<<std::endl;
    return false;
  }

  // Check leg length. TODO Replace with joint limits?
  for (const auto& foot : stance_) {
    const Position footPositionInBase(
        adapter_.transformPosition(adapter_.getWorldFrameId(), adapter_.getBaseFrameId(), foot.second));
    const double legLength = Vector(footPositionInBase - adapter_.getPositionBaseToHipInBaseFrame(foot.first)).norm();
    if (legLength < minLimbLenghts_[foot.first] - legLengthTolerance_ || legLength > maxLimbLenghts_[foot.first] + legLengthTolerance_) {
      std::cout<<"Leg lenth is out of limits"<<std::endl;
      return false;
    }
  }

  return true;
}

}
