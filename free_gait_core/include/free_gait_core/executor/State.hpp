/*
 * State.hpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"
//#include <quadruped_model/QuadrupedState.hpp>
#include "quadruped_model/quadruped_state.h"
#include <quadruped_model/QuadrupedModel.hpp>

// STD
#include <vector>
#include <iostream>
#include <unordered_map>

#include "grid_map_core/Polygon.hpp"
#include "grid_map_core/grid_map_core.hpp"

namespace free_gait {
//! TODO(Shunyao): add state feedback from real or simulated robot,
//!                and build a new adapter inheritate from the base
//!                class AdapterBase, just like the test AdapterDummy
class State : public quadruped_model::QuadrupedState
{
 public:
  using QD = quadruped_model::QuadrupedModel::QuadrupedDescription;

  State();
  virtual ~State();

  virtual void initialize(const std::vector<LimbEnum>& limbs, const std::vector<BranchEnum>& branches);

  bool getRobotExecutionStatus() const;
  void setRobotExecutionStatus(bool robotExecutionStatus);

  const std::string& getStepId() const;
  void setStepId(const std::string& stepId);

  bool isSupportLeg(const LimbEnum& limb) const;
  void setSupportLeg(const LimbEnum& limb, bool isSupportLeg);
  /**
   * @brief getNumberOfSupportLegs
   * @return how many support legs?
   */
  unsigned int getNumberOfSupportLegs() const;

  bool isIgnoreContact(const LimbEnum& limb) const;
  void setIgnoreContact(const LimbEnum& limb, bool ignoreContact);

  bool hasSurfaceNormal(const LimbEnum& limb) const;
  const Vector& getSurfaceNormal(const LimbEnum& limb) const;
  void setSurfaceNormal(const LimbEnum& limb, const Vector& surfaceNormal);
  void removeSurfaceNormal(const LimbEnum& limb);

  bool isIgnoreForPoseAdaptation(const LimbEnum& limb) const;
  void setIgnoreForPoseAdaptation(const LimbEnum& limb, bool ignorePoseAdaptation);

  const JointPositionsLeg getJointPositionsForLimb(const LimbEnum& limb) const;
  void setJointPositionsForLimb(const LimbEnum& limb, const JointPositionsLeg& jointPositions);
  void setAllJointPositions(const JointPositions& jointPositions);

  const JointVelocitiesLeg getJointVelocitiesForLimb(const LimbEnum& limb) const;
  void setJointVelocitiesForLimb(const LimbEnum& limb, const JointVelocitiesLeg& jointVelocities);
  void setAllJointVelocities(const JointVelocities& jointVelocities);

  const JointAccelerationsLeg getJointAccelerationsForLimb(const LimbEnum& limb) const;
  const JointAccelerations& getAllJointAccelerations() const;
  void setJointAccelerationsForLimb(const LimbEnum& limb, const JointAccelerationsLeg& jointAccelerations);
  void setAllJointAccelerations(const JointAccelerations& jointAccelerations);

  const JointEffortsLeg getJointEffortsForLimb(const LimbEnum& limb) const;
  const JointEfforts& getAllJointEfforts() const;
  void setJointEffortsForLimb(const LimbEnum& limb, const JointEffortsLeg& jointEfforts);
  void setAllJointEfforts(const JointEfforts& jointEfforts);

  const ControlSetup& getControlSetup(const BranchEnum& branch) const;
  const ControlSetup& getControlSetup(const LimbEnum& limb) const;
  bool isControlSetupEmpty(const BranchEnum& branch) const;
  bool isControlSetupEmpty(const LimbEnum& limb) const;
  void setControlSetup(const BranchEnum& branch, const ControlSetup& controlSetup);
  void setControlSetup(const LimbEnum& limb, const ControlSetup& controlSetup);
  void setEmptyControlSetup(const BranchEnum& branch);
  void setEmptyControlSetup(const LimbEnum& limb);

  void getAllJointNames(std::vector<std::string>& jointNames) const;
  Position getSupportFootPosition(const LimbEnum& limb);
  void setSupportFootStance(const Stance& footInSupport);
  const Pose getFootholdsPlanePoseInWorld();
  friend std::ostream& operator << (std::ostream& out, const State& state);

 private:
  LocalAngularVelocity angularVelocityBaseInWorldFrame_;//dimension 3
  JointEfforts jointEfforts_;//dimension 12
  JointAccelerations jointAccelerations_;//dimension 12
  LinearAcceleration linearAccelerationBaseInWorldFrame_;//3D
  AngularAcceleration angularAccelerationBaseInBaseFrame_;//3D

  // Free gait specific.
  std::unordered_map<BranchEnum, ControlSetup, EnumClassHash> controlSetups_;//BranchEnum->control_level
  Force netForceOnBaseInBaseFrame_;//3D
  Torque netTorqueOnBaseInBaseFrame_;//3D
  std::unordered_map<LimbEnum, bool, EnumClassHash> isSupportLegs_;
  std::unordered_map<LimbEnum, bool, EnumClassHash> ignoreContact_;
  std::unordered_map<LimbEnum, bool, EnumClassHash> ignoreForPoseAdaptation_;//???Why need the ignore th e pose adaptation?
  std::unordered_map<LimbEnum, Vector, EnumClassHash> surfaceNormals_;//get the corresponding the leg footholds surfacenormals?
  bool robotExecutionStatus_;
  std::string stepId_; // empty if undefined.

  Stance footHoldInSupport_;//typedef std::unordered_map<LimbEnum, Position, EnumClassHash> Stance

  Pose footholds_plane_pose_;//position and rotation
};

} /* namespace */
