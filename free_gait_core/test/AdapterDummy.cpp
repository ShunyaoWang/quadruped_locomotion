/*
 * AdapterDummy.cpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "AdapterDummy.hpp"

namespace free_gait {

AdapterDummy::AdapterDummy()
    : AdapterBase(),
      worldFrameId_("odom"),
      baseFrameId_("base")
{
  state_.reset(new State());

  limbs_.push_back(LimbEnum::LF_LEG);
  limbs_.push_back(LimbEnum::RF_LEG);
  limbs_.push_back(LimbEnum::LH_LEG);
  limbs_.push_back(LimbEnum::RH_LEG);

  branches_.push_back(BranchEnum::BASE);
  branches_.push_back(BranchEnum::LF_LEG);
  branches_.push_back(BranchEnum::RF_LEG);
  branches_.push_back(BranchEnum::LH_LEG);
  branches_.push_back(BranchEnum::RH_LEG);
}

AdapterDummy::~AdapterDummy()
{
}

bool AdapterDummy::resetExtrasWithRobot(const StepQueue& stepQueue, State& state)
{
  return true;
}

bool AdapterDummy::updateExtrasBefore(const StepQueue& stepQueue, State& state)
{
  return true;
}

bool AdapterDummy::updateExtrasAfter(const StepQueue& stepQueue, State& state)
{
  //TODO(Shunyao): This is to update to Extras after Executor, real or sim Robot
  return true;
}

const std::string& AdapterDummy::getWorldFrameId() const
{
  return worldFrameId_;
}

const std::string& AdapterDummy::getBaseFrameId() const
{
  return baseFrameId_;
}

const std::vector<LimbEnum>& AdapterDummy::getLimbs() const
{
  return limbs_;
}

const std::vector<BranchEnum>& AdapterDummy::getBranches() const
{
  return branches_;
}

LimbEnum AdapterDummy::getLimbEnumFromLimbString(const std::string& limb) const
{
  throw std::runtime_error("AdapterDummy::getLimbEnumFromLimbString() is not implemented.");
}

std::string AdapterDummy::getLimbStringFromLimbEnum(const LimbEnum& limb) const
{
  throw std::runtime_error("AdapterDummy::getLimbStringFromLimbEnum() is not implemented.");
}

std::string AdapterDummy::getBaseString() const
{
  throw std::runtime_error("AdapterDummy::getBaseString() is not implemented.");
}

JointNodeEnum AdapterDummy::getJointNodeEnumFromJointNodeString(const std::string& jointNode) const
{
  throw std::runtime_error("AdapterDummy::getJointNodeEnumFromJointNodeString() is not implemented.");
}

std::string AdapterDummy::getJointNodeStringFromJointNodeEnum(const JointNodeEnum& jointNode) const
{
  throw std::runtime_error("AdapterDummy::getJointNodeStringFromJointNodeEnum() is not implemented.");
}

bool AdapterDummy::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
    const Position& positionBaseToFootInBaseFrame, const LimbEnum& limb,
    JointPositionsLeg& jointPositions) const
{
  // TODO(Shunyao): solve the single limb IK
  throw std::runtime_error("AdapterDummy::getLimbJointPositionsFromPositionBaseToFootInBaseFrame() is not implemented.");
}

Position AdapterDummy::getPositionBaseToFootInBaseFrame(
    const LimbEnum& limb, const JointPositionsLeg& jointPositions) const
{
    std::cout <<"what" <<std::endl;
  throw std::runtime_error("AdapterDummy::getPositionBaseToFootInBaseFrame() is not implemented.");
}

Position AdapterDummy::getPositionBaseToHipInBaseFrame(const LimbEnum& limb) const
{
  switch (limb) {
    case LimbEnum::LF_LEG:
      return Position(0.42, 0.075, 0.0);
    case LimbEnum::RF_LEG:
      return Position(0.42, -0.075, 0.0);
    case LimbEnum::LH_LEG:
      return Position(-0.42, 0.075, 0.0);
    case LimbEnum::RH_LEG:
      return Position(-0.42, -0.075, 0.0);
    default:
      throw std::runtime_error("AdapterDummy::getPositionBaseToHipInBaseFrame() something went wrong.");
  }
}

bool AdapterDummy::isExecutionOk() const
{
  throw std::runtime_error("AdapterDummy::isExecutionOk() is not implemented.");
}

bool AdapterDummy::isLegGrounded(const LimbEnum& limb) const
{
  throw std::runtime_error("AdapterDummy::isLegGrounded() is not implemented.");
}

JointPositionsLeg AdapterDummy::getJointPositionsForLimb(const LimbEnum& limb) const
{
  //! WSHY: update measurement
  JointPositions all_joints_position = state_->getJointPositionFeedback();

  switch (limb) {
    case LimbEnum::LF_LEG :
      return JointPositionsLeg(all_joints_position(0),all_joints_position(1),all_joints_position(2));
      break;
    case LimbEnum::RF_LEG :
      return JointPositionsLeg(all_joints_position(3),all_joints_position(4),all_joints_position(5));
      break;
    case LimbEnum::RH_LEG :
      return JointPositionsLeg(all_joints_position(6),all_joints_position(7),all_joints_position(8));
      break;
    case LimbEnum::LH_LEG :
      return JointPositionsLeg(all_joints_position(9),all_joints_position(10),all_joints_position(11));
      break;
    }
  state_->getJointPositionFeedback();
  return state_->getJointPositionsForLimb(limb);
}

JointPositions AdapterDummy::getAllJointPositions() const
{
  return state_->getJointPositions();
}

JointVelocitiesLeg AdapterDummy::getJointVelocitiesForLimb(const LimbEnum& limb) const
{
  return state_->getJointVelocitiesForLimb(limb);
}

JointVelocities AdapterDummy::getAllJointVelocities() const
{
  return state_->getJointVelocities();
}

JointAccelerationsLeg AdapterDummy::getJointAccelerationsForLimb(const LimbEnum& limb) const
{
  throw std::runtime_error("AdapterDummy::getJointAccelerationsForLimb() is not implemented.");
}

JointAccelerations AdapterDummy::getAllJointAccelerations() const
{
  throw std::runtime_error("AdapterDummy::getAllJointAccelerations() is not implemented.");
}

JointEffortsLeg AdapterDummy::getJointEffortsForLimb(const LimbEnum& limb) const
{
  return state_->getJointEffortsForLimb(limb);
}

JointEfforts AdapterDummy::getAllJointEfforts() const
{
  return state_->getAllJointEfforts();
}

Position AdapterDummy::getPositionWorldToBaseInWorldFrame() const
{
//  return Position(0,0,0);
  return state_->getPositionWorldToBaseInWorldFrame();
}

RotationQuaternion AdapterDummy::getOrientationBaseToWorld() const
{
//  return RotationQuaternion(1,0,0,0);
  return state_->getOrientationBaseToWorld();
}

LinearVelocity AdapterDummy::getLinearVelocityBaseInWorldFrame() const
{
  return state_->getLinearVelocityBaseInWorldFrame();
}

LocalAngularVelocity AdapterDummy::getAngularVelocityBaseInBaseFrame() const
{
  return state_->getAngularVelocityBaseInBaseFrame();
}

LinearAcceleration AdapterDummy::getLinearAccelerationBaseInWorldFrame() const
{
  throw std::runtime_error("AdapterDummy::getLinearAccelerationBaseInWorldFrame() is not implemented.");
}

AngularAcceleration AdapterDummy::getAngularAccelerationBaseInBaseFrame() const
{
  throw std::runtime_error("AdapterDummy::getAngularAccelerationBaseInBaseFrame() is not implemented.");
}

Position AdapterDummy::getPositionBaseToFootInBaseFrame(const LimbEnum& limb) const
{
  throw std::runtime_error("AdapterDummy::getPositionBaseToFootInBaseFrame() is not implemented.");
}

Position AdapterDummy::getPositionWorldToFootInWorldFrame(const LimbEnum& limb) const
{
  throw std::runtime_error("AdapterDummy::getPositionWorldToFootInWorldFrame() is not implemented.");
}

Position AdapterDummy::getCenterOfMassInWorldFrame() const
{
//  return Position(0,0,0);
  return state_->getPositionWorldToBaseInWorldFrame();
}

void AdapterDummy::getAvailableFrameTransforms(std::vector<std::string>& frameTransforms) const
{
  throw std::runtime_error("AdapterDummy::getAvailableFrameTransforms() is not implemented.");
}

Pose AdapterDummy::getFrameTransform(const std::string& frameId) const
{
  throw std::runtime_error("AdapterDummy::getFrameTransform() is not implemented.");
}

ControlSetup AdapterDummy::getControlSetup(const BranchEnum& branch) const
{
  throw std::runtime_error("AdapterDummy::getControlSetup() is not implemented.");
}

ControlSetup AdapterDummy::getControlSetup(const LimbEnum& limb) const
{
  throw std::runtime_error("AdapterDummy::getControlSetup() is not implemented.");
}

//! State depending on real robot.
JointVelocitiesLeg AdapterDummy::getJointVelocitiesFromEndEffectorLinearVelocityInWorldFrame(
    const LimbEnum& limb, const LinearVelocity& endEffectorLinearVelocityInWorldFrame) const
{
  //TODO(Shunyao):dp = jaccobian * dq
  throw std::runtime_error("AdapterDummy::getJointVelocitiesFromEndEffectorLinearVelocityInWorldFrame() is not implemented.");
}

LinearVelocity AdapterDummy::getEndEffectorLinearVelocityFromJointVelocities(const LimbEnum& limb,
                                                                       const JointVelocitiesLeg& jointVelocities,
                                                                       const std::string& frameId) const
{
  throw std::runtime_error("AdapterDummy::getEndEffectorLinearVelocityFromJointVelocities() is not implemented.");
}

JointAccelerationsLeg AdapterDummy::getJointAccelerationsFromEndEffectorLinearAccelerationInWorldFrame(
    const LimbEnum& limb, const LinearAcceleration& endEffectorLinearAccelerationInWorldFrame) const
{
  throw std::runtime_error("AdapterDummy::getJointAccelerationsFromEndEffectorLinearAccelerationInWorldFrame() is not implemented.");
}

bool AdapterDummy::setInternalDataFromState(const State& state, bool updateContacts, bool updatePosition,
                                                 bool updateVelocity, bool updateAcceleration) const
{
  *state_ = state;
  return true;
}

void AdapterDummy::createCopyOfState() const
{

}

void AdapterDummy::resetToCopyOfState() const
{

}

const State& AdapterDummy::getState() const
{
  return *state_;
}

} /* namespace */

