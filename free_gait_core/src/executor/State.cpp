/*
 * State.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <free_gait_core/executor/State.hpp>
#include <free_gait_core/TypePrints.hpp>

#include <stdexcept>//stl error lib

namespace free_gait {

State::State()
    : QuadrupedState(),
      robotExecutionStatus_(false)
{
}

State::~State()
{
}


void State::initialize(const std::vector<LimbEnum>& limbs, const std::vector<BranchEnum>& branches)
{
  for (const auto& limb : limbs) {
    isSupportLegs_[limb] = false;
    ignoreContact_[limb] = false;
    ignoreForPoseAdaptation_[limb] = false;
  }

  for (const auto& branch : branches) {
    setEmptyControlSetup(branch);
  }
  QuadrupedState::Initialize();
}

bool State::getRobotExecutionStatus() const
{
  return robotExecutionStatus_;
}

void State::setRobotExecutionStatus(bool robotExecutionStatus)
{
  robotExecutionStatus_ = robotExecutionStatus;
}

const std::string& State::getStepId() const
{
  return stepId_;
}

void State::setStepId(const std::string& stepId)
{
  stepId_ = stepId;
}

bool State::isSupportLeg(const LimbEnum& limb) const
{
  return isSupportLegs_.at(limb);
}

void State::setSupportLeg(const LimbEnum& limb, bool isSupportLeg)
{
  isSupportLegs_[limb] = isSupportLeg;
}

unsigned int State::getNumberOfSupportLegs() const
{
  unsigned int nLegs = 0;
  for (const auto& supportLeg : isSupportLegs_) {
    if (supportLeg.second) ++nLegs;
  }
  return nLegs;
}

bool State::isIgnoreContact(const LimbEnum& limb) const
{
  return ignoreContact_.at(limb);
}

void State::setIgnoreContact(const LimbEnum& limb, bool ignoreContact)
{
  ignoreContact_[limb] = ignoreContact;
}

bool State::hasSurfaceNormal(const LimbEnum& limb) const
{
  return (surfaceNormals_.count(limb) > 0u);
}

const Vector& State::getSurfaceNormal(const LimbEnum& limb) const
{
  return surfaceNormals_.at(limb);
}

void State::setSurfaceNormal(const LimbEnum& limb, const Vector& surfaceNormal)
{
  surfaceNormals_[limb] = surfaceNormal;
}

void State::removeSurfaceNormal(const LimbEnum& limb)
{
  surfaceNormals_.erase(limb);
}

bool State::isIgnoreForPoseAdaptation(const LimbEnum& limb) const
{
  return ignoreForPoseAdaptation_.at(limb);
}

void State::setIgnoreForPoseAdaptation(const LimbEnum& limb, bool ignorePoseAdaptation)
{
  ignoreForPoseAdaptation_[limb] = ignorePoseAdaptation;
}
/**
 * @brief State::getJointPositionsForLimb, get the target joint positions
 * @param limb
 * @return
 */
const JointPositionsLeg State::getJointPositionsForLimb(const LimbEnum& limb) const
{
  //TODO(Shunyao): fix state feedback
  int start, n;
  start = QD::getLimbStartIndexInJ(limb);
  n = QD::getNumDofLimb();
  return JointPositionsLeg(
      quadruped_model::QuadrupedState::getJointPositions().vector().segment(start, n)
  );
//  return JointPositionsLeg(
//      quadruped_model::QuadrupedState::getJointPositions().vector().segment<QD::getNumDofLimb()>(
//          QD::getLimbStartIndexInJ(limb))
//  );
}
/**
 * @brief State::setJointPositionsForLimb, set the Target Joint positions
 * @param limb
 * @param jointPositions
 */
void State::setJointPositionsForLimb(const LimbEnum& limb, const JointPositionsLeg& jointPositions)
{
//TODO(Shunyao): fix state feedback, why can not use n replaced 3?
//  int start, n;
//  start = QD::getLimbStartIndexInJ(limb);
//  n = QD::getNumDofLimb();
  // TODO(Shunyao) : something wrong with setSegment?
//  JointPositions J;
//  J.setZero();

//  J = getJointPositions();
//  J.setSegment<3>(QD::getLimbStartIndexInJ(limb), jointPositions);
  quadruped_model::QuadrupedState::getJointPositions().setSegment<3>(
      QD::getLimbStartIndexInJ(limb), jointPositions);
//  std::cout<<"***********************************"<<std::endl
//     <<quadruped_model::QuadrupedState::getJointPositions()<<std::endl
//    <<"**********************************"<<std::endl;
}

void State::setAllJointPositions(const JointPositions& jointPositions)
{
  setCurrentLimbJoints(jointPositions);
//  quadruped_model::QuadrupedState::setJointPositions(quadruped_model::JointPositions(jointPositions.vector()));
}

const JointVelocitiesLeg State::getJointVelocitiesForLimb(const LimbEnum& limb) const
{
  int start, n;
  start = QD::getLimbStartIndexInJ(limb);
  n = QD::getNumDofLimb();
  return JointVelocitiesLeg(
      quadruped_model::QuadrupedState::getJointVelocities().vector().segment(start, n)
  );
//  return JointVelocitiesLeg(
//      quadruped_model::QuadrupedState::getJointVelocities().vector().segment<QD::getNumDofLimb()>(
//      QD::getLimbStartIndexInJ(limb)
//  ));
}

void State::setJointVelocitiesForLimb(const LimbEnum& limb, const JointVelocitiesLeg& jointVelocities)
{
  quadruped_model::QuadrupedState::getJointVelocities().setSegment<3>(
      QD::getLimbStartIndexInJ(limb), jointVelocities);
}

void State::setAllJointVelocities(const JointVelocities& jointVelocities)
{
  setCurrentLimbJointVelocities(jointVelocities);
//  quadruped_model::QuadrupedState::setJointVelocities(quadruped_model::JointVelocities(jointVelocities.vector()));
}

const JointAccelerationsLeg State::getJointAccelerationsForLimb(const LimbEnum& limb) const
{
  int start, n;
  start = QD::getLimbStartIndexInJ(limb);
  n = QD::getNumDofLimb();
  return JointAccelerationsLeg(jointAccelerations_.vector().segment(start, n));

  //  return JointAccelerationsLeg(jointAccelerations_.vector().segment<QD::getNumDofLimb()>(QD::getLimbStartIndexInJ(limb)));
}

const JointAccelerations& State::getAllJointAccelerations() const
{
  return jointAccelerations_;
}

void State::setJointAccelerationsForLimb(const LimbEnum& limb, const JointAccelerationsLeg& jointAccelerations)
{
  jointAccelerations_.setSegment<3>(QD::getLimbStartIndexInJ(limb), jointAccelerations);
//  jointAccelerations_.setSegment<QD::getNumDofLimb()>(QD::getLimbStartIndexInJ(limb), jointAccelerations);
}

void State::setAllJointAccelerations(const JointAccelerations& jointAccelerations)
{
  jointAccelerations_ = jointAccelerations;
}

const JointEffortsLeg State::getJointEffortsForLimb(const LimbEnum& limb) const
{
  int start, n;
  start = QD::getLimbStartIndexInJ(limb);
  n = QD::getNumDofLimb();
  return JointEffortsLeg(getAllJointEfforts().vector().segment(start, n));
//  return JointEffortsLeg(getAllJointEfforts().vector().segment<QD::getNumDofLimb()>(QD::getLimbStartIndexInJ(limb)));
}

const JointEfforts& State::getAllJointEfforts() const
{
  return jointEfforts_;
}

void State::setJointEffortsForLimb(const LimbEnum& limb, const JointEffortsLeg& jointEfforts)
{
  jointEfforts_.setSegment<3>(QD::getLimbStartIndexInJ(limb), jointEfforts);
//  jointEfforts_.getSegment<QD::getNumDofLimb()>(QD::getLimbStartIndexInJ(limb)) = jointEfforts;
}

void State::setAllJointEfforts(const JointEfforts& jointEfforts)
{
  jointEfforts_ = jointEfforts;
}

const ControlSetup& State::getControlSetup(const BranchEnum& branch) const
{
  return controlSetups_.at(branch);
}

const ControlSetup& State::getControlSetup(const LimbEnum& limb) const
{
  return controlSetups_.at(QD::mapEnums(limb));
//  return controlSetups_.at(QD::mapEnums<QD::BranchEnum>(limb));
}

bool State::isControlSetupEmpty(const BranchEnum& branch) const
{
  for (const auto& level : controlSetups_.at(branch)) {
    if (level.second) return false;
  }
  return true;
}

bool State::isControlSetupEmpty(const LimbEnum& limb) const
{
  return isControlSetupEmpty(QD::mapEnums(limb));
//  return isControlSetupEmpty(QD::mapEnums<QD::BranchEnum>(limb));
}

void State::setControlSetup(const BranchEnum& branch, const ControlSetup& controlSetup)
{
  controlSetups_[branch] = controlSetup;
}

void State::setControlSetup(const LimbEnum& limb, const ControlSetup& controlSetup)
{
  controlSetups_[QD::mapEnums(limb)] =controlSetup;
//  controlSetups_[QD::mapEnums<QD::BranchEnum>(limb)] = controlSetup;
}

void State::setEmptyControlSetup(const BranchEnum& branch)
{
  ControlSetup emptyControlSetup;
  emptyControlSetup[ControlLevel::Position] = false;
  emptyControlSetup[ControlLevel::Velocity] = false;
  emptyControlSetup[ControlLevel::Acceleration] = false;
  emptyControlSetup[ControlLevel::Effort] = false;
  controlSetups_[branch] = emptyControlSetup;
}

void State::setEmptyControlSetup(const LimbEnum& limb)
{
  setEmptyControlSetup(QD::mapEnums(limb));
//  setEmptyControlSetup(QD::mapEnums<QD::BranchEnum>(limb));
}

void State::getAllJointNames(std::vector<std::string>& jointNames) const
{
  jointNames.clear();
  std::vector<std::string> controller_joint_names{"front_left_1_joint", "front_left_2_joint", "front_left_3_joint" ,
                                                  "front_right_1_joint", "front_right_2_joint", "front_right_3_joint",
                                                  "rear_right_1_joint", "rear_right_2_joint", "rear_right_3_joint",
                                                  "rear_left_1_joint", "rear_left_2_joint", "rear_left_3_joint"};
  jointNames.reserve(12);
  jointNames = controller_joint_names;
//  jointNames.reserve(QD::getJointsDimension());
//  for (const auto& jointKey : QD::getJointKeys()) {
//    jointNames.push_back(QD::mapKeyEnumToKeyName(jointKey.getEnum()));
//  }
}

Position State::getSupportFootPosition(const LimbEnum& limb)
{
  return footHoldInSupport_.at(limb);
}
void State::setSupportFootStance(const Stance& footInSupport)
{
  footHoldInSupport_ =footInSupport;
}

const Pose State::getFootholdsPlanePoseInWorld()
{
//  grid_map::Polygon supportRegion;
  std::vector<Position> footholdsOrdered;
  getFootholdsCounterClockwiseOrdered(footHoldInSupport_, footholdsOrdered);
//  for (auto foothold : footholdsOrdered) {
//    supportRegion.addVertex(foothold.vector());
////    std::cout<<"footholds ordered is "<<foothold.vector().head<2>()<<std::endl;
//  }


}

std::ostream& operator<<(std::ostream& out, const State& state)
{
//  out << static_cast<const quadruped_model::QuadrupedState&>(state) << std::endl;
  out << "Support legs: " << state.isSupportLegs_ << std::endl;
  out << "Ignore contact: " << state.ignoreContact_ << std::endl;
  out << "Ignore for pose adaptation: " << state.ignoreForPoseAdaptation_ << std::endl;
  out << "Control setup:" << std::endl;
  for (const auto& controlSetup : state.controlSetups_) {
    out << controlSetup.first << ": ";
    for (const auto& controlLevel : controlSetup.second) {
      if (controlLevel.second) out << controlLevel.first << ", ";
    }
    out << std::endl;
  }
  out << "Surface normals: " << state.surfaceNormals_ << std::endl;
  if (!state.stepId_.empty()) out << "Step ID: " << state.stepId_ << std::endl;
  return out;
}

} /* namespace free_gait */
