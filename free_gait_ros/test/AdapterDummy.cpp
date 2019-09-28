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
      baseFrameId_("base_link")
{
  state_.reset(new State());
  state_->initialize(getLimbs(), getBranches());
  std::cout<<"Constructing AdapterDummy"<<std::endl;
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
  state.setCurrentLimbJoints(getAllJointPositions());
  return true;
}

bool AdapterDummy::updateExtrasAfter(const StepQueue& stepQueue, State& state)
{
  //TODO(Shunyao): This is to update to Extras after Executor, real or sim Robot
  *state_ = state;
  JointPositionsLeg joint_limb;
  Position I_r_IF, B_r_BF;
  Position I_r_IB = state_->getPositionWorldToBaseInWorldFrame();
  RotationQuaternion I_R_B = state_->getOrientationBaseToWorld();
//  BaseMotionBase baseMotion = stepQueue.getCurrentStep().getBaseMotion();
//  BaseAuto baseAuto;
//  BaseTarget baseTarget;
//  BaseTrajectory baseTrajectory;
//  switch (baseMotion.getType()) {
//    case BaseMotionBase::Type::Auto:
//    {
//      baseAuto = (dynamic_cast<BaseAuto&>(baseMotion));
//      break;
//    }
//    case BaseMotionBase::Type::Target:
//    {
//      baseTarget = (dynamic_cast<BaseTarget&>(baseMotion));
//      break;
//    }
//    case BaseMotionBase::Type::Trajectory:
//    {
//      baseTrajectory = (dynamic_cast<BaseTrajectory&>(baseMotion));
//      break;
//    }
//    default:
//    {
//      baseAuto = (dynamic_cast<BaseAuto&>(baseMotion));
//      break;
//    }
//  }
  if(stepQueue.empty()) return true;
  double time = stepQueue.getCurrentStep().getTime();
  std::cout<<"+++++++++++++TIME++++++++++++++++"<<std::endl
          <<stepQueue.getCurrentStep().getTime()<<std::endl<<"++++++++++++TIME++++++++++++++++++"<<std::endl;
  for(const auto& limb : getLimbs())
  {
    if(state_->isSupportLeg(limb))
    {
      if(!(time>0.01))
      {
        footholdsInSupport_[limb] = state_->getPositionWorldToFootInWorldFrame(limb);
      }
//      I_r_IF = state_->getPositionWorldToFootInWorldFrame(limb);
      I_r_IF = footholdsInSupport_.at(limb);
//      baseMotion.getDuration();
//      std::cout<<"=========================="<<getLimbStringFromLimbEnum(limb)<<"============================"<<std::endl;
//      std::cout<<"+++++++++++++I_r_IF++++++++++++++++"<<std::endl
//              <<I_r_IF<<std::endl<<"+++++++++++++I_r_IF+++++++++++++++++"<<std::endl;
//      I_r_IF = stepQueue.getCurrentStep().getLegMotion(limb).
      B_r_BF = I_R_B.inverseRotate(I_r_IF - I_r_IB);
//      std::cout<<"+++++++++++++B_r_BF++++++++++++++++"<<std::endl
//              <<B_r_BF<<std::endl<<"+++++++++++++B_r_BF+++++++++++++++++"<<std::endl;
      state_->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(B_r_BF, limb, joint_limb);
      state_->setJointPositionsForLimb(limb, joint_limb);
    } else {

    }
  }
//  stepQueue.getCurrentStep().getLegMotion(LimbEnum::LF_LEG
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
  if(limb == "LF_LEG")
    return static_cast<LimbEnum>(0);
  if(limb == "RF_LEG")
    return static_cast<LimbEnum>(1);
  if(limb == "RH_LEG")
    return static_cast<LimbEnum>(2);
  if(limb == "LH_LEG")
    return static_cast<LimbEnum>(3);

//  throw std::runtime_error("AdapterDummy::getLimbEnumFromLimbString() is not implemented.");
}

std::string AdapterDummy::getLimbStringFromLimbEnum(const LimbEnum& limb) const
{
  if(limb == LimbEnum::LF_LEG)
    return "LF_LEG";
  if(limb == LimbEnum::RF_LEG)
    return "RF_LEG";
  if(limb == LimbEnum::LH_LEG)
    return "LH_LEG";
  if(limb == LimbEnum::RH_LEG)
    return "RH_LEG";
//  throw std::runtime_error("AdapterDummy::getLimbStringFromLimbEnum() is not implemented.");
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
//  QuadrupedKinematics::FowardKinematicsSolve()
//  state_->FowardKinematicsSolve()
//  std::cout<<"start"<<std::endl;
  if(state_->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(positionBaseToFootInBaseFrame,
                                                            limb,jointPositions))
  {
    return true;
  }
  throw std::runtime_error("AdapterDummy::getLimbJointPositionsFromPositionBaseToFootInBaseFrame() is not implemented.");
}

Position AdapterDummy::getPositionBaseToFootInBaseFrame(
    const LimbEnum& limb, const JointPositionsLeg& jointPositions) const
{
  // TODO(Shunyao): solve the single limb Kinematics
//  state_->FowardKinematicsSolve()
  std::cout << "what? " << std::endl;
  return state_->getPositionBaseToFootInBaseFrame(limb,jointPositions);
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
  // TODO(Shunyao): How to confirm isOK? from State?
  return true;
  throw std::runtime_error("AdapterDummy::isExecutionOk() is not implemented.");
}

bool AdapterDummy::isLegGrounded(const LimbEnum& limb) const
{
//  return state_->isSupportLeg(limb);
  return true;
  throw std::runtime_error("AdapterDummy::isLegGrounded() is not implemented.");
}

JointPositionsLeg AdapterDummy::getJointPositionsForLimb(const LimbEnum& limb) const
{
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
  return state_->getPositionBaseToFootInBaseFrame(limb);
  throw std::runtime_error("AdapterDummy::getPositionBaseToFootInBaseFrame() is not implemented.");
}

Position AdapterDummy::getPositionWorldToFootInWorldFrame(const LimbEnum& limb) const
{
  return state_->getPositionWorldToFootInWorldFrame(limb);
  throw std::runtime_error("AdapterDummy::getPositionWorldToFootInWorldFrame() is not implemented.");
}

Position AdapterDummy::getCenterOfMassInWorldFrame() const
{
//  return Position(0,0,0);
  return state_->getPositionWorldToBaseInWorldFrame();
}

void AdapterDummy::getAvailableFrameTransforms(std::vector<std::string>& frameTransforms) const
{
  frameTransforms.push_back("/odom");
//  throw std::runtime_error("AdapterDummy::getAvailableFrameTransforms() is not implemented.");
}

Pose AdapterDummy::getFrameTransform(const std::string& frameId) const
{
  //TODO(Shunyao): It's seems to feedback Odom To Map, amcl?
  return Pose(Position(0,0,0),RotationQuaternion());//fixed odom with map temparently
  throw std::runtime_error("AdapterDummy::getFrameTransform() is not implemented.");
}

ControlSetup AdapterDummy::getControlSetup(const BranchEnum& branch) const
{
//  TODO(Shunyao): How to set up Control Level, In State class to set controlSetup
  return ControlSetup({{ControlLevel::Position, true},
                       {ControlLevel::Velocity, false},
                       {ControlLevel::Acceleration, false},
                       {ControlLevel::Effort, false}});
  throw std::runtime_error("AdapterDummy::getControlSetup() is not implemented.");
}

ControlSetup AdapterDummy::getControlSetup(const LimbEnum& limb) const
{  
  return ControlSetup({{ControlLevel::Position, true},
                       {ControlLevel::Velocity, false},
                       {ControlLevel::Acceleration, false},
                       {ControlLevel::Effort, false}});
  throw std::runtime_error("AdapterDummy::getControlSetup() is not implemented.");
}

//! State depending on real robot.
JointVelocitiesLeg AdapterDummy::getJointVelocitiesFromEndEffectorLinearVelocityInWorldFrame(
    const LimbEnum& limb, const LinearVelocity& endEffectorLinearVelocityInWorldFrame) const
{
  throw std::runtime_error("AdapterDummy::getJointVelocitiesFromEndEffectorLinearVelocityInWorldFrame() is not implemented.");
}

LinearVelocity AdapterDummy::getEndEffectorLinearVelocityFromJointVelocities(const LimbEnum& limb,
                                                                       const JointVelocitiesLeg& jointVelocities,
                                                                       const std::string& frameId) const
{
  //TODO(Shunyao): dp = Jacobian * dq
//  if(frameId != getBaseFrameId())
//  {
//    transformLinearVelocity(frameId, getBaseFrameId(),)
//  }
  return state_->getEndEffectorLinearVelocityFromJointVelocities(limb, jointVelocities);
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

