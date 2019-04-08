/*
 * AdapterGazebo.cpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */
/********************************************
*  __    __    __     ______            /  *
*         /   / /   /                  /   *
*        / / / /    |               __/    *
*        /    /      ______          /     *
*       /    /                      /      *
*                           |      /       *
*                    ______/      /        *
*********************************************
*  filename.cpp
*  Descriotion:
*
*  Created on: Aug 31, 2017
*  Author: Shunyao Wang
*  Institute: Harbin Institute of Technology, Shenzhen
*************************************/
#include "free_gait_ros/AdapterGazebo.hpp"

namespace free_gait {

AdapterGazebo::AdapterGazebo()
    : AdapterBase(),
      worldFrameId_("odom"),
      baseFrameId_("base_link")
{
  state_.reset(new State());
  state_->initialize(getLimbs(), getBranches());
//  state_->LoadRobotDescriptionFromFile("/home/hitstar/catkin_ws/src/quadruped_locomotion-dev/quadruped_model/urdf/simpledog.urdf");
//  std::cout<<state_->LF_Chain.getNrOfJoints()<<std::endl;
  std::cout<<"Constructing AdapterGazebo"<<std::endl;
  limbs_.push_back(LimbEnum::LF_LEG);
  limbs_.push_back(LimbEnum::RF_LEG);
  limbs_.push_back(LimbEnum::RH_LEG);
  limbs_.push_back(LimbEnum::LH_LEG);

  branches_.push_back(BranchEnum::BASE);
  branches_.push_back(BranchEnum::LF_LEG);
  branches_.push_back(BranchEnum::RF_LEG);
  branches_.push_back(BranchEnum::RH_LEG);
  branches_.push_back(BranchEnum::LH_LEG);
}

AdapterGazebo::~AdapterGazebo()
{
}

bool AdapterGazebo::resetExtrasWithRobot(const StepQueue& stepQueue, State& state)
{
  return true;
}
/**
 * @brief AdapterGazebo::updateExtrasBefore, update state before excutor
 * @param stepQueue
 * @param state
 * @return
 */
bool AdapterGazebo::updateExtrasBefore(const StepQueue& stepQueue, State& state)
{
//  std::cout<<"Flagggggggggggggggggggggggggggggggggggggggggggggggggggggggg"<<std::endl;
//  state_->getPositionWorldToFootInWorldFrame(LimbEnum::LF_LEG);
//  std::cout<<state_->getJointPositionFeedback()<<std::endl;//getAll not get the current but get the joint position order

//  state.setCurrentLimbJoints(state_->getJointPositionFeedback());
//  state.setPoseBaseToWorld(Pose(state_->getPositionWorldToBaseInWorldFrame(),
//                                state_->getOrientationBaseToWorld()));

  return true;
}

bool AdapterGazebo::updateExtrasAfter(const StepQueue& stepQueue, State& state)
{
  //TODO(Shunyao): This is to update to Extras after Executor, real or sim Robot
//  *state_ = state;
  State state_new = state;
  *state_ = state_new;
//  JointPositionsLeg joint_limb;
//  Position I_r_IF, B_r_BF;
//  Position I_r_IB = state_->getTargetPositionWorldToBaseInWorldFrame();//feedback, the actual base pose
//  //TODO(shunyao) : need the target base pose
//  RotationQuaternion I_R_B = state_->getTargetOrientationBaseToWorld();
////  BaseMotionBase baseMotion = stepQueue.getCurrentStep().getBaseMotion();
////  BaseAuto baseAuto;
////  BaseTarget baseTarget;
////  BaseTrajectory baseTrajectory;
////  switch (baseMotion.getType()) {
////    case BaseMotionBase::Type::Auto:
////    {
////      baseAuto = (dynamic_cast<BaseAuto&>(baseMotion));
////      break;
////    }
////    case BaseMotionBase::Type::Target:
////    {
////      baseTarget = (dynamic_cast<BaseTarget&>(baseMotion));
////      break;
////    }
////    case BaseMotionBase::Type::Trajectory:
////    {
////      baseTrajectory = (dynamic_cast<BaseTrajectory&>(baseMotion));
////      break;
////    }
////    default:
////    {
////      baseAuto = (dynamic_cast<BaseAuto&>(baseMotion));
////      break;
////    }
////  }
//  if(stepQueue.empty()) return true;
//  double time = stepQueue.getCurrentStep().getTime();
////  std::cout<<"+++++++++++++TIME++++++++++++++++"<<std::endl
////          <<stepQueue.getCurrentStep().getTime()<<std::endl<<"++++++++++++TIME++++++++++++++++++"<<std::endl;
//  for(const auto& limb : getLimbs())
//  {
//    if(state_->isSupportLeg(limb)&&stepQueue.getCurrentStep().hasBaseMotion())
//    {
//      std::cout<<"==========================================================="<<std::endl
//              <<"base motion phase : "<<stepQueue.getCurrentStep().getBaseMotionPhase()<<std::endl;
//      if(stepQueue.getCurrentStep().getBaseMotionPhase()>=1.0) continue;
//      if(!(time>0.01))
//      {
//        footholdsInSupport_[limb] = state_->getPositionWorldToFootInWorldFrame(limb);
//        }
////      state.getPositionWorldToFootInWorldFrame(limb);
//    //      I_r_IF = state_->getPositionWorldToFootInWorldFrame(limb);
//          I_r_IF = footholdsInSupport_.at(limb);
//    //      baseMotion.getDuration();
//    //      std::cout<<"=========================="<<getLimbStringFromLimbEnum(limb)<<"============================"<<std::endl;
////          std::cout<<"+++++++++++++I_r_IF++++++++++++++++"<<std::endl
////                  <<I_r_IF<<std::endl<<"+++++++++++++I_r_IF+++++++++++++++++"<<std::endl;
//    //      I_r_IF = stepQueue.getCurrentStep().getLegMotion(limb).
//          B_r_BF = I_R_B.inverseRotate(I_r_IF - I_r_IB);
////          std::cout<<"+++++++++++++I_r_IB++++++++++++++++"<<std::endl
////                  <<I_r_IB<<std::endl<<"+++++++++++++I_r_IB+++++++++++++++++"<<std::endl;
////          std::cout<<"Solving for "<<getLimbStringFromLimbEnum(limb)<<std::endl;
////          std::cout<<"========================================================"<<std::endl;
//          state_->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(B_r_BF, limb, joint_limb);
//          //! WSHY: comment to use balance_controller to control base motion
////          state_->setJointPositionsForLimb(limb, joint_limb);
////          std::cout<<"joint position : "<<joint_limb<<std::endl;

//    } else {

//    }
//  }
//  stepQueue.getCurrentStep().getLegMotion(LimbEnum::LF_LEG
  return true;
}

const std::string& AdapterGazebo::getWorldFrameId() const
{
  return worldFrameId_;
}

const std::string& AdapterGazebo::getBaseFrameId() const
{
  return baseFrameId_;
}

const std::vector<LimbEnum>& AdapterGazebo::getLimbs() const
{
  return limbs_;
}

const std::vector<BranchEnum>& AdapterGazebo::getBranches() const
{
  return branches_;
}

LimbEnum AdapterGazebo::getLimbEnumFromLimbString(const std::string& limb) const
{
  if(limb == "LF_LEG")
    return static_cast<LimbEnum>(0);
  if(limb == "RF_LEG")
    return static_cast<LimbEnum>(1);
  if(limb == "RH_LEG")
    return static_cast<LimbEnum>(2);
  if(limb == "LH_LEG")
    return static_cast<LimbEnum>(3);

//  throw std::runtime_error("AdapterGazebo::getLimbEnumFromLimbString() is not implemented.");
}

std::string AdapterGazebo::getLimbStringFromLimbEnum(const LimbEnum& limb) const
{
  if(limb == LimbEnum::LF_LEG)
    return "LF_LEG";
  if(limb == LimbEnum::RF_LEG)
    return "RF_LEG";
  if(limb == LimbEnum::LH_LEG)
    return "LH_LEG";
  if(limb == LimbEnum::RH_LEG)
    return "RH_LEG";
//  throw std::runtime_error("AdapterGazebo::getLimbStringFromLimbEnum() is not implemented.");
}

std::string AdapterGazebo::getBaseString() const
{
  return "BASE";
  throw std::runtime_error("AdapterGazebo::getBaseString() is not implemented.");
}

JointNodeEnum AdapterGazebo::getJointNodeEnumFromJointNodeString(const std::string& jointNode) const
{
  throw std::runtime_error("AdapterGazebo::getJointNodeEnumFromJointNodeString() is not implemented.");
}

std::string AdapterGazebo::getJointNodeStringFromJointNodeEnum(const JointNodeEnum& jointNode) const
{
  throw std::runtime_error("AdapterGazebo::getJointNodeStringFromJointNodeEnum() is not implemented.");
}

bool AdapterGazebo::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
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
//  throw std::runtime_error("AdapterGazebo::getLimbJointPositionsFromPositionBaseToFootInBaseFrame() is not implemented.");
}

Position AdapterGazebo::getPositionBaseToFootInBaseFrame(
    const LimbEnum& limb, const JointPositionsLeg& jointPositions) const
{
  // TODO(Shunyao): solve the single limb Kinematics
//  state_->FowardKinematicsSolve()
  return state_->getPositionBaseToFootInBaseFrame(limb,jointPositions);
  throw std::runtime_error("AdapterGazebo::getPositionBaseToFootInBaseFrame() is not implemented.");
}

Position AdapterGazebo::getPositionBaseToHipInBaseFrame(const LimbEnum& limb) const
{
  switch (limb) {
    case LimbEnum::LF_LEG:
      return Position(0.3, 0.2, 0.0);
    case LimbEnum::RF_LEG:
      return Position(0.3, -0.2, 0.0);
    case LimbEnum::LH_LEG:
      return Position(-0.3, 0.2, 0.0);
    case LimbEnum::RH_LEG:
      return Position(-0.3, -0.2, 0.0);
    default:
      throw std::runtime_error("AdapterGazebo::getPositionBaseToHipInBaseFrame() something went wrong.");
  }
}

bool AdapterGazebo::isExecutionOk() const
{
  // TODO(Shunyao): How to confirm isOK? from State?
  return true;
  throw std::runtime_error("AdapterGazebo::isExecutionOk() is not implemented.");
}

bool AdapterGazebo::isLegGrounded(const LimbEnum& limb) const
{
  return state_->isSupportLeg(limb);
//  return true;
  throw std::runtime_error("AdapterGazebo::isLegGrounded() is not implemented.");
}

JointPositionsLeg AdapterGazebo::getJointPositionsForLimb(const LimbEnum& limb) const
{
  return state_->getJointPositionsForLimb(limb);
}
/**
 * @brief AdapterGazebo::getAllJointPositions, joint feedback
 * @return
 */
JointPositions AdapterGazebo::getAllJointPositions() const
{
//  return state_->getJointPositions();
  return state_->getJointPositionFeedback();
}

JointVelocitiesLeg AdapterGazebo::getJointVelocitiesForLimb(const LimbEnum& limb) const
{
  return state_->getJointVelocitiesForLimb(limb);
}

JointVelocities AdapterGazebo::getAllJointVelocities() const
{
  return state_->getJointVelocities();
}

JointAccelerationsLeg AdapterGazebo::getJointAccelerationsForLimb(const LimbEnum& limb) const
{
  throw std::runtime_error("AdapterGazebo::getJointAccelerationsForLimb() is not implemented.");
}

JointAccelerations AdapterGazebo::getAllJointAccelerations() const
{
  throw std::runtime_error("AdapterGazebo::getAllJointAccelerations() is not implemented.");
}

JointEffortsLeg AdapterGazebo::getJointEffortsForLimb(const LimbEnum& limb) const
{
  return state_->getJointEffortsForLimb(limb);
}

JointEfforts AdapterGazebo::getAllJointEfforts() const
{
  return state_->getAllJointEfforts();
}
/**
 * @brief AdapterGazebo::getPositionWorldToBaseInWorldFrame, Feedback
 * @return
 */
Position AdapterGazebo::getPositionWorldToBaseInWorldFrame() const
{
//  return Position(0,0,0);
/****************
* TODO(Shunyao) : look up tf
*  from /odom to base and update to state do it int the updateExtras function
****************/
  return state_->getPositionWorldToBaseInWorldFrame();
}

RotationQuaternion AdapterGazebo::getOrientationBaseToWorld() const
{
//  return RotationQuaternion(1,0,0,0);
  return state_->getOrientationBaseToWorld();
}

LinearVelocity AdapterGazebo::getLinearVelocityBaseInWorldFrame() const
{
  return state_->getLinearVelocityBaseInWorldFrame();
}

LocalAngularVelocity AdapterGazebo::getAngularVelocityBaseInBaseFrame() const
{
  return state_->getAngularVelocityBaseInBaseFrame();
}

LinearAcceleration AdapterGazebo::getLinearAccelerationBaseInWorldFrame() const
{
  throw std::runtime_error("AdapterGazebo::getLinearAccelerationBaseInWorldFrame() is not implemented.");
}

AngularAcceleration AdapterGazebo::getAngularAccelerationBaseInBaseFrame() const
{
  throw std::runtime_error("AdapterGazebo::getAngularAccelerationBaseInBaseFrame() is not implemented.");
}

Position AdapterGazebo::getPositionBaseToFootInBaseFrame(const LimbEnum& limb) const
{
  return state_->getPositionBaseToFootInBaseFrame(limb);
  throw std::runtime_error("AdapterGazebo::getPositionBaseToFootInBaseFrame() is not implemented.");
}

Position AdapterGazebo::getPositionWorldToFootInWorldFrame(const LimbEnum& limb) const
{
  return state_->getPositionWorldToFootInWorldFrame(limb);
  throw std::runtime_error("AdapterGazebo::getPositionWorldToFootInWorldFrame() is not implemented.");
}

Position AdapterGazebo::getCenterOfMassInWorldFrame() const
{
//  return Position(0,0,0);
  return state_->getPositionWorldToBaseInWorldFrame();
}

void AdapterGazebo::getAvailableFrameTransforms(std::vector<std::string>& frameTransforms) const
{
  frameTransforms.push_back("/odom");
//  throw std::runtime_error("AdapterGazebo::getAvailableFrameTransforms() is not implemented.");
}

Pose AdapterGazebo::getFrameTransform(const std::string& frameId) const
{
  //TODO(Shunyao): It's seems to feedback Odom To Map, amcl?
  return Pose(Position(0,0,0),RotationQuaternion());//fixed odom with map temparently
  throw std::runtime_error("AdapterGazebo::getFrameTransform() is not implemented.");
}

ControlSetup AdapterGazebo::getControlSetup(const BranchEnum& branch) const
{
//  TODO(Shunyao): How to set up Control Level, In State class to set controlSetup
  return ControlSetup({{ControlLevel::Position, true},
                       {ControlLevel::Velocity, false},
                       {ControlLevel::Acceleration, false},
                       {ControlLevel::Effort, false}});
  throw std::runtime_error("AdapterGazebo::getControlSetup() is not implemented.");
}

ControlSetup AdapterGazebo::getControlSetup(const LimbEnum& limb) const
{
  return ControlSetup({{ControlLevel::Position, true},
                       {ControlLevel::Velocity, false},
                       {ControlLevel::Acceleration, false},
                       {ControlLevel::Effort, false}});
  throw std::runtime_error("AdapterGazebo::getControlSetup() is not implemented.");
}

//! State depending on real robot.
JointVelocitiesLeg AdapterGazebo::getJointVelocitiesFromEndEffectorLinearVelocityInWorldFrame(
    const LimbEnum& limb, const LinearVelocity& endEffectorLinearVelocityInWorldFrame) const
{
  throw std::runtime_error("AdapterGazebo::getJointVelocitiesFromEndEffectorLinearVelocityInWorldFrame() is not implemented.");
}

LinearVelocity AdapterGazebo::getEndEffectorLinearVelocityFromJointVelocities(const LimbEnum& limb,
                                                                       const JointVelocitiesLeg& jointVelocities,
                                                                       const std::string& frameId) const
{
  //TODO(Shunyao): dp = Jacobian * dq
//  if(frameId != getBaseFrameId())
//  {
//    transformLinearVelocity(frameId, getBaseFrameId(),)
//  }
  return state_->getEndEffectorLinearVelocityFromJointVelocities(limb, jointVelocities);
  throw std::runtime_error("AdapterGazebo::getEndEffectorLinearVelocityFromJointVelocities() is not implemented.");
}

JointAccelerationsLeg AdapterGazebo::getJointAccelerationsFromEndEffectorLinearAccelerationInWorldFrame(
    const LimbEnum& limb, const LinearAcceleration& endEffectorLinearAccelerationInWorldFrame) const
{
  throw std::runtime_error("AdapterGazebo::getJointAccelerationsFromEndEffectorLinearAccelerationInWorldFrame() is not implemented.");
}

bool AdapterGazebo::setInternalDataFromState(const State& state, bool updateContacts, bool updatePosition,
                                                 bool updateVelocity, bool updateAcceleration) const
{
  *state_ = state;
  return true;
}

void AdapterGazebo::createCopyOfState() const
{

}

void AdapterGazebo::resetToCopyOfState() const
{

}

const State& AdapterGazebo::getState() const
{
  return *state_;
}

} /* namespace */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(free_gait::AdapterGazebo, free_gait::AdapterBase)
