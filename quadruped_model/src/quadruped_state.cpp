#include "quadruped_model/quadruped_state.h"
namespace quadruped_model {
//JointPositions QuadrupedState::joint_positions_ = JointPositions(Eigen::VectorXd::Zero(12,1));
JointPositions QuadrupedState::joint_positions_ = JointPositions().setZero();
JointPositions QuadrupedState::joint_positions_feedback_ = JointPositions(Eigen::VectorXd::Zero(12,1));
JointVelocities QuadrupedState::joint_velocities_ = JointVelocities(Eigen::VectorXd::Zero(12,1));

QuadrupedState::QuadrupedState()
  : QuadrupedKinematics()
{
//  QuadrupedKinematics::LoadRobotDescriptionFromFile("/home/hitstar/catkin_ws/src/quadruped_locomotion-dev/quadruped_model/urdf/simpledog.urdf");
//  QK = new QuadrupedKinematics()
//  QK.LoadRobotDescriptionFromFile("/home/hitstar/catkin_ws/src/quadruped_locomotion-dev/quadruped_model/urdf/simpledog.urdf");
//  LoadRobotDescriptionFromFile("/home/hitstar/catkin_ws/src/quadruped_locomotion-dev/quadruped_model/urdf/simpledog.urdf");
  std::cout<<"Constructor QuadrupedState"<<std::endl;

  setPoseBaseToWorld(Pose(Position(0,0,0), RotationQuaternion()));
  joint_positions_ << 0,1.57,-3.14,0,1.57,-3.14,0,1.57,-3.14,0,1.57,-3.14;
  setCurrentLimbJoints(joint_positions_);
};

QuadrupedState::~QuadrupedState()
{
 std::cout<<"QuadrupedState Destroied"<<std::endl;
};

const Position QuadrupedState::getPositionWorldToBaseInWorldFrame() const
{
  return poseInWorldFrame_.getPosition();//TODO(Shunyao):How to update poseInWorldFrame
};

bool QuadrupedState::setPoseBaseToWorld(const Pose pose)
{
  poseInWorldFrame_ = pose;
  return true;
};
const Position QuadrupedState::getPositionWorldToFootInWorldFrame(const LimbEnum& limb)
{
  Position foot_in_base, base_in_world;
//  setCurrentLimbJoints(joint_positions_);
  foot_in_base = getPositionBaseToFootInBaseFrame(limb);
  base_in_world = poseInWorldFrame_.getPosition();
  return poseInWorldFrame_.getRotation().rotate(foot_in_base) + base_in_world;//poseInWorldFrame_.getPosition() + getPositionWorldToFootInWorldFrame(limb);
}
const Position QuadrupedState::getPositionBaseToFootInBaseFrame(const LimbEnum& limb)
{
//  std::cout<<"get here"<<std::endl;
  JointPositionsLimb jointPositions = current_limb_joints_.at(limb);
  std::cout<<"in getPositionBaseToFootInBaseFrame()  "<<jointPositions<<std::endl;
  Pose foot_pose;
  FowardKinematicsSolve(jointPositions, limb, foot_pose);
  footPoseInBaseFrame_[limb] = foot_pose;
  return foot_pose.getPosition();
}
const RotationQuaternion QuadrupedState::getOrientationBaseToWorld() const
{
  return  poseInWorldFrame_.getRotation();
}


JointPositions& QuadrupedState::getJointPositions()
{
  return joint_positions_;// joint command
}
JointVelocities& QuadrupedState::getJointVelocities()
{
  return joint_velocities_;
}

const LinearVelocity QuadrupedState::getLinearVelocityBaseInWorldFrame() const
{}
const LocalAngularVelocity QuadrupedState::getAngularVelocityBaseInBaseFrame() const
{}

bool QuadrupedState::setPositionWorldToBaseInWorldFrame(const Position position)
{
  positionWorldToBaseInWorldFrame_ = position;
  poseInWorldFrame_.getPosition() = position;
  return true;
}
bool QuadrupedState::setOrientationBaseToWorld(const RotationQuaternion rotation)
{
  orientationBaseToWorld_ = rotation;
  poseInWorldFrame_.getRotation() = rotation;
  return true;
}
bool QuadrupedState::setLinearVelocityBaseInWorldFrame(const LinearVelocity linear_velocity)
{
  return true;
}
bool QuadrupedState::setAngularVelocityBaseInBaseFrame(const LocalAngularVelocity local_angular_velocity)
{
  return true;
}

bool QuadrupedState::setJointPositions(const JointPositions joint_positions)
{
  joint_positions_ = joint_positions;
  setCurrentLimbJoints(joint_positions_);
  return true;
}
bool QuadrupedState::setJointVelocities(const JointVelocities joint_velocoties)
{
  return true;
}

Position QuadrupedState::getPositionBaseToFootInBaseFrame(const LimbEnum& limb, const JointPositionsLimb& jointPositions)
{
  Pose pose_base_to_foot_in_base;
//  std::cout<<hip_pose_in_base_.at(LimbEnum::LF_LEG)<<std::endl;
  FowardKinematicsSolve(jointPositions, limb, pose_base_to_foot_in_base);
  return pose_base_to_foot_in_base.getPosition();
}
bool QuadrupedState::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(const Position& positionBaseToFootInBaseFrame,
                                                            const LimbEnum& limb,
                                                            JointPositionsLimb& jointPositions)
{
//  std::cout<<"in quadruped state"<<current_limb_joints_.at(limb)<<std::endl;
  JointPositionsLimb joint_position_last = current_limb_joints_.at(limb);
//  std::cout<<limb<<std::endl;
  if(!InverseKinematicsSolve(positionBaseToFootInBaseFrame,limb,joint_position_last,jointPositions))
  {
    return false;
  }
  std::cout<<"***********************************"<<std::endl
     <<jointPositions<<std::endl
    <<"**********************************"<<std::endl;
  return true;

}
LinearVelocity QuadrupedState::getEndEffectorLinearVelocityFromJointVelocities(const LimbEnum& limb,
                                                                               const JointVelocitiesLimb& jointVelocities)
{
  //TODO(Shunyao) : where to update joint Positions
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd joint_diff(3);
  joint_diff << jointVelocities(0), jointVelocities(1), jointVelocities(2);
  AnalysticJacobian(current_limb_joints_.at(limb), limb, jacobian);
  Eigen::VectorXd pose_diff = jacobian * joint_diff;
  return LinearVelocity(pose_diff(0), pose_diff(1), pose_diff(2));

}

void QuadrupedState::setCurrentLimbJoints(JointPositions all_joints_position)
{

  current_limb_joints_[LimbEnum::LF_LEG] = JointPositionsLimb(all_joints_position(0),all_joints_position(1),all_joints_position(2));
  current_limb_joints_[LimbEnum::RF_LEG] = JointPositionsLimb(all_joints_position(3),all_joints_position(4),all_joints_position(5));
  current_limb_joints_[LimbEnum::RH_LEG] = JointPositionsLimb(all_joints_position(6),all_joints_position(7),all_joints_position(8));
  current_limb_joints_[LimbEnum::LH_LEG] = JointPositionsLimb(all_joints_position(9),all_joints_position(10),all_joints_position(11));


}

}; //namespace
