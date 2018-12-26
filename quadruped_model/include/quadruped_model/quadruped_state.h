#ifndef QUADRUPED_STATE_H
#define QUADRUPED_STATE_H
#include "quadruped_model/QuadrupedModel.hpp"
#include "quadruped_model/quadrupedkinematics.h"
namespace quadruped_model
{
class QuadrupedState : public QuadrupedKinematics
{
public:
  typedef std::unordered_map<LimbEnum, Pose, EnumClassHash> FootPoseInBase;
  typedef std::unordered_map<LimbEnum, JointPositionsLimb, EnumClassHash> limb_joints;
  QuadrupedState();
  ~QuadrupedState();
  const Position getPositionWorldToBaseInWorldFrame() const;
  const RotationQuaternion getOrientationBaseToWorld() const;
  static JointPositions& getJointPositions();
  static JointVelocities& getJointVelocities();
  const LinearVelocity getLinearVelocityBaseInWorldFrame() const;
  const LocalAngularVelocity getAngularVelocityBaseInBaseFrame() const;

  const Position getPositionWorldToFootInWorldFrame(const LimbEnum& limb);
  const Position getPositionBaseToFootInBaseFrame(const LimbEnum& limb);

  Position getPositionBaseToFootInBaseFrame(const LimbEnum& limb, const JointPositionsLimb& jointPositions);
  bool getLimbJointPositionsFromPositionBaseToFootInBaseFrame(const Position& positionBaseToFootInBaseFrame,
                                                              const LimbEnum& limb,
                                                              JointPositionsLimb& jointPositions);
  LinearVelocity getEndEffectorLinearVelocityFromJointVelocities(const LimbEnum& limb, const JointVelocitiesLimb& jointVelocities);
//  struct getJointPositions
//  {
//    Eigen::VectorXd joint_position_;
//    Eigen::VectorXd vector()
//    {
//       return Eigen::VectorXd(joint_position_);
//    }

//    void setSegment(Eigen::Index start, Eigen::Index n, JointPositionsLimb joint_position_limb)
//    {
//      joint_position_.segment(start, n) = joint_position_limb.vector();
//    }
//  };
  void setCurrentLimbJoints(JointPositions all_joints_position);
  bool setPoseBaseToWorld(const Pose pose);
  bool setPositionWorldToBaseInWorldFrame(const Position position);
  bool setOrientationBaseToWorld(const RotationQuaternion rotation);
  bool setLinearVelocityBaseInWorldFrame(const LinearVelocity linear_velocity);
  bool setAngularVelocityBaseInBaseFrame(const LocalAngularVelocity local_angular_velocity);
  bool setJointPositions(const JointPositions joint_positions);
  bool setJointVelocities(const JointVelocities joint_velocities);
private:
//  QuadrupedKinematics QK;
  limb_joints current_limb_joints_;
  FootPoseInBase footPoseInBaseFrame_;
  Pose poseInWorldFrame_;
  static JointPositions joint_positions_, joint_positions_feedback_;
  static JointVelocities joint_velocities_;
  Position positionWorldToBaseInWorldFrame_;
  RotationQuaternion orientationBaseToWorld_;
};
}//namespace

#endif // QUADRUPED_STATE_H
