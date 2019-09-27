/*
 *  quadruped_state.h
 *  Descriotion:
 *
 *  Created on: , 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
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
  typedef std::unordered_map<LimbEnum, Vector, EnumClassHash> FootVectorInBase;
  typedef std::unordered_map<LimbEnum, JointPositionsLimb, EnumClassHash> limb_joints;
  typedef std::unordered_map<LimbEnum, std::string, EnumClassHash> limb_configure;
  typedef std::unordered_map<LimbEnum, Position, EnumClassHash> limb_position;
  typedef std::unordered_map<LimbEnum, double, EnumClassHash> limb_mass;
  QuadrupedState();
  ~QuadrupedState();
//  QuadrupedState(const QuadrupedState& other);
  bool Initialize();
  const Position getPositionWorldToBaseInWorldFrame() const;
  const RotationQuaternion getOrientationBaseToWorld() const;
  const Position getTargetPositionWorldToBaseInWorldFrame() const;
  const RotationQuaternion getTargetOrientationBaseToWorld() const;

  const Position getTargetFootPositionInBaseForLimb(const LimbEnum& limb) const;
  const LinearVelocity getTargetFootVelocityInBaseForLimb(const LimbEnum& limb) const;
  const LinearAcceleration getTargetFootAccelerationInBaseForLimb(const LimbEnum& limb) const;
  //! WSHY: command
  static JointPositions& getJointPositions();
  static JointVelocities& getJointVelocities();
  static JointTorques& getJointTorques();

  const JointPositions& getJointPositionFeedback() const;
  const JointPositionsLimb getJointPositionFeedbackForLimb(const LimbEnum& limb) const;
  const LinearVelocity getLinearVelocityBaseInWorldFrame() const;
  const LocalAngularVelocity getAngularVelocityBaseInBaseFrame() const;
  const LinearVelocity getTargetLinearVelocityBaseInWorldFrame() const;
  const LocalAngularVelocity getTargetAngularVelocityBaseInBaseFrame() const;

  const Position getPositionWorldToFootInWorldFrame(const LimbEnum& limb);
  const Position getPositionBaseToFootInBaseFrame(const LimbEnum& limb);

  Position getPositionBaseToFootInBaseFrame(const LimbEnum& limb, const JointPositionsLimb& jointPositions);
  bool getLimbJointPositionsFromPositionBaseToFootInBaseFrame(const Position& positionBaseToFootInBaseFrame,
                                                              const LimbEnum& limb,
                                                              JointPositionsLimb& jointPositions);
  LinearVelocity getEndEffectorLinearVelocityFromJointVelocities(const LimbEnum& limb, const JointVelocitiesLimb& jointVelocities);
  LinearVelocity getEndEffectorVelocityInBaseForLimb(const LimbEnum& limb);

  Pose getPoseFootInBaseFrame(const LimbEnum& limb);
  Pose getPoseFootInBaseFrame(const LimbEnum& limb, JointPositionsLimb& jointleg);
  double getRobotMass();
  Position getCenterOfMassInBase();
  Position getPositionLegBaseToCoMInBaseFrame(const LimbEnum& limb) const;
  double getLegMass(const LimbEnum& limb) const;
  Eigen::Matrix3d getTranslationJacobianFromBaseToFootInBaseFrame(const LimbEnum& limb);
  Eigen::Matrix3d getTranslationJacobianDotFromBaseToFootInBaseFrame(const LimbEnum& limb);
  Eigen::Matrix3d getTranslationJacobianBaseToCoMInBaseFrame(const LimbEnum& limb, const int link_index);

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
  //! WSHY: feedback
  void setCurrentLimbJoints(JointPositions all_joints_position);
  void setCurrentLimbJointVelocities(JointVelocities all_joints_velocities);
  bool setPoseBaseToWorld(const Pose pose);
  //! WSHY: Target
  bool setPositionWorldToBaseInWorldFrame(const Position position);
  bool setOrientationBaseToWorld(const RotationQuaternion rotation);
  bool setLinearVelocityBaseInWorldFrame(const LinearVelocity linear_velocity);
  bool setAngularVelocityBaseInBaseFrame(const LocalAngularVelocity local_angular_velocity);
  bool setJointPositions(const JointPositions joint_positions);
  bool setJointVelocities(const JointVelocities joint_velocities);
  bool setLimbConfigure(const std::string leg_configure = "<<");
  bool setBaseStateFromFeedback(const LinearVelocity& base_linear_velocity,
                                const LocalAngularVelocity& base_angular_velocity);
  bool setTargetFootPositionInBaseForLimb(const Position& foot_position, const LimbEnum& limb);
  bool setTargetFootVelocityInBaseForLimb(const LinearVelocity& foot_velocity, const LimbEnum& limb);
  bool setTargetFootAccelerationInBaseForLimb(const LinearAcceleration& foot_acceleration, const LimbEnum& limb);
private:
//  QuadrupedKinematics QK;
  limb_joints current_limb_joints_, current_limb_joint_velocities_;
  limb_configure limb_configure_;
  FootPoseInBase footPoseInBaseFrame_, footPoseInBase_;

  //! feedback, current actual pose
  static FootVectorInBase target_foot_position_in_base_, target_foot_velocity_in_base_,
                          target_foot_acceleration_in_base_;
  static Pose poseInWorldFrame_, footholds_plane_pose_;
  static JointPositions joint_positions_, joint_positions_feedback_;
  static JointPositions allJointPositionsFeedback_, allJointPositionsCB;
  static JointVelocities joint_velocities_, joint_velocities_feedback_;
  static JointTorques joint_torques_, joint_torques_feedback_;
  //! target base position and orientation
  static Position positionWorldToBaseInWorldFrame_;
  static RotationQuaternion orientationBaseToWorld_;
  static LinearVelocity base_feedback_linear_velocity_, base_target_linear_velocity_;
  static LocalAngularVelocity base_feedback_angular_velocity_, base_target_angular_velocity_;
  double robot_mass_;
  Position CoM_in_base_;
  limb_position limb_to_CoM_;
  limb_mass limb_mass_;
  Eigen::Matrix3d translation_jacobian_;
//   foot_positions_;

};
}//namespace

#endif // QUADRUPED_STATE_H
