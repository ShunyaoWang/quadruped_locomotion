/*
 *  gait_generate_client.hpp
 *  Descriotion:
 *
 *  Created on: Mar 26, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#pragma once

#include "free_gait_ros/free_gait_ros.hpp"
#include "free_gait_msgs/ExecuteStepsActionGoal.h"
#include "free_gait_msgs/RobotState.h"
#include "free_gait_core/free_gait_core.hpp"
#include "free_gait_msgs/Footstep.h"
#include "pluginlib/class_loader.h"
#include "kindr_ros/kindr_ros.hpp"

#include "free_gait_ros/FootstepOptimization.hpp"
#include "free_gait_ros/RosVisualization.hpp"

class GaitGenerateClient
{
public:
  struct phase{
    double swing_phase;
    double stance_phase;
    bool stance_status;
    bool swing_status;
    double weight_to_CoM;
    Position virtual_point;
    bool ready_to_swing;
  };

   typedef std::unordered_map<free_gait::LimbEnum,
  std::unique_ptr<free_gait::LegMotionBase>,
  EnumClassHash> LegMotions;

  typedef std::unordered_map<free_gait::LimbEnum, phase, EnumClassHash> LimbPhase;

  GaitGenerateClient(const ros::NodeHandle& node_handle);
  ~GaitGenerateClient();

  void initialize();

  bool initializeTrot(const double t_swing, const double t_stance);

  bool initializePace(const double t_swing, const double t_stance);


  bool sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal);

  bool isUpdated();

  bool isDone();

  bool isActive();

  void activeCallback();

  bool copyRobotState(const free_gait::State& state);

  bool generateFootHolds(const std::string frame);

  bool updateBaseMotion(LinearVelocity& desired_linear_velocity,
                        LocalAngularVelocity& desired_angular_velocity);
  bool optimizePose(free_gait::Pose& pose);

  bool sendMotionGoal();

  void velocityCommandCallback(const geometry_msgs::TwistConstPtr& twist);

  std::string getLimbStringFromLimbEnum(const free_gait::LimbEnum& limb) const;

  bool advance(double dt);

//  bool updateBaseVelocity(LinearVelocity& desired_linear_velocity,
//                          LocalAngularVelocity& desired_angular_velocity);







private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber velocity_command_sub_;
  ros::ServiceServer gaitSwitchServer_;
  ros::Publisher foot_marker_pub_;

  free_gait::State robot_state_;
  Pose base_pose;
  bool is_updated, is_done, is_active;
  std::unique_ptr<free_gait::FreeGaitActionClient> action_client_ptr;
  double height_, t_swing_, t_stance_, sigma_sw_0, sigma_sw_1, sigma_st_0, sigma_st_1;
  int step_number;
  Position LF_nominal, RF_nominal, LH_nominal, RH_nominal, P_CoM_desired_;
  Position footholds_in_stance, footprint_center_in_base, footprint_center_in_world;
  geometry_msgs::Vector3Stamped surface_normal;
  geometry_msgs::PointStamped lf_foot_holds, rf_foot_holds, lh_foot_holds, rh_foot_holds;

  LinearVelocity desired_linear_velocity_, desired_linear_velocity_world_;
  LocalAngularVelocity desired_angular_velocity_;

  free_gait::Step step_;
  LegMotions leg_motions_;
  std::unique_ptr<free_gait::BaseMotionBase> base_motion_;

  LimbPhase limb_phase;
  free_gait::Stance foothold_in_support_;
//  std::unique_ptr<free_gait::PoseOptimizationGeometric> poseOptimizationGeometric_;
//  const free_gait::StepParameters& parameters_;
  free_gait::PlanarStance nominalPlanarStanceInBaseFrame;
  free_gait::Stance nominalStanceInBaseFrame_, stanceForOrientation_;

  free_gait_msgs::ExecuteStepsGoal motion_goal_;
  free_gait_msgs::Step step_msg_;
  free_gait_msgs::BaseAuto base_auto_msg_;
  free_gait_msgs::BaseTarget base_target_msg_;
  free_gait_msgs::Footstep footstep_msg_;
  bool base_auto_flag, base_target_flag, pace_flag, trot_flag;

  void feedbackCallback(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback);

  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const free_gait_msgs::ExecuteStepsResult& result);

  std::unique_ptr<FootstepOptimization> footstepOptimization;




};
