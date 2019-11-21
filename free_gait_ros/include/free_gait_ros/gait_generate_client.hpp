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
#include "free_gait_msgs/SetStepParameter.h"
#include "geometry_msgs/PolygonStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

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
    bool real_contact;
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

  bool initializeCrawl(const double t_swing, const double t_stance);


  bool sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal);

  bool isUpdated();

  bool isDone();

  bool isActive();

  void activeCallback();

  bool copyRobotState(const free_gait::State& state);

  bool generateFootHolds(std::string frame);

  bool updateBaseMotion(LinearVelocity& desired_linear_velocity,
                        LocalAngularVelocity& desired_angular_velocity);
  bool optimizePose(free_gait::Pose& pose);

  bool sendMotionGoal();

  void velocityCommandCallback(const geometry_msgs::TwistConstPtr& twist);

  std::string getLimbStringFromLimbEnum(const free_gait::LimbEnum& limb) const;

  bool advance(double dt);

//  bool updateBaseVelocity(LinearVelocity& desired_linear_velocity,
//                          LocalAngularVelocity& desired_angular_velocity);


  bool publishMarkers();

  Pose getDesiredBasePose();

  std::vector<LinearVelocity> current_velocity_buffer_;
  bool ignore_vd;

  int getGaitType();


private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber velocity_command_sub_;
  ros::ServiceServer gaitSwitchServer_, stepParameterServer_;
  ros::Publisher foot_marker_pub_, com_proj_marker_pub_, desired_base_com_marker_pub_, support_polygon_pub_, weight_pub_, desired_com_path_pub_;

  free_gait::State robot_state_;
  Pose base_pose;
  bool is_updated, is_done, is_active, use_terrian_map;
  std::unique_ptr<free_gait::FreeGaitActionClient> action_client_ptr;
  double height_, t_swing_delay, step_displacement, profile_height, t_swing_, t_stance_, sigma_sw_0, sigma_sw_1, sigma_st_0, sigma_st_1;
  std::string profile_type;
  int step_number;
  Position LF_nominal, RF_nominal, LH_nominal, RH_nominal, P_CoM_desired_;
  Position footholds_in_stance, footprint_center_in_base, footprint_center_in_world;
  Pose start_pose, desired_base_pose_;
  geometry_msgs::Vector3Stamped surface_normal;
  geometry_msgs::PointStamped lf_foot_holds, rf_foot_holds, lh_foot_holds, rh_foot_holds;
  nav_msgs::Path desired_com_path_;

  LinearVelocity desired_linear_velocity_, desired_linear_velocity_world_;
  LocalAngularVelocity desired_angular_velocity_;

  free_gait::Step step_;
  LegMotions leg_motions_;
  std::unique_ptr<free_gait::BaseMotionBase> base_motion_;

  LimbPhase limb_phase;
  free_gait::Stance foothold_in_support_, start_footholds, hip_to_foot_in_base;
//  std::unique_ptr<free_gait::PoseOptimizationGeometric> poseOptimizationGeometric_;
//  const free_gait::StepParameters& parameters_;
  free_gait::PlanarStance nominalPlanarStanceInBaseFrame;
  free_gait::Stance nominalStanceInBaseFrame_, stanceForOrientation_, hip_dispacement;

  free_gait_msgs::ExecuteStepsGoal motion_goal_;
  free_gait_msgs::Step step_msg_;
  free_gait_msgs::BaseAuto base_auto_msg_;
  free_gait_msgs::BaseTarget base_target_msg_;
  free_gait_msgs::Footstep footstep_msg_;
  bool base_auto_flag, base_target_flag, pace_flag, trot_flag, crawl_flag, crawl_leg_switched, pre_step;

  std::vector<int> crawl_leg_order;

  void feedbackCallback(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback);

  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const free_gait_msgs::ExecuteStepsResult& result);
  bool setStepParameterCallback(free_gait_msgs::SetStepParameterRequest& req,
                                free_gait_msgs::SetStepParameterResponse& res);

  std::unique_ptr<FootstepOptimization> footstepOptimization;

  Pose optimized_base_pose;

//  std::vector<LinearVelocity> current_velocity_buffer_;




};
