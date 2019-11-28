/*
 *  gait_generate_client.cpp
 *  Descriotion:
 *
 *  Created on: Mar 26, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 *
 *
//  __/\\\______________/\\\_____/\\\\\\\\\\\____/\\\________/\\\__/\\\________/\\\_
//   _\/\\\_____________\/\\\___/\\\/////////\\\_\/\\\_______\/\\\_\///\\\____/\\\/__
//    _\/\\\_____________\/\\\__\//\\\______\///__\/\\\_______\/\\\___\///\\\/\\\/____
//     _\//\\\____/\\\____/\\\____\////\\\_________\/\\\\\\\\\\\\\\\_____\///\\\/______
//      __\//\\\__/\\\\\__/\\\________\////\\\______\/\\\/////////\\\_______\/\\\_______
//       ___\//\\\/\\\/\\\/\\\____________\////\\\___\/\\\_______\/\\\_______\/\\\_______
//        ____\//\\\\\\//\\\\\______/\\\______\//\\\__\/\\\_______\/\\\_______\/\\\_______
//         _____\//\\\__\//\\\______\///\\\\\\\\\\\/___\/\\\_______\/\\\_______\/\\\_______
//          ______\///____\///_________\///////////_____\///________\///________\///________

 */
#include "free_gait_ros/gait_generate_client.hpp"

using namespace free_gait;

  GaitGenerateClient::GaitGenerateClient(const ros::NodeHandle& node_handle)//, FreeGaitActionClient& actionClient)
    : nodeHandle_(node_handle),
      is_done(false),
      is_active(false),
      is_updated(false),
      use_terrian_map(false),
      profile_height(0.18),
      step_displacement(0.2),
      profile_type("triangle"),
      crawl_flag(false),
      ignore_vd(false),
      t_swing_delay(0.0),
      update_start_pose_flag_(false),
      crawl_support_margin(0.06)
//      actionClient_(nodeHandle_)
  {
    if(!nodeHandle_.getParam("/use_terrian_map", use_terrian_map))
      {
        ROS_ERROR("Can't get parameter '/use_terrian_map', nodehandle namespace is %s", nodeHandle_.getNamespace().c_str());
      }
    if(!nodeHandle_.getParam("/step_height", profile_height))
      {
        ROS_ERROR("Can't get parameter '/step_height', nodehandle namespace is %s", nodeHandle_.getNamespace().c_str());
      }
    if(!nodeHandle_.getParam("/profile_type", profile_type))
      {
        ROS_ERROR("Can't get parameter '/profile_type', nodehandle namespace is %s", nodeHandle_.getNamespace().c_str());
      }
    if(!nodeHandle_.getParam("/step_displacement", step_displacement))
      {
        ROS_ERROR("Can't get parameter '/profile_type', nodehandle namespace is %s", nodeHandle_.getNamespace().c_str());
      }


    initialize();
    footstepOptimization.reset(new FootstepOptimization(nodeHandle_));

    velocity_command_sub_ = nodeHandle_.subscribe("/cmd_vel", 1, &GaitGenerateClient::velocityCommandCallback, this);
    foot_marker_pub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("desired_footholds", 1);
    com_proj_marker_pub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("base_center_projection", 1);
    desired_base_com_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("desired_base_center", 1);
    support_polygon_pub_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("/support_region", 1);
    weight_pub_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>("/leg_weight",1);
    desired_com_path_pub_ = nodeHandle_.advertise<nav_msgs::Path>("/desired_com_path",1);

    stepParameterServer_ = nodeHandle_.advertiseService("/step_parameters", &GaitGenerateClient::setStepParameterCallback, this);

    action_client_ptr.reset(new free_gait::FreeGaitActionClient(nodeHandle_));
//    free_gait::FreeGaitActionClient actionClient(nodeHandle_);

//    actionClient.registerCallback(std::bind(&GaitGenerateClient::activeCallback, this),
//                                  std::bind(&GaitGenerateClient::feedbackCallback, this, std::placeholders::_1),
//                                  std::bind(&GaitGenerateClient::doneCallback, this, std::placeholders::_1, std::placeholders::_2));

    action_client_ptr->registerCallback(boost::bind(&GaitGenerateClient::activeCallback, this),
                                  boost::bind(&GaitGenerateClient::feedbackCallback, this, _1),
                                  boost::bind(&GaitGenerateClient::doneCallback, this, _1, _2));


  }

  GaitGenerateClient::~GaitGenerateClient() {

  }

  void GaitGenerateClient::initialize()
  {
    surface_normal.vector.x = 0.0;
    surface_normal.vector.y = 0.0;
    surface_normal.vector.z = 1.0;
    height_ = 0.5;
    step_number = 0;
    sigma_st_0 = 0.8;
    sigma_st_1 = 0.8;
    sigma_sw_0 = 1;
    sigma_sw_1 = 1;

    limb_phase[LimbEnum::LF_LEG].swing_phase = 0;
    limb_phase[LimbEnum::LF_LEG].stance_phase = 0;
    limb_phase[LimbEnum::LF_LEG].swing_status = false;
    limb_phase[LimbEnum::LF_LEG].stance_status = false;
    limb_phase[LimbEnum::RF_LEG].swing_phase = 0;
    limb_phase[LimbEnum::RF_LEG].stance_phase = 0;
    limb_phase[LimbEnum::RF_LEG].swing_status = false;
    limb_phase[LimbEnum::RF_LEG].stance_status = false;
    limb_phase[LimbEnum::RH_LEG].swing_phase = 0;
    limb_phase[LimbEnum::RH_LEG].stance_phase = 0;
    limb_phase[LimbEnum::RH_LEG].swing_status = false;
    limb_phase[LimbEnum::RH_LEG].stance_status = false;
    limb_phase[LimbEnum::LH_LEG].swing_phase = 0;
    limb_phase[LimbEnum::LH_LEG].stance_phase = 0;
    limb_phase[LimbEnum::LH_LEG].swing_status = false;
    limb_phase[LimbEnum::LH_LEG].stance_status = false;

    Position2 position;
    position << 0.4, 0.25;
    nominalPlanarStanceInBaseFrame.emplace(LimbEnum::LF_LEG, position);
    nominalPlanarStanceInBaseFrame.emplace(LimbEnum::RF_LEG, Position2(Eigen::Vector2d(position(0), -position(1))));
    nominalPlanarStanceInBaseFrame.emplace(LimbEnum::LH_LEG, Position2(Eigen::Vector2d(-position(0), position(1))));
    nominalPlanarStanceInBaseFrame.emplace(LimbEnum::RH_LEG, Position2(Eigen::Vector2d(-position(0), -position(1))));


    hip_dispacement.emplace(LimbEnum::LF_LEG, Position(0,step_displacement,0));
    hip_dispacement.emplace(LimbEnum::RF_LEG, Position(0,-step_displacement,0));
    hip_dispacement.emplace(LimbEnum::LH_LEG, Position(0,step_displacement,0));
    hip_dispacement.emplace(LimbEnum::RH_LEG, Position(0,-step_displacement,0));

    hip_to_foot_in_base.emplace(LimbEnum::LF_LEG, Position(0,0,-0.5));
    hip_to_foot_in_base.emplace(LimbEnum::RF_LEG, Position(0,0,-0.5));
    hip_to_foot_in_base.emplace(LimbEnum::RH_LEG, Position(0,0,-0.5));
    hip_to_foot_in_base.emplace(LimbEnum::LH_LEG, Position(0,0,-0.5));

  }

  bool GaitGenerateClient::initializeTrot(const double t_swing, const double t_stance)
  {
    sigma_st_0 = 0.8;
    sigma_st_1 = 0.8;
    sigma_sw_0 = 1;
    sigma_sw_1 = 1;

    step_msg_.base_auto.clear();
    step_msg_.base_target.clear();
    step_msg_.footstep.clear();

    optimized_base_pose.getPosition() = robot_state_.getPositionWorldToBaseInWorldFrame();
    optimized_base_pose.getRotation() = robot_state_.getOrientationBaseToWorld();
    t_swing_ = t_swing;
    t_stance_ = t_stance;
//    step_msg_.base_target.resize(1);
//    step_msg_.base_auto.resize(1);
//    step_msg_.footstep.resize(4);
    limb_phase[LimbEnum::LF_LEG].swing_phase = 0;
    limb_phase[LimbEnum::LF_LEG].stance_phase = 0;
    limb_phase[LimbEnum::LF_LEG].swing_status = true;
    limb_phase[LimbEnum::LF_LEG].stance_status = false;
    limb_phase[LimbEnum::LF_LEG].ready_to_swing = true;
    limb_phase[LimbEnum::LF_LEG].real_contact = false;

    limb_phase[LimbEnum::RF_LEG].swing_phase = 0;
    limb_phase[LimbEnum::RF_LEG].stance_phase = 0;
    limb_phase[LimbEnum::RF_LEG].swing_status = false;
    limb_phase[LimbEnum::RF_LEG].stance_status = true;
    limb_phase[LimbEnum::RF_LEG].ready_to_swing = false;
    limb_phase[LimbEnum::RF_LEG].real_contact = true;

    limb_phase[LimbEnum::RH_LEG].swing_phase = 0;
    limb_phase[LimbEnum::RH_LEG].stance_phase = 0;
    limb_phase[LimbEnum::RH_LEG].swing_status = true;
    limb_phase[LimbEnum::RH_LEG].stance_status = false;
    limb_phase[LimbEnum::RH_LEG].ready_to_swing = true;
    limb_phase[LimbEnum::RH_LEG].real_contact = false;

    limb_phase[LimbEnum::LH_LEG].swing_phase = 0;
    limb_phase[LimbEnum::LH_LEG].stance_phase = 0;
    limb_phase[LimbEnum::LH_LEG].swing_status = false;
    limb_phase[LimbEnum::LH_LEG].stance_status = true;
    limb_phase[LimbEnum::LH_LEG].ready_to_swing = false;
    limb_phase[LimbEnum::LH_LEG].real_contact = true;
    base_auto_flag = false;
    base_target_flag = false;
    pace_flag =false;
    trot_flag = true;
    crawl_flag = false;
    return true;

  }

  bool GaitGenerateClient::initializePace(const double t_swing, const double t_stance)
  {
//    sigma_st_0 = 0.2;
//    sigma_st_1 = 0.2;
//    sigma_sw_0 = 0.5;
//    sigma_sw_1 = 0.5;

    height_ = 0.5;
    sigma_st_0 = 0.2;
    sigma_st_1 = 0.2;
    sigma_sw_0 = 0.5;
    sigma_sw_1 = 0.5;

    t_swing_delay = 0.2;

//    sigma_st_0 = 0.1;
//    sigma_st_1 = 0.1;
//    sigma_sw_0 = 1.0;
//    sigma_sw_1 = 1.0;

    step_msg_.base_auto.clear();
    step_msg_.base_target.clear();
    step_msg_.footstep.clear();
    t_swing_ = t_swing + t_swing_delay;
    t_stance_ = t_stance;
//    step_msg_.base_auto.resize(1);
//    step_msg_.base_target.resize(1);
//    limb_phase[LimbEnum::LF_LEG].swing_phase = 0;
//    limb_phase[LimbEnum::LF_LEG].stance_phase = 0;
//    limb_phase[LimbEnum::LF_LEG].swing_status = true;
//    limb_phase[LimbEnum::LF_LEG].stance_status = false;
//    limb_phase[LimbEnum::LF_LEG].ready_to_swing = true;
//    limb_phase[LimbEnum::LF_LEG].real_contact = false;

//    limb_phase[LimbEnum::RF_LEG].swing_phase = 0;
//    limb_phase[LimbEnum::RF_LEG].stance_phase = t_stance_/3;;
//    limb_phase[LimbEnum::RF_LEG].swing_status = false;
//    limb_phase[LimbEnum::RF_LEG].stance_status = true;
//    limb_phase[LimbEnum::RF_LEG].ready_to_swing = false;
//    limb_phase[LimbEnum::RF_LEG].real_contact = true;

//    limb_phase[LimbEnum::RH_LEG].swing_phase = 0;
//    limb_phase[LimbEnum::RH_LEG].stance_phase = 2*t_stance_/3;
//    limb_phase[LimbEnum::RH_LEG].swing_status = false;
//    limb_phase[LimbEnum::RH_LEG].stance_status = true;
//    limb_phase[LimbEnum::RH_LEG].ready_to_swing = false;
//    limb_phase[LimbEnum::RH_LEG].real_contact = true;

//    limb_phase[LimbEnum::LH_LEG].swing_phase = 0;
//    limb_phase[LimbEnum::LH_LEG].stance_phase = 0;
//    limb_phase[LimbEnum::LH_LEG].swing_status = false;
//    limb_phase[LimbEnum::LH_LEG].stance_status = true;
//    limb_phase[LimbEnum::LH_LEG].ready_to_swing = false;
//    limb_phase[LimbEnum::LH_LEG].real_contact = true;
    //! WSHY: this is for base move to support area before lift up the swing leg
//    t_swing_delay = 0.2;//t_swing/4;
    t_stance_ = 3*t_swing_;// +4*t_swing_delay;
    limb_phase[LimbEnum::LF_LEG].swing_phase = 0;
    limb_phase[LimbEnum::LF_LEG].stance_phase = t_stance_;//t_stance_- t_swing_delay;
    limb_phase[LimbEnum::LF_LEG].swing_status = false;
    limb_phase[LimbEnum::LF_LEG].stance_status = true;
    limb_phase[LimbEnum::LF_LEG].ready_to_swing = false;
    limb_phase[LimbEnum::LF_LEG].real_contact = true;

    limb_phase[LimbEnum::RF_LEG].swing_phase = 0;
    limb_phase[LimbEnum::RF_LEG].stance_phase = t_stance_ -  2*t_swing_;//t_stance_ - (3*t_swing_delay +2*t_swing_);;
    limb_phase[LimbEnum::RF_LEG].swing_status = false;
    limb_phase[LimbEnum::RF_LEG].stance_status = true;
    limb_phase[LimbEnum::RF_LEG].ready_to_swing = false;
    limb_phase[LimbEnum::RF_LEG].real_contact = true;

    limb_phase[LimbEnum::RH_LEG].swing_phase = 0;
    limb_phase[LimbEnum::RH_LEG].stance_phase = t_stance_ -  t_swing_;//t_stance_ - (2*t_swing_delay +t_swing_);
    limb_phase[LimbEnum::RH_LEG].swing_status = false;
    limb_phase[LimbEnum::RH_LEG].stance_status = true;
    limb_phase[LimbEnum::RH_LEG].ready_to_swing = false;
    limb_phase[LimbEnum::RH_LEG].real_contact = true;

    limb_phase[LimbEnum::LH_LEG].swing_phase = 0;
    limb_phase[LimbEnum::LH_LEG].stance_phase = 0;
    limb_phase[LimbEnum::LH_LEG].swing_status = false;
    limb_phase[LimbEnum::LH_LEG].stance_status = true;
    limb_phase[LimbEnum::LH_LEG].ready_to_swing = false;
    limb_phase[LimbEnum::LH_LEG].real_contact = true;
    base_auto_flag = false;
    base_target_flag = false;
    pace_flag =true;
    trot_flag = false;
    crawl_flag = false;
    update_start_pose_flag_ = false;
    if(base_auto_flag)
      step_msg_.base_auto.resize(1);
    if(base_target_flag)
      step_msg_.base_target.resize(1);
    return true;
  }

  bool GaitGenerateClient::initializeCrawl(const double t_swing, const double t_stance)
  {
    t_swing_ = t_swing;
    t_stance_ = t_stance;

    height_ = 0.55;
//    step_msg_.base_auto.resize(1);
//    step_msg_.base_target.resize(1);
    limb_phase[LimbEnum::LF_LEG].swing_phase = 0;
    limb_phase[LimbEnum::LF_LEG].stance_phase = 0;
    limb_phase[LimbEnum::LF_LEG].swing_status = true;
    limb_phase[LimbEnum::LF_LEG].stance_status = false;
    limb_phase[LimbEnum::LF_LEG].ready_to_swing = true;
    limb_phase[LimbEnum::LF_LEG].real_contact = false;

    limb_phase[LimbEnum::RF_LEG].swing_phase = 0;
    limb_phase[LimbEnum::RF_LEG].stance_phase = 0;
    limb_phase[LimbEnum::RF_LEG].swing_status = false;
    limb_phase[LimbEnum::RF_LEG].stance_status = true;
    limb_phase[LimbEnum::RF_LEG].ready_to_swing = false;
    limb_phase[LimbEnum::RF_LEG].real_contact = true;

    limb_phase[LimbEnum::RH_LEG].swing_phase = 0;
    limb_phase[LimbEnum::RH_LEG].stance_phase = 2*t_stance_/3;
    limb_phase[LimbEnum::RH_LEG].swing_status = false;
    limb_phase[LimbEnum::RH_LEG].stance_status = true;
    limb_phase[LimbEnum::RH_LEG].ready_to_swing = false;
    limb_phase[LimbEnum::RH_LEG].real_contact = true;

    limb_phase[LimbEnum::LH_LEG].swing_phase = 0;
    limb_phase[LimbEnum::LH_LEG].stance_phase = t_stance_/3;
    limb_phase[LimbEnum::LH_LEG].swing_status = false;
    limb_phase[LimbEnum::LH_LEG].stance_status = true;
    limb_phase[LimbEnum::LH_LEG].ready_to_swing = false;
    limb_phase[LimbEnum::LH_LEG].real_contact = true;
    crawl_flag = true;
    trot_flag = false;
    pace_flag = false;
    crawl_leg_order.resize(4);
    crawl_leg_order[0] = 0;
    crawl_leg_order[1] = 2;
    crawl_leg_order[2] = 1;
    crawl_leg_order[3] = 3;
    crawl_leg_switched = true;
    pre_step = true;
    is_done = false;
    step_msg_.base_auto.clear();
    step_msg_.base_target.clear();
    step_msg_.footstep.clear();
    update_start_pose_flag_ = false;
    return true;
  }

  bool GaitGenerateClient::sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal)
  {
    action_client_ptr->sendGoal(goal);
    return true;
  }

  bool GaitGenerateClient::isUpdated()
  {
    return is_updated;
  }
  bool GaitGenerateClient::isDone()
  {
    return is_done;
  }
  bool GaitGenerateClient::isActive()
  {
    return is_active;
  }

  void GaitGenerateClient::activeCallback()
  {
    is_active = true;
    is_done = false;
//    ROS_INFO("Goal just went active");
  }

  void GaitGenerateClient::feedbackCallback(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback)
  {
//    ROS_INFO("step time is : %f", feedback->phase);
  }

  void GaitGenerateClient::doneCallback(const actionlib::SimpleClientGoalState& state,
                    const free_gait_msgs::ExecuteStepsResult& result)
  {
//    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    is_done = true;
    is_active = false;
  }

  void GaitGenerateClient::velocityCommandCallback(const geometry_msgs::TwistConstPtr& twist)
  {
    //! WSHY: velocity in base
    desired_linear_velocity_(0) = twist->linear.x;
    desired_linear_velocity_(1) = twist->linear.y;
    desired_linear_velocity_(2) = twist->linear.z;

    desired_angular_velocity_(0) = twist->angular.x;
    desired_angular_velocity_(1) = twist->angular.y;
    desired_angular_velocity_(2) = twist->angular.z;
  }

  bool GaitGenerateClient::setStepParameterCallback(free_gait_msgs::SetStepParameterRequest& req,
                                                    free_gait_msgs::SetStepParameterResponse& res)
  {
    profile_type = req.profile_type;
    profile_height = req.profile_height;
    ROS_INFO("Request to Change foot step profile type to '%s' with height %f", profile_type.c_str(), profile_height);
    res.result = true;
    return true;

  }

  bool GaitGenerateClient::copyRobotState(const free_gait::State& state)
  {
    robot_state_ = state;

//    current_velocity_buffer_.push_back(robot_state_.getOrientationBaseToWorld().inverseRotate(robot_state_.getLinearVelocityBaseInWorldFrame()););
    desired_linear_velocity_world_ = robot_state_.getOrientationBaseToWorld().rotate(desired_linear_velocity_);
//    EulerAnglesZyx EulerZYXBaseInWorld = EulerAnglesZyx(robot_state_.getOrientationBaseToWorld());
//    RotationQuaternion orientaionBaseInWorld = RotationQuaternion(EulerAnglesZyx(EulerZYXBaseInWorld.setUnique().vector()(0), 0, 0));
//    desired_linear_velocity_world_ = orientaionBaseInWorld.rotate(desired_linear_velocity_);

//    foothold_in_support_.clear();
    Position foot_sum;
    for(int i = 0;i<4;i++)
      {

        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        limb_phase.at(limb).real_contact = robot_state_.isSupportLeg(limb);
        if(robot_state_.isSupportLeg(limb))
          {
            foothold_in_support_[limb] = robot_state_.getPositionWorldToFootInWorldFrame(limb);
            stanceForOrientation_[limb] = foothold_in_support_[limb];
            foot_sum += foothold_in_support_[limb];
          }
      }
//    std::cout<<robot_state_.getNumberOfSupportLegs()<<" Foots Sum : "<<foot_sum<<std::endl;
    if(robot_state_.getNumberOfSupportLegs()>0)
      footprint_center_in_world = foot_sum/robot_state_.getNumberOfSupportLegs();
    publishMarkers();
    return true;
  }

  bool GaitGenerateClient::publishMarkers()
  {
    visualization_msgs::Marker base_marker;
    visualization_msgs::MarkerArray com_proj_markers;
    std_msgs::ColorRGBA color;
    color.a = 1;
    color.b = 1;

    Position com_pos = robot_state_.getPositionWorldToBaseInWorldFrame();
    color.g = 1;
    com_proj_markers = free_gait::RosVisualization::getComWithProjectionMarker(com_pos,"odom",color,0.08,0.5,0.02);
    com_proj_marker_pub_.publish(com_proj_markers);

    geometry_msgs::PolygonStamped support_region;
    support_region.header.frame_id = "odom";
    support_region.header.stamp = ros::Time::now();
    for(int i = 0;i<4;i++)
      {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        if(robot_state_.isSupportLeg(limb))
          {
            geometry_msgs::Point32 point;
            kindr_ros::convertToRosGeometryMsg(robot_state_.getPositionWorldToFootInWorldFrame(limb), point);
            support_region.polygon.points.push_back(point);
          }

      }
    support_polygon_pub_.publish(support_region);
  }
  bool GaitGenerateClient::generateFootHolds(std::string frame)
  {
//    ROS_INFO("In Generate FootHolds");
    footholds_in_stance.setZero();
//    double t_stance = 0.4;
    double height = robot_state_.getPositionWorldToBaseInWorldFrame()(2);
//    double profile_height = 0.18;
    LinearVelocity current_vel_in_base = robot_state_.getOrientationBaseToWorld().inverseRotate(robot_state_.getLinearVelocityBaseInWorldFrame());
    current_velocity_buffer_.push_back(current_vel_in_base);

    Position displace_in_footprint;
    Stance stance_to_reach;
    visualization_msgs::MarkerArray foot_markers;
    for(int i = 0;i<4;i++)
      {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);

        if(!robot_state_.isSupportLeg(limb) && limb_phase.at(limb).ready_to_swing)
          {
            LinearVelocity average_vel;
            average_vel.setZero();
            for(int k = current_velocity_buffer_.size();k>20;k--)
              {
                average_vel += current_velocity_buffer_[k];
              }
            average_vel = average_vel/(current_velocity_buffer_.size()-20);

            limb_phase.at(limb).ready_to_swing = false;
            LinearVelocity desired_vel2D, current_vel2D;
            desired_vel2D = desired_linear_velocity_;
            desired_vel2D(2) = 0.0;
            current_vel2D = average_vel;// current_vel_in_base;
//            current_vel2D(0) = 0.0;
            current_vel2D(2) = 0.0;
            double z_hip = height - footprint_center_in_world(2);
//            ROS_INFO("relative base height is : %f", z_hip);
            double yaw_angle;
            EulerAnglesZyx ypr(robot_state_.getOrientationBaseToWorld());
            yaw_angle = ypr.setUnique().vector()(0);
            RotationQuaternion orinetationFootprintToWorld = RotationQuaternion(EulerAnglesZyx(yaw_angle,0,0));
            RotationQuaternion orinetationFootprintToBase = robot_state_.getOrientationBaseToWorld().inverted() * orinetationFootprintToWorld;
//            if(pace_flag){
//                displace_in_footprint = Position(0.33*t_stance_*desired_vel2D
//                                                   + sqrt(z_hip/9.8) * (current_vel2D - desired_vel2D));
//              }
//            displace_in_footprint = Position(0.5*t_stance_*desired_vel2D);
            displace_in_footprint = Position(0.5*t_stance_*desired_vel2D
                                                 + sqrt(z_hip/9.8) * (current_vel2D - desired_vel2D));

            if(pace_flag || crawl_flag)
              {
                displace_in_footprint = Position(0.34*t_stance_*desired_vel2D);

              }
//            if(displace_in_footprint.y()>0.2)
//              displace_in_footprint.y()=0.2;
//            if(displace_in_footprint.y()<-0.2)
//              displace_in_footprint.y() = -0.2;
            displace_in_footprint(2) = 0.02;
            Position displace_in_baselink = displace_in_footprint;
//            displace_in_baselink(2) = -height_+ 0.025;

            Position displace_in_odom = orinetationFootprintToWorld.rotate(displace_in_footprint + Position(t_stance_*desired_vel2D));
            displace_in_odom(2) = 0.02;
//            Position target_in_base = Position(robot_state_.getPositionBaseToHipInBaseFrame(limb).toImplementation()
//                                       + 0.5*t_stance*desired_linear_velocity_.toImplementation()
//                                       + sqrt(height/9.8) * (current_vel - desired_linear_velocity_).toImplementation());
            Position hip_in_odom = robot_state_.getPositionWorldToBaseInWorldFrame() +
                robot_state_.getOrientationBaseToWorld().rotate(robot_state_.getPositionBaseToHipInBaseFrame(limb));
            hip_in_odom(2) = 0.0; //project to floor plane

            Position foot_in_base = robot_state_.getPositionBaseToFootInBaseFrame(limb);
//            Position hip_in_base = robot_state_.getPositionBaseToHipInBaseFrame(limb) + hip_dispacement.at(limb);

//            Position hip_in_footprint = robot_state_.getPositionBaseToHipInBaseFrame(limb) + hip_dispacement.at(limb);
            Position hip_in_baselink = robot_state_.getPositionBaseToHipInBaseFrame(limb) + hip_dispacement.at(limb)
                 + robot_state_.getOrientationBaseToWorld().inverseRotate(Position(0,0,-height_-0.03));//in sim set 0.03
            Position foothold_in_base = foot_in_base;
            foot_in_base.z() = hip_in_baselink.z();
            foot_in_base.y() = hip_in_baselink.y();
//            Position position_delta = robot_state_.getOrientationBaseToWorld().inverseRotate(
//                  optimized_base_pose.getPosition() - robot_state_.getPositionWorldToBaseInWorldFrame());

//            position_delta.z() = 0;
//            Position target_in_footprint = hip_in_footprint + displace_in_footprint;
            Position target_in_baselink = displace_in_baselink + hip_in_baselink;// + position_delta;
//            if(crawl_flag)
//              target_in_baselink = 2*displace_in_baselink + foot_in_base;
            //            target_in_footprint(2) = 0.0;
            Position target_in_odom = hip_in_odom + displace_in_odom;
            Position target_in_base =robot_state_.getOrientationBaseToWorld().inverseRotate(target_in_odom - robot_state_.getPositionWorldToBaseInWorldFrame());

            Position target_in_footprint = Position(0,0,height) + orinetationFootprintToBase.inverseRotate(target_in_baselink);
//            target_in_footprint.z() -= 0.03;
            target_in_odom = robot_state_.getPositionWorldToBaseInWorldFrame()
                + robot_state_.getOrientationBaseToWorld().rotate(target_in_baselink);

//            target_in_odom = robot_state_.getPositionWorldToBaseInWorldFrame() + Position(0,0,-height) +
//                orinetationFootprintToWorld.rotate(target_in_footprint + Position(t_stance_*desired_vel2D));

            footstep_msg_.name = getLimbStringFromLimbEnum(limb);

            if(crawl_flag || pace_flag)
              {
                frame = "odom";
                Position displace_of_angular = Position(desired_angular_velocity_.cross(robot_state_.getPositionBaseToHipInBaseFrame(limb)));

                if(limb == free_gait::LimbEnum::LF_LEG || limb == free_gait::LimbEnum::RF_LEG)
                  crawl_support_margin = 0.05;
                else
                  crawl_support_margin = 0.10;
//                Pose start_pose;
                if(limb == free_gait::LimbEnum::LF_LEG)
                  {
                    Position foot_sum;
                    foot_sum.setZero();
                    for(int i=0;i<4;i++)
                      {
                        free_gait::LimbEnum limb_i = static_cast<free_gait::LimbEnum>(i);
                        foot_sum += foothold_in_support_[limb_i];
                        hip_to_foot_in_base[limb_i] = robot_state_.getPositionBaseToFootInBaseFrame(limb_i)
                            -robot_state_.getPositionBaseToHipInBaseFrame(limb_i);
//                        std::cout<<getLimbStringFromLimbEnum(limb_i)<<" Hip to Foot In Base : "<<hip_to_foot_in_base[limb_i]<<std::endl;
                      }
//                    start_pose = Pose(robot_state_.getPositionWorldToBaseInWorldFrame(), robot_state_.getOrientationBaseToWorld());
                    optimizePose(start_pose);
                    start_pose.getPosition() = foot_sum/4;
//                    start_pose.getPosition().z() = robot_state_.getPositionWorldToBaseInWorldFrame().z();
                    start_pose.getPosition().z() += height_;
                    start_footholds = foothold_in_support_;
//                    EulerAnglesZyx(start_pose.getRotation()).yaw();


                    if(!update_start_pose_flag_)
                      {
                        next_pose_ = start_pose;
                        update_start_pose_flag_ = true;
                      }
                    next_pose_.getPosition() = next_pose_.getPosition() + next_pose_.getRotation().rotate(displace_in_baselink);// + displace_of_angular);
                    next_pose_.getPosition().z() = start_pose.getPosition().z();
                    next_pose_.getRotation() = start_pose.getRotation();


                  }
                next_pose_.getPosition().z() = start_footholds.at(limb).z() + height_;
//                hip_to_foot_in_base[limb] = robot_state_.getPositionBaseToFootInBaseFrame(limb)
//                    -robot_state_.getPositionBaseToHipInBaseFrame(limb);
//                std::cout<<getLimbStringFromLimbEnum(limb)<<" Hip to Foot In Base : "<<hip_to_foot_in_base[limb]<<std::endl;

//                double yaw = EulerAnglesZyx(start_pose.getRotation()).getUnique().vector()(0);
//                ROS_WARN_STREAM("Yaw angle is :%"<<180.0 / M_PI *yaw);
//                RotationQuaternion yaw_rotate = RotationQuaternion(EulerAnglesZyx(yaw,0,0));
                //! WSHY: use neurul point
                hip_in_baselink = robot_state_.getPositionBaseToHipInBaseFrame(limb) + hip_dispacement.at(limb)
                                 + Position(0,0,-height_);//Position(0,0,hip_to_foot_in_base.at(limb).z());//Position(0,0,-height_+0.02);//start_pose.getRotation().inverseRotate(Position(0,0,-height_-0.02));
//                target_in_baselink = displace_in_baselink + hip_in_baselink + displace_of_angular;
////                Position displace_in_odom = start_pose.getPosition() + start_pose.getRotation().rotate(displace_in_baselink + displace_of_angular);

//                target_in_odom = start_pose.getPosition()
//                    + start_pose.getRotation().rotate(target_in_baselink);
                //! WSHY: directly displace footholds
//                displace_in_odom = start_pose.getRotation().rotate(displace_in_baselink + displace_of_angular);
//                target_in_odom = start_footholds.at(limb) + displace_in_odom;
//                std::cout<<"Foothold in support"<<foothold_in_support_<<std::endl;
//                std::cout<< "current base position"<<robot_state_.getPositionWorldToBaseInWorldFrame()<<std::endl;
//                std::cout << "Start Position: " << start_pose.getPosition() << std::endl;
//                std::cout << "Start Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(start_pose.getRotation()).getUnique().vector().transpose() << std::endl;

//                if(!update_start_pose_flag_)
//                  {
//                    next_pose_ = start_pose;
//                    update_start_pose_flag_ = true;
//                  }
//                next_pose_.getPosition() = next_pose_.getPosition() + next_pose_.getRotation().rotate(displace_in_baselink + displace_of_angular);
//                next_pose_.getRotation() = next_pose_.getRotation();
                target_in_odom = next_pose_.getPosition() + next_pose_.getRotation().rotate(hip_in_baselink + displace_of_angular);

//                ROS_INFO_STREAM("Hip Projection in base_link :"<<hip_in_baselink<<std::endl);

              }
            footstep_msg_.profile_height = profile_height;
            footstep_msg_.profile_type = profile_type;

            Position target_optimized = target_in_odom;
            if(frame=="odom")
              {

                if(use_terrian_map)
                  {
                    if(!footstepOptimization->getOptimizedFoothold(target_optimized,
                                                                   robot_state_,
                                                                   limb, "odom"))
                      {
                        ROS_ERROR("Failed to Find a optimized foothold");
                        kindr_ros::convertToRosGeometryMsg(target_in_odom,
                                                           footstep_msg_.target.point);
                        kindr_ros::convertToRosGeometryMsg(Vector(0,0,1), footstep_msg_.surface_normal.vector);

                      }else{
                        ROS_WARN("success to Find a optimized foothold");
                        kindr_ros::convertToRosGeometryMsg(target_optimized,
                                                           footstep_msg_.target.point);
                        kindr_ros::convertToRosGeometryMsg(footstepOptimization->getSurfaceNormal(target_optimized),
                                                           footstep_msg_.surface_normal.vector);
                        footstep_msg_.profile_type = "square";
                        footstep_msg_.profile_height = 0.23;
                      }
                  }else{
                    kindr_ros::convertToRosGeometryMsg(target_optimized,
                                                       footstep_msg_.target.point);
                    kindr_ros::convertToRosGeometryMsg(footstepOptimization->getSurfaceNormal(target_optimized),
                                                       footstep_msg_.surface_normal.vector);


                  }

                footstep_msg_.target.header.frame_id = "odom";
                ROS_WARN_STREAM("target before optimized :"<<target_in_odom<<std::endl
                                <<"target after optimized :"<<target_optimized<<std::endl);
                stance_to_reach[limb] = target_in_odom;
                std_msgs::ColorRGBA color;
                color.a = 1;
                color.g = 1;
                foot_markers = free_gait::RosVisualization::getFootholdsMarker(stance_to_reach, frame, color, 0.08);

              }
            if(frame=="foot_print")
              {
//                ROS_WARN_STREAM("target before optimized :"<<target_in_footprint<<std::endl);
                target_optimized = robot_state_.getPositionWorldToBaseInWorldFrame() + Position(0,0,-height) +
                    orinetationFootprintToWorld.rotate(target_in_footprint + Position(t_stance_*current_vel2D));
                stance_to_reach[limb] = target_in_footprint;//target_optimized;
                if(use_terrian_map)
                  {
                    kindr_ros::convertToRosGeometryMsg(target_in_footprint,
                                                      footstep_msg_.target.point);
                    kindr_ros::convertToRosGeometryMsg(Vector(0,0,1), footstep_msg_.surface_normal.vector);
                    if(!footstepOptimization->getOptimizedFoothold(target_optimized,
                                                                   robot_state_,
                                                                   limb, "odom"))
                      {
                        ROS_ERROR("Failed to Find a optimized foothold");
                        kindr_ros::convertToRosGeometryMsg(target_in_footprint,
                                                           footstep_msg_.target.point);
                        kindr_ros::convertToRosGeometryMsg(Vector(0,0,1), footstep_msg_.surface_normal.vector);

                      }else{
                        ROS_WARN_STREAM("target before optimized :"<<target_in_footprint<<std::endl);
                        target_in_footprint = orinetationFootprintToWorld.inverseRotate(target_optimized
                            - robot_state_.getPositionWorldToBaseInWorldFrame() - Position(0,0,-height))- Position(t_stance_*current_vel2D);
//                        target_in_footprint(2) = target_in_footprint(2) - 0.05;
                        kindr_ros::convertToRosGeometryMsg(target_in_footprint,
                                                           footstep_msg_.target.point);
                        kindr_ros::convertToRosGeometryMsg(footstepOptimization->getSurfaceNormal(target_optimized),
                                                           footstep_msg_.surface_normal.vector);

                       footstep_msg_.profile_type = "square";
//                       profile_height = footstepOptimization->getMaxObstacleHeight(target_optimized,robot_state_,limb);
                       ROS_WARN_STREAM("target after optimized :"<<target_in_footprint<<std::endl);
                      }
                  }else{
                    kindr_ros::convertToRosGeometryMsg(target_in_footprint,
                                                       footstep_msg_.target.point);
                    kindr_ros::convertToRosGeometryMsg(Vector(0,0,1), footstep_msg_.surface_normal.vector);
                  }


//                kindr_ros::convertToRosGeometryMsg(target_in_footprint,
//                                                   footstep_msg_.target.point);
    //            footstep_msg_.target.point.z = 0;
                footstep_msg_.target.header.frame_id = "foot_print";

//                stance_to_reach[limb] = target_optimized;
                std_msgs::ColorRGBA color;
                color.a = 1;
                color.g = 1;
                foot_markers = free_gait::RosVisualization::getFootholdsMarker(stance_to_reach, frame, color, 0.08);

              }
            if(frame=="base_link")
              {
//                ROS_WARN_STREAM("target before optimized :"<<target_in_footprint<<std::endl);
                target_optimized = robot_state_.getPositionWorldToBaseInWorldFrame() + Position(0,0,-height) +
                    orinetationFootprintToWorld.rotate(target_in_footprint + Position(t_stance_*current_vel2D));
                stance_to_reach[limb] = target_in_baselink;//target_optimized;
                if(use_terrian_map)
                  {
                    kindr_ros::convertToRosGeometryMsg(target_in_footprint,
                                                      footstep_msg_.target.point);
                    kindr_ros::convertToRosGeometryMsg(Vector(0,0,1), footstep_msg_.surface_normal.vector);
                    if(!footstepOptimization->getOptimizedFoothold(target_optimized,
                                                                   robot_state_,
                                                                   limb, "odom"))
                      {
                        ROS_ERROR("Failed to Find a optimized foothold");
                      }else{
                        ROS_WARN_STREAM("target before optimized :"<<target_in_footprint<<std::endl);
                        target_in_footprint = orinetationFootprintToWorld.inverseRotate(target_optimized
                            - robot_state_.getPositionWorldToBaseInWorldFrame() - Position(0,0,-height))- Position(t_stance_*current_vel2D);
//                        target_in_footprint(2) = target_in_footprint(2) - 0.05;
                        kindr_ros::convertToRosGeometryMsg(target_in_footprint,
                                                           footstep_msg_.target.point);
                        kindr_ros::convertToRosGeometryMsg(footstepOptimization->getSurfaceNormal(target_optimized),
                                                           footstep_msg_.surface_normal.vector);

                       footstep_msg_.profile_type = "square";
//                       profile_height = footstepOptimization->getMaxObstacleHeight(target_optimized,robot_state_,limb);
                       ROS_WARN_STREAM("target after optimized :"<<target_in_footprint<<std::endl);
                      }
                  }else{
                    kindr_ros::convertToRosGeometryMsg(target_in_baselink,
                                                       footstep_msg_.target.point);
                    kindr_ros::convertToRosGeometryMsg(Vector(0,0,1), footstep_msg_.surface_normal.vector);
                  }


//                kindr_ros::convertToRosGeometryMsg(target_in_footprint,
//                                                   footstep_msg_.target.point);
    //            footstep_msg_.target.point.z = 0;
                footstep_msg_.target.header.frame_id = "base_link";

//                stance_to_reach[limb] = target_optimized;
                std_msgs::ColorRGBA color;
                color.a = 1;
                color.g = 1;
                foot_markers = free_gait::RosVisualization::getFootholdsMarker(stance_to_reach, frame, color, 0.08);

              }


            footstep_msg_.average_velocity = sqrt(displace_in_odom.norm()*displace_in_odom.norm() + profile_height*profile_height)*2/0.1;
//            footstep_msg_.profile_height = profile_height;
//            footstep_msg_.profile_type = profile_type;
            footstep_msg_.ignore_contact = false;
            footstep_msg_.ignore_for_pose_adaptation = false;
            if(crawl_flag)
              {
                footstep_msg_.average_velocity = 0.3;
                footstep_msg_.profile_height = 0.23;
                footstep_msg_.profile_type = "square";
              }
            //            if(pace_flag)
//              footstep_msg_.ignore_for_pose_adaptation = true;

            kindr_ros::convertToRosGeometryMsg(robot_state_.getSurfaceNormal(limb),
                                               footstep_msg_.surface_normal.vector);
//            step_msg_.footstep[i] = footstep_msg_;
            step_msg_.footstep.push_back(footstep_msg_);
//            ROS_WARN("Generate foot hold position(%f, %f, %f) for %s ",footstep_msg_.target.point.x,footstep_msg_.target.point.y,footstep_msg_.target.point.z,
//                     getLimbStringFromLimbEnum(limb).c_str());
//            stanceForOrientation_[limb] = robot_state_.getOrientationBaseToWorld().rotate(target_in_base) + robot_state_.getPositionWorldToBaseInWorldFrame();
//            footholds_in_stance += robot_state_.getPositionBaseToFootInBaseFrame(limb);
//            ROS_INFO_STREAM("Footholds limb "<<getLimbStringFromLimbEnum(limb).c_str()<<" in Base is : "<<robot_state_.getPositionBaseToFootInBaseFrame(limb)<<std::endl);


          }
//      ROS_WARN_STREAM("Displacement of foothold : "<<displace_in_footprint<<std::endl);
      }
      foot_marker_pub_.publish(foot_markers);
//    footprint_center_in_base = footholds_in_stance/robot_state_.getNumberOfSupportLegs();
//    footprint_center_in_world = robot_state_.getPositionWorldToBaseInWorldFrame()
//        + robot_state_.getOrientationBaseToWorld().rotate(footprint_center_in_base);
//    ROS_INFO_STREAM("Footprint Center in World is : "<<footprint_center_in_world<<std::endl);
//    ROS_INFO_STREAM("Footprint Center in Base is : "<<footprint_center_in_base<<std::endl);
    return true;
  }

  bool GaitGenerateClient::updateBaseMotion(LinearVelocity& desired_linear_velocity,
                                            LocalAngularVelocity& desired_angular_velocity)
  {
//    ROS_INFO("In update Base Motion");
//    ROS_WARN_STREAM("Desired Velocity :"<<desired_linear_velocity_<<std::endl);
    visualization_msgs::Marker base_marker;
    visualization_msgs::MarkerArray com_proj_markers;
    if(trot_flag)
      {
        ignore_vd = false;
        desired_linear_velocity = desired_linear_velocity_world_;
        desired_angular_velocity = desired_angular_velocity_;
      }else{
        ignore_vd = true;
        desired_linear_velocity = LinearVelocity(0,0,0);
        desired_angular_velocity = LocalAngularVelocity(0,0,0);
      }
    P_CoM_desired_.setZero();
    Position foot_sum;
    std_msgs::Float64MultiArray leg_weights;
    leg_weights.data.resize(4);
    for(int i = 0; i<4 ;i++)
      {
        /****************
* TODO(Shunyao) : Adjust the sigma to change the move of CoM
****************/
        LimbEnum limb = static_cast<LimbEnum>(i);
        double total_phase = 0;
        if(limb_phase.at(limb).swing_status)
          total_phase = limb_phase.at(limb).swing_phase/(t_swing_);
//          total_phase = limb_phase.at(limb).swing_phase/(t_stance_ + t_swing_);
        if(limb_phase.at(limb).stance_status)
          total_phase = (limb_phase.at(limb).stance_phase)/(t_stance_);
//          total_phase = (limb_phase.at(limb).stance_phase + t_swing_)/(t_stance_ + t_swing_);
        double k_st = 0.5*(erf(total_phase/(sigma_st_0*sqrt(2))) +
                           erf((1 - total_phase)/(sigma_st_1*sqrt(2))));
        double k_sw = 0.5*(2 + erf(-total_phase/(sigma_sw_0*sqrt(2))) +
                           erf((total_phase - 1)/(sigma_sw_1*sqrt(2))));
        limb_phase.at(limb).weight_to_CoM = limb_phase.at(limb).stance_status * k_st
                                            +limb_phase.at(limb).swing_status * k_sw;
        leg_weights.data[i] = limb_phase.at(limb).weight_to_CoM;

//      ROS_INFO("Calculate weight for %s\n",getLimbStringFromLimbEnum(limb).c_str());
      }
    weight_pub_.publish(leg_weights);
    for(int j = 0; j<4; j++)
      {
        LimbEnum limb_c = static_cast<LimbEnum>(j);
        //! WSHY: to decide the CW leg and CCW leg, oidered [LF,RF,RH,LH]
        int limb_cw_index = j + 1;
        int limb_ccw_index = j - 1;
        if(limb_cw_index>3)
          limb_cw_index = limb_cw_index - 4;
        if(limb_ccw_index<0)
          limb_ccw_index = limb_ccw_index + 4;
        LimbEnum limb_cw = static_cast<LimbEnum>(limb_cw_index);
        LimbEnum limb_ccw = static_cast<LimbEnum>(limb_ccw_index);
        Position foot_c, foot_cw, foot_ccw, vp_cw, vp_ccw;
        foot_cw = robot_state_.getPositionWorldToFootInWorldFrame(limb_cw);
        foot_ccw = robot_state_.getPositionWorldToFootInWorldFrame(limb_ccw);
        foot_c = robot_state_.getPositionWorldToFootInWorldFrame(limb_c);
        vp_cw = foot_c * limb_phase.at(limb_c).weight_to_CoM +
            foot_cw * (1 - limb_phase.at(limb_c).weight_to_CoM);
        vp_ccw = foot_c * limb_phase.at(limb_c).weight_to_CoM +
            foot_ccw * (1 - limb_phase.at(limb_c).weight_to_CoM);
        limb_phase.at(limb_c).virtual_point = (limb_phase.at(limb_c).weight_to_CoM * foot_c
            + limb_phase.at(limb_cw).weight_to_CoM * vp_cw + limb_phase.at(limb_ccw).weight_to_CoM
            * vp_ccw) / (limb_phase.at(limb_c).weight_to_CoM + limb_phase.at(limb_ccw).weight_to_CoM
                         + limb_phase.at(limb_cw).weight_to_CoM);
        P_CoM_desired_ = P_CoM_desired_ + limb_phase.at(limb_c).virtual_point;
//        ROS_INFO("Calculate Virtual Position for %s\n",getLimbStringFromLimbEnum(limb_c).c_str());
//        if(robot_state_.isSupportLeg(limb_c))
//          foot_sum += robot_state_.getPositionBaseToFootInBaseFrame(limb_c);
//        ROS_INFO_STREAM("Footholds limb "<<getLimbStringFromLimbEnum(limb_c).c_str()<<" in Base is : "<<robot_state_.getPositionBaseToFootInBaseFrame(limb_c)<<std::endl);

      }
    //! WSHY: When troting, we adjust the pitch to adapt ramp or stairs, but sometimes we need
    //! the base keep horizontal we should set base target pitch and yaw ZERO, but I think
    //! this can apply in a stational walk situation aimed for more stability.
//    double z_diff, pitch_angle;
//    std::vector<double> foot_height;
//    for(auto foot : foothold_in_support_)
//      {
//        foot_sum += foot.second;//robot_state_.getPositionBaseToFootInBaseFrame(foot.first);
//        foot_height.push_back(foot.second(2));
//        std::cout<<"Heightof Leg is : "<<foot.second(2)<<std::endl;
//      }
//      z_diff = foot_height.front() - foot_height.back();
//      std::cout<<"Height diff of Leg is : "<<z_diff<<std::endl;
//      pitch_angle = asin(z_diff/0.8);
////      footprint_center_in_base = foot_sum/robot_state_.getNumberOfSupportLegs();
////      footprint_center_in_world = robot_state_.getPositionWorldToBaseInWorldFrame()
////          + robot_state_.getOrientationBaseToWorld().rotate(footprint_center_in_base);
//      footprint_center_in_world = foot_sum/foothold_in_support_.size();//robot_state_.getNumberOfSupportLegs();
//      ROS_WARN_STREAM("Stance in support :"<<foothold_in_support_<<std::endl);
//      Pose optimized_base_pose;
      optimizePose(optimized_base_pose);

      P_CoM_desired_ = 0.25 * P_CoM_desired_;

      LinearVelocity current_vel = robot_state_.getLinearVelocityBaseInWorldFrame();
//      current_vel.z() = 0;
      LinearVelocity vd = desired_linear_velocity_world_;
      desired_base_pose_.getPosition() = P_CoM_desired_;
      P_CoM_desired_ = P_CoM_desired_ + Position(t_stance_*vd);

      std_msgs::ColorRGBA color;
      color.a = 1;
      color.b = 1;

      Position com_pos = robot_state_.getPositionWorldToBaseInWorldFrame();
      color.g = 1;
//      com_proj_markers = free_gait::RosVisualization::getComWithProjectionMarker(com_pos,"odom",color,0.08,0.5,0.02);
//      com_proj_marker_pub_.publish(com_proj_markers);

//      geometry_msgs::PolygonStamped support_region;
//      support_region.header.frame_id = "odom";
//      support_region.header.stamp = ros::Time::now();
//      for(int i = 0;i<4;i++)
//        {
//          free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
//          if(robot_state_.isSupportLeg(limb))
//            {
//              geometry_msgs::Point32 point;
//              kindr_ros::convertToRosGeometryMsg(robot_state_.getPositionWorldToFootInWorldFrame(limb), point);
//              support_region.polygon.points.push_back(point);
//            }

//        }
//      support_polygon_pub_.publish(support_region);


//      P_CoM_desired_(2) = height_ + footprint_center_in_world(2) - 0.02;//robot_state_.getPositionWorldToBaseInWorldFrame()(2);
      P_CoM_desired_(2) = height_ + optimized_base_pose.getPosition()(2);// - 0.03;// + t_stance_*vd.z();// - 0.02;// + t_stance_*current_vel.z();
//      P_CoM_desired_(2) += robot_state_.getPositionWorldToBaseInWorldFrame()(2) - footprint_center_in_world(2);
      desired_base_pose_.getPosition().z() = P_CoM_desired_.z();
      base_marker = free_gait::RosVisualization::getComMarker(desired_base_pose_.getPosition(), "odom", color, 0.08);
      desired_base_com_marker_pub_.publish(base_marker);


      base_target_msg_.ignore_timing_of_leg_motion = false;
      base_target_msg_.average_linear_velocity = desired_linear_velocity_.norm();
      base_target_msg_.average_angular_velocity = desired_angular_velocity_.norm();
      base_target_msg_.target.header.frame_id = "odom";
      kindr_ros::convertToRosGeometryMsg(P_CoM_desired_, base_target_msg_.target.pose.position);
//      kindr_ros::convertToRosGeometryMsg(robot_state_.getOrientationBaseToWorld(), base_target_msg_.target.pose.orientation);
      double yaw_angle;
      EulerAnglesZyx ypr(robot_state_.getOrientationBaseToWorld());
      yaw_angle = ypr.setUnique().vector()(0);

      EulerAnglesZyx ypr_optimized(optimized_base_pose.getRotation());
      ypr_optimized.setUnique();
      double pitch_angle = ypr_optimized.pitch();
      yaw_angle = ypr_optimized.yaw();
      desired_base_pose_.getRotation() = EulerAnglesZyx(yaw_angle, pitch_angle, 0);
      kindr_ros::convertToRosGeometryMsg(RotationQuaternion(EulerAnglesZyx(yaw_angle, pitch_angle, 0)), base_target_msg_.target.pose.orientation);

      base_target_msg_.target.header.stamp = ros::Time::now();
      desired_com_path_.poses.push_back(base_target_msg_.target);
      desired_com_path_.header.frame_id = "odom";
      if(desired_com_path_.poses.size()>1000)
        desired_com_path_.poses.clear();
      desired_com_path_pub_.publish(desired_com_path_);
      EulerAnglesZyx eular_zyx(optimized_base_pose.getRotation());
//      EulerAnglesZyx eular_zyx(yaw_angle, pitch_angle, 0);
      eular_zyx.setUnique();
//      std::cout<<"Pose Optiazation Geometric solve result:"<<std::endl<<optimized_base_pose.getPosition()<<std::endl<<
//            "Rotation: "<<std::endl<<"Roll: "<<180.0 / M_PI * eular_zyx.roll()<<std::endl<<"Pitch: "<<
//            180.0 / M_PI * eular_zyx.pitch()<<std::endl<<"Yaw: "<<180.0 / M_PI * eular_zyx.yaw()<<std::endl;

//      kindr_ros::convertToRosGeometryMsg(RotationQuaternion(ypr_optimized), base_target_msg_.target.pose.orientation);
//      kindr_ros::convertToRosGeometryMsg(optimized_base_pose.getRotation(), base_target_msg_.target.pose.orientation);


      /****************
* TODO(Shunyao) : HOW to use base auto, I find that there problem use baseauto because
* of the delaying in contact?
****************/
//      base_auto_msg_.average_linear_velocity = desired_linear_velocity.norm();
//      base_auto_msg_.average_angular_velocity = desired_angular_velocity.norm();
//      base_auto_msg_.height = height_;
//      base_auto_msg_.ignore_timing_of_leg_motion = false;
//      step_msg_.base_auto[0] =base_auto_msg_;

      if(base_target_flag && !crawl_flag)
        step_msg_.base_target[0] = base_target_msg_;

      if(base_auto_flag && !crawl_flag)
        {
          base_auto_msg_.average_linear_velocity = 1;//2*desired_linear_velocity.norm();
          base_auto_msg_.average_angular_velocity = 1;//5*desired_angular_velocity.norm();
          base_auto_msg_.height = height_;
          base_auto_msg_.ignore_timing_of_leg_motion = false;
          base_auto_msg_.support_margin = 0.03;
          step_msg_.base_auto[0] = base_auto_msg_;
        }



//      ROS_INFO("Update base Target Position : ");
//      std::cout<<P_CoM_desired_<<std::endl;
  }

  Pose GaitGenerateClient::getDesiredBasePose()
  {
    return desired_base_pose_;
  }
  int GaitGenerateClient::getGaitType()
  {
    if(trot_flag == true)
      return 1;
    if(pace_flag == true)
      return 2;
    if(crawl_flag == true)
      return 3;
    return 0;
  }

  bool GaitGenerateClient::optimizePose(free_gait::Pose& pose)
  {/*
    Position foot_sum;
    for(auto foot : foothold_in_support_)
      {
        if(robot_state_.isSupportLeg(foot.first))
          foot_sum += foot.second;//robot_state_.getPositionBaseToFootInBaseFrame(foot.first);
      }
       footprint_center_in_world = foot_sum/robot_state_.getNumberOfSupportLegs();*/
      pose.getPosition() = footprint_center_in_world;

      nominalStanceInBaseFrame_.clear();
      for (const auto& stance : nominalPlanarStanceInBaseFrame) {
        nominalStanceInBaseFrame_.emplace(stance.first, Position(stance.second(0), stance.second(1), -height_));
      }
//      for(const auto& stance : nominalStanceInBaseFrame_)
//        {
//          stanceForOrientation_.at(stance.first) = robot_state_.getOrientationBaseToWorld().rotate(nominalStanceInBaseFrame_.at(stance.first));
//        }

      /*Eigen::Matrix4d C(Eigen::Matrix4d::Zero()); // See (45).
      Eigen::Matrix4d A(Eigen::Matrix4d::Zero()); // See (46).
      for (const auto& foot : foothold_in_support_) {
        kindr::QuaternionD footPositionInertialFrame(0.0, foot.second.vector()); // \bar_a_k = S^T * a_k (39).
        kindr::QuaternionD defaultFootPositionBaseFrame(0.0, nominalStanceInBaseFrame_.at(foot.first).vector()); // \bar_b_k = S^T * b_k.
        Eigen::Matrix4d Ak = footPositionInertialFrame.getQuaternionMatrix() - defaultFootPositionBaseFrame.getConjugateQuaternionMatrix(); // See (46).
        C += Ak * Ak; // See (45).
        A += Ak; // See (46).
      }
      A = A / ((double) foothold_in_support_.size()); // Error in (46).
      C -= foothold_in_support_.size() * A * A; // See (45).
      Eigen::EigenSolver<Eigen::Matrix4d> eigenSolver(C);
      int maxCoeff;
      eigenSolver.eigenvalues().real().maxCoeff(&maxCoeff);
      // Eigen vector corresponding to max. eigen value.
      pose.getRotation() = RotationQuaternion(eigenSolver.eigenvectors().col(maxCoeff).real());
      pose.getRotation().setUnique();*/
      stanceForOrientation_ = foothold_in_support_;
      const Position positionForeFeetMidPointInWorld = (stanceForOrientation_.at(LimbEnum::LF_LEG) + stanceForOrientation_.at(LimbEnum::RF_LEG)) * 0.5;
      const Position positionHindFeetMidPointInWorld = (stanceForOrientation_.at(LimbEnum::LH_LEG) + stanceForOrientation_.at(LimbEnum::RH_LEG)) * 0.5;
      Vector desiredHeadingDirectionInWorld = Vector(positionForeFeetMidPointInWorld - positionHindFeetMidPointInWorld);
//      desiredHeadingDirectionInWorld.z() = 0.0;
      RotationQuaternion desiredHeading;
      desiredHeading.setFromVectors(Vector::UnitX().toImplementation(), desiredHeadingDirectionInWorld.vector());

//      const RotationQuaternion yawRotation(RotationVector(RotationVector(pose.getRotation()).vector().cwiseProduct(Eigen::Vector3d::UnitZ())));
//      const RotationQuaternion rollPitchRotation(RotationVector(1 * RotationVector(yawRotation.inverted() * pose.getRotation()).vector()));
//    //  pose.getRotation() = yawRotation * rollPitchRotation; // Alternative.
//      pose.getRotation() = desiredHeading * rollPitchRotation;
      pose.getRotation() = desiredHeading;
      pose.getRotation().setUnique();

//      ROS_WARN_STREAM("Stance to Reach :"<<stanceForOrientation_<<std::endl);
//      ROS_WARN_STREAM("Stance in support :"<<foothold_in_support_<<std::endl);
//      ROS_WARN_STREAM("Nominal stance :"<<nominalStanceInBaseFrame_<<std::endl);

//      EulerAnglesZyx eular_zyx(pose.getRotation());
//      eular_zyx.setUnique();
//      std::cout<<"Pose Optiazation Geometric solve result:"<<std::endl<<pose.getPosition()<<std::endl<<
//            "Rotation: "<<std::endl<<"Roll: "<<180.0 / M_PI * eular_zyx.roll()<<std::endl<<"Pitch: "<<
//            180.0 / M_PI * eular_zyx.pitch()<<std::endl<<"Yaw: "<<180.0 / M_PI * eular_zyx.yaw()<<std::endl;
      return true;

  }

  bool GaitGenerateClient::sendMotionGoal()
  {
    free_gait_msgs::ExecuteStepsGoal steps_goal;
    /****************
* TODO(Shunyao) : decide if the step is empty to send
****************/
    if(step_msg_.footstep.size()>0 && !crawl_flag){
        steps_goal.steps.push_back(step_msg_);
        steps_goal.preempt = steps_goal.PREEMPT_NO;
        sendGoal(steps_goal);
        ROS_DEBUG("Send Step Goal Once!!!!!!!!!!");
        step_msg_.footstep.clear();
        current_velocity_buffer_.clear();
      }
//    if(pace_flag)
//      {
//        free_gait_msgs::Step preStep;
//        free_gait_msgs::BaseTarget baseMotion;
//        baseMotion.average_linear_velocity = 0.5;
//        baseMotion.average_angular_velocity = 0.5;
//        baseMotion.

//      }
    if(crawl_flag && crawl_leg_switched)
      {
          crawl_leg_switched = false;

            free_gait_msgs::Step preStep;
            free_gait_msgs::BaseAuto baseMotion;
            baseMotion.height = height_;
            baseMotion.average_linear_velocity = 0.1;//2*desired_linear_velocity.norm();
            baseMotion.average_angular_velocity = 0.15;//5*desired_angular_velocity.norm();
            baseMotion.ignore_timing_of_leg_motion = true;
            baseMotion.support_margin = crawl_support_margin;

            preStep.base_auto.push_back(baseMotion);
            steps_goal.steps.push_back(preStep);

//            baseMotion.ignore_timing_of_leg_motion = false;
//            step_msg_.base_auto.push_back(baseMotion);
            steps_goal.steps.push_back(step_msg_);
            steps_goal.preempt = steps_goal.PREEMPT_NO;
            sendGoal(steps_goal);
            ROS_DEBUG("Send Step Goal Once!!!!!!!!!!");
            step_msg_.footstep.clear();
            step_msg_.base_auto.clear();

//        action_client_ptr->waitForResult(3);

      }



//    ROS_WARN("Foot Step SIZE is : %f",step_msg_.footstep.size());
//    step_msg_.footstep.resize(4);
//    step_msg_.base_target.clear();
  }


  bool GaitGenerateClient::advance(double dt)
  {
    //! WSHY: this to generate periodical swing and stance situation for each leg
    //! TODO : Test it via a topic?
    if(crawl_flag)
      {
          if(is_done)
          {
            ROS_INFO("Switch to Next Step");
            is_done = false;
            LimbEnum current_swing_leg, next_swing_leg;
            //! WSHY: switch leg
            for(int i = 0; i<4; i++)
              {
                LimbEnum limb = static_cast<LimbEnum>(i);
                if(limb_phase.at(limb).swing_status == true)
                  {
                    current_swing_leg = limb;
                    limb_phase.at(limb).swing_status = false;
                    limb_phase.at(limb).stance_status = true;
                  }
              }
            int next_index = 0;
            int current_leg = static_cast<int>(current_swing_leg);
            for(int i = 0; i<4; i++)
              {
                if(crawl_leg_order[i] == current_leg)
                  {
                    next_index = i+1;
                    if(next_index>3)
                      next_index = next_index - 4;
                  }
              }
            next_swing_leg = static_cast<LimbEnum>(crawl_leg_order[next_index]);
            limb_phase.at(next_swing_leg).ready_to_swing = true;
            limb_phase.at(next_swing_leg).swing_status = true;
            limb_phase.at(next_swing_leg).stance_status = false;
            crawl_leg_switched = true;
//            pre_step = true;
          }
          for(int i = 0;i<4;i++)
            {
              LimbEnum limb = static_cast<LimbEnum>(i);
              robot_state_.setSupportLeg(limb, limb_phase.at(limb).stance_status);
            }
//          if(is_done && !crawl_leg_switched)
//            {
//              pre_step = true;
//            }

          }else{

        int number_of_swing_contacted = 0;
        bool is_contact = false;
        for(int i = 0;i<4;i++)
          {
            LimbEnum limb = static_cast<LimbEnum>(i);
            if(limb_phase.at(limb).swing_status && limb_phase.at(limb).swing_phase>t_swing_/2 && limb_phase.at(limb).real_contact)
              {
                number_of_swing_contacted++;
              }
          }
        if(trot_flag && number_of_swing_contacted == 2)
          is_contact = true;
        if(pace_flag && number_of_swing_contacted == 1)
          is_contact = true;

        is_contact = true;
        for(int i =0;i<4;i++)
          {
            LimbEnum limb = static_cast<LimbEnum>(i);

            if(limb_phase.at(limb).swing_status){
              limb_phase.at(limb).swing_phase = limb_phase.at(limb).swing_phase + dt;
    //          ROS_INFO("Leg %s is in swing phase : %f/%f\n", getLimbStringFromLimbEnum(limb).c_str(),
    //                   limb_phase.at(limb).swing_phase, t_swing_);
              if(limb_phase.at(limb).swing_phase>t_swing_ && is_contact)
                {
                  //! WSHY: normal contacted
                  limb_phase.at(limb).swing_phase = 0;
                  limb_phase.at(limb).swing_status = false;
                  limb_phase.at(limb).stance_status = true;
                }/* else if (limb_phase.at(limb).swing_phase>t_swing_ && !is_contact) {
                  //! WSHY: late contacted
                  limb_phase.at(limb).swing_status = true;
                  limb_phase.at(limb).stance_status = false;
                } else if (limb_phase.at(limb).swing_phase>0.5*t_swing_ &&
                           limb_phase.at(limb).swing_phase<t_swing_ && is_contact) {
                  //! WSHY: early contacted
    //              for(int i=0;i<4;i++)
    //              {
    //                free_gait::LimbEnum stance_limb = static_cast<free_gait::LimbEnum>(i);
    //                if(limb_phase.at(stance_limb).stance_status)
    //                  limb_phase.at(stance_limb).stance_phase = t_swing_ - limb_phase.at(limb).swing_phase;
    //              }
                  limb_phase.at(limb).swing_phase = 0;
                  limb_phase.at(limb).swing_status = false;
                  limb_phase.at(limb).stance_status = true;


                }*/
              } else if (limb_phase.at(limb).stance_status) {
                limb_phase.at(limb).stance_phase = limb_phase.at(limb).stance_phase + dt;
      //          ROS_INFO("Leg %s is in stance phase : %f/%f\n", getLimbStringFromLimbEnum(limb).c_str(),
      //                   limb_phase.at(limb).stance_phase, t_stance_);
                if(limb_phase.at(limb).stance_phase>t_stance_ && is_contact)
                  {
                    if(pace_flag && limb_phase.at(limb).stance_phase<(t_stance_+t_swing_delay))
                      {
                        //! WSHY: delay for really to swing
                        limb_phase.at(limb).swing_phase = limb_phase.at(limb).swing_phase + dt;
                      }else{
                        limb_phase.at(limb).stance_phase = 0;
                        limb_phase.at(limb).stance_status = false;
                        limb_phase.at(limb).swing_status = true;
                        limb_phase.at(limb).ready_to_swing = true;
                      }


                  }

            };
          robot_state_.setSupportLeg(limb, limb_phase.at(limb).stance_status);
          }

      }

  }

  std::string GaitGenerateClient::getLimbStringFromLimbEnum(const LimbEnum& limb) const
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

