/*
 *  gait_generate_client.cpp
 *  Descriotion:
 *
 *  Created on: Mar 26, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#include "free_gait_ros/gait_generate_client.hpp"

using namespace free_gait;

  GaitGenerateClient::GaitGenerateClient(const ros::NodeHandle& node_handle)//, FreeGaitActionClient& actionClient)
    : nodeHandle_(node_handle),
      is_done(false),
      is_active(false),
      is_updated(false)
//      actionClient_(nodeHandle_)
  {
    initialize();
    velocity_command_sub_ = nodeHandle_.subscribe("/cmd_vel", 1, &GaitGenerateClient::velocityCommandCallback, this);
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
    height = 0.4;
    step_number = 0;
    sigma_st_0 = 0.5;
    sigma_st_1 = 0.5;
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

  }

  bool GaitGenerateClient::initializeTrot(const double t_swing, const double t_stance)
  {
    t_swing_ = t_swing;
    t_stance_ = t_stance;
    step_msg_.base_target.resize(1);
//    step_msg_.footstep.resize(4);
    limb_phase[LimbEnum::LF_LEG].swing_phase = 0;
    limb_phase[LimbEnum::LF_LEG].stance_phase = 0;
    limb_phase[LimbEnum::LF_LEG].swing_status = true;
    limb_phase[LimbEnum::LF_LEG].stance_status = false;
    limb_phase[LimbEnum::LF_LEG].ready_to_swing = true;

    limb_phase[LimbEnum::RF_LEG].swing_phase = 0;
    limb_phase[LimbEnum::RF_LEG].stance_phase = 0;
    limb_phase[LimbEnum::RF_LEG].swing_status = false;
    limb_phase[LimbEnum::RF_LEG].stance_status = true;
    limb_phase[LimbEnum::RF_LEG].ready_to_swing = false;

    limb_phase[LimbEnum::RH_LEG].swing_phase = 0;
    limb_phase[LimbEnum::RH_LEG].stance_phase = 0;
    limb_phase[LimbEnum::RH_LEG].swing_status = true;
    limb_phase[LimbEnum::RH_LEG].stance_status = false;
    limb_phase[LimbEnum::RH_LEG].ready_to_swing = true;

    limb_phase[LimbEnum::LH_LEG].swing_phase = 0;
    limb_phase[LimbEnum::LH_LEG].stance_phase = 0;
    limb_phase[LimbEnum::LH_LEG].swing_status = false;
    limb_phase[LimbEnum::LH_LEG].stance_status = true;
    limb_phase[LimbEnum::LH_LEG].ready_to_swing = false;

  }

  bool GaitGenerateClient::initializePace(const double t_swing, const double t_stance)
  {

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
    ROS_INFO("Goal just went active");
  }

  void GaitGenerateClient::feedbackCallback(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback)
  {
    ROS_INFO("step time is : %f", feedback->phase);
  }

  void GaitGenerateClient::doneCallback(const actionlib::SimpleClientGoalState& state,
                    const free_gait_msgs::ExecuteStepsResult& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
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

  bool GaitGenerateClient::copyRobotState(const free_gait::State& state)
  {
    robot_state_ = state;
    desired_linear_velocity_world_ = robot_state_.getOrientationBaseToWorld().rotate(desired_linear_velocity_);
    return true;
  }

  bool GaitGenerateClient::generateFootHolds()
  {
    ROS_INFO("In Generate FootHolds");
    double t_stance = 0.4;
    double height = robot_state_.getPositionWorldToBaseInWorldFrame()(2);
    LinearVelocity current_vel_in_base = robot_state_.getOrientationBaseToWorld().inverseRotate(robot_state_.getLinearVelocityBaseInWorldFrame());
    for(int i = 0;i<4;i++)
      {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);

        if(!robot_state_.isSupportLeg(limb) && limb_phase.at(limb).ready_to_swing)
          {
            limb_phase.at(limb).ready_to_swing = false;
            Position displace_in_footprint = Position(0.5*t_stance*desired_linear_velocity_
                                                 + sqrt(height/9.8) * (current_vel_in_base - desired_linear_velocity_));
            Position displace_in_odom = robot_state_.getOrientationBaseToWorld().rotate(displace_in_footprint);
            displace_in_odom(2) = 0.0;
//            Position target_in_base = Position(robot_state_.getPositionBaseToHipInBaseFrame(limb).toImplementation()
//                                       + 0.5*t_stance*desired_linear_velocity_.toImplementation()
//                                       + sqrt(height/9.8) * (current_vel - desired_linear_velocity_).toImplementation());
            Position hip_in_odom = robot_state_.getPositionWorldToBaseInWorldFrame() +
                robot_state_.getOrientationBaseToWorld().rotate(robot_state_.getPositionBaseToHipInBaseFrame(limb));

//            target_in_base(2) = 0.0;
            hip_in_odom(2) = 0.0; //project to floor plane
            Position target_in_odom = hip_in_odom + displace_in_odom;
            footstep_msg_.name = getLimbStringFromLimbEnum(limb);
            kindr_ros::convertToRosGeometryMsg(target_in_odom,
                                               footstep_msg_.target.point);
//            footstep_msg_.target.point.z = 0;
            footstep_msg_.target.header.frame_id = "odom";
            footstep_msg_.average_velocity = sqrt(displace_in_odom.norm()*displace_in_odom.norm() + 0.1*0.1)*2/0.1;
            footstep_msg_.profile_height = 0.1;
            footstep_msg_.ignore_contact = false;
            footstep_msg_.ignore_for_pose_adaptation = false;
            kindr_ros::convertToRosGeometryMsg(robot_state_.getSurfaceNormal(limb),
                                               footstep_msg_.surface_normal.vector);

//            step_msg_.footstep[i] = footstep_msg_;
            step_msg_.footstep.push_back(footstep_msg_);
            ROS_WARN("Generate foot hold position(%f, %f, %f) for %s ",footstep_msg_.target.point.x,footstep_msg_.target.point.y,footstep_msg_.target.point.z,
                     getLimbStringFromLimbEnum(limb).c_str());
          }
      }

  }

  bool GaitGenerateClient::updateBaseMotion(LinearVelocity& desired_linear_velocity,
                                            LocalAngularVelocity& desired_angular_velocity)
  {
    ROS_INFO("In update Base Motion");
//    ROS_WARN_STREAM("Desired Velocity :"<<desired_linear_velocity_<<std::endl);
    desired_linear_velocity = desired_linear_velocity_world_;
    desired_angular_velocity = desired_angular_velocity_;
    P_CoM_desired_.setZero();
    for(int i = 0; i<4 ;i++)
      {
        LimbEnum limb = static_cast<LimbEnum>(i);
        double total_phase = 0;
        if(limb_phase.at(limb).swing_status)
          total_phase = limb_phase.at(limb).swing_phase/(t_stance_ + t_swing_);
        if(limb_phase.at(limb).stance_status)
          total_phase = (limb_phase.at(limb).stance_phase + t_swing_)/(t_stance_ + t_swing_);
        double k_st = 0.5*(erf(total_phase/(sigma_st_0*sqrt(2))) +
                           erf((1 - total_phase)/(sigma_st_1*sqrt(2))));
        double k_sw = 0.5*(2 + erf(-total_phase/(sigma_sw_0*sqrt(2))) +
                           erf((total_phase - 1)/(sigma_sw_1*sqrt(2))));
        limb_phase.at(limb).weight_to_CoM = limb_phase.at(limb).stance_status * k_st
                                            +limb_phase.at(limb).swing_status * k_sw;

//      ROS_INFO("Calculate weight for %s\n",getLimbStringFromLimbEnum(limb).c_str());
      }
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
      }
      P_CoM_desired_ = 0.25 * P_CoM_desired_;
      P_CoM_desired_(2) = 0.4;//robot_state_.getPositionWorldToBaseInWorldFrame()(2);
      base_target_msg_.ignore_timing_of_leg_motion = false;
      base_target_msg_.average_linear_velocity = desired_linear_velocity_.norm();
      base_target_msg_.average_angular_velocity = desired_angular_velocity_.norm();
      base_target_msg_.target.header.frame_id = "odom";
      kindr_ros::convertToRosGeometryMsg(P_CoM_desired_, base_target_msg_.target.pose.position);
//      kindr_ros::convertToRosGeometryMsg(robot_state_.getOrientationBaseToWorld(), base_target_msg_.target.pose.orientation);
      double yaw_angle;
      EulerAnglesZyx ypr(robot_state_.getOrientationBaseToWorld());
      yaw_angle = ypr.setUnique().vector()(0);
      kindr_ros::convertToRosGeometryMsg(RotationQuaternion(EulerAnglesZyx(yaw_angle, 0, 0)), base_target_msg_.target.pose.orientation);

      step_msg_.base_target[0] = base_target_msg_;
      ROS_INFO("Update base Target Position : ");
      std::cout<<P_CoM_desired_<<std::endl;
  }

  bool GaitGenerateClient::sendMotionGoal()
  {
    free_gait_msgs::ExecuteStepsGoal steps_goal;
    /****************
* TODO(Shunyao) : decide if the step is empty to send
****************/
    if(step_msg_.footstep.size()>0){
        steps_goal.steps.push_back(step_msg_);
        sendGoal(steps_goal);
        ROS_INFO("Send Step Goal Once!!!!!!!!!!");
        step_msg_.footstep.clear();
      }



//    ROS_WARN("Foot Step SIZE is : %f",step_msg_.footstep.size());
//    step_msg_.footstep.resize(4);
//    step_msg_.base_target.clear();
  }

  bool GaitGenerateClient::advance(double dt)
  {
    //! WSHY: this to generate periodical swing and stance situation for each leg
    //! TODO : Test it via a topic?
    for(int i =0;i<4;i++)
      {

        LimbEnum limb = static_cast<LimbEnum>(i);
        bool is_contact = robot_state_.isSupportLeg(limb);
        if(limb_phase.at(limb).swing_status){
          limb_phase.at(limb).swing_phase = limb_phase.at(limb).swing_phase + dt;
//          ROS_INFO("Leg %s is in swing phase : %f/%f\n", getLimbStringFromLimbEnum(limb).c_str(),
//                   limb_phase.at(limb).swing_phase, t_swing_);
          if(limb_phase.at(limb).swing_phase>t_swing_)// && is_contact)
            {
              //! WSHY: normal contacted
              limb_phase.at(limb).swing_phase = 0;
              limb_phase.at(limb).swing_status = false;
              limb_phase.at(limb).stance_status = true;
            } /*else if (limb_phase.at(limb).swing_phase>t_swing_ && !is_contact) {
              //! WSHY: late contacted
              limb_phase.at(limb).swing_status = true;
            } else if (limb_phase.at(limb).swing_phase<t_swing_ && is_contact) {
              //! WSHY: early contacted
              limb_phase.at(limb).swing_phase = 0;
              limb_phase.at(limb).swing_status = false;
              limb_phase.at(limb).stance_status = true;
            }*/
          } else if (limb_phase.at(limb).stance_status) {
          limb_phase.at(limb).stance_phase = limb_phase.at(limb).stance_phase + dt;
//          ROS_INFO("Leg %s is in stance phase : %f/%f\n", getLimbStringFromLimbEnum(limb).c_str(),
//                   limb_phase.at(limb).stance_phase, t_stance_);
          if(limb_phase.at(limb).stance_phase>t_stance_)
            {
              limb_phase.at(limb).stance_phase = 0;
              limb_phase.at(limb).stance_status = false;
              limb_phase.at(limb).swing_status = true;
              limb_phase.at(limb).ready_to_swing = true;
            }

        };
      robot_state_.setSupportLeg(limb, limb_phase.at(limb).stance_status);
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

