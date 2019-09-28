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
    footstepOptimization.reset(new FootstepOptimization(nodeHandle_));

    velocity_command_sub_ = nodeHandle_.subscribe("/cmd_vel", 1, &GaitGenerateClient::velocityCommandCallback, this);
    foot_marker_pub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("desired_footholds", 1);

    action_client_ptr.reset(new free_gait::FreeGaitActionClient(nodeHandle_));//FreeGaitActionClient

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

  }

  bool GaitGenerateClient::initializeTrot(const double t_swing, const double t_stance)
  {
    t_swing_ = t_swing;
    t_stance_ = t_stance;
    step_msg_.base_target.resize(1);
//    step_msg_.base_auto.resize(1);
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
    base_auto_flag = false;
    base_target_flag = true;
    pace_flag =false;
    trot_flag = true;
    return true;

  }

  bool GaitGenerateClient::initializePace(const double t_swing, const double t_stance)
  {
    t_swing_ = t_swing;
    t_stance_ = t_stance;
//    step_msg_.base_auto.resize(1);
    step_msg_.base_target.resize(1);
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
    limb_phase[LimbEnum::RH_LEG].stance_phase = 2*t_stance_/3;
    limb_phase[LimbEnum::RH_LEG].swing_status = false;
    limb_phase[LimbEnum::RH_LEG].stance_status = true;
    limb_phase[LimbEnum::RH_LEG].ready_to_swing = false;

    limb_phase[LimbEnum::LH_LEG].swing_phase = 0;
    limb_phase[LimbEnum::LH_LEG].stance_phase = t_stance_/3;
    limb_phase[LimbEnum::LH_LEG].swing_status = false;
    limb_phase[LimbEnum::LH_LEG].stance_status = true;
    limb_phase[LimbEnum::LH_LEG].ready_to_swing = false;
    base_auto_flag = false;
    base_target_flag = true;
    pace_flag =true;
    trot_flag = false;
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
    robot_state_ = state;//state = /gazebo/robot_state

    desired_linear_velocity_world_ = robot_state_.getOrientationBaseToWorld().rotate(desired_linear_velocity_);
    Position foot_sum;
    for(int i = 0;i<4;i++)
      {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        if(robot_state_.isSupportLeg(limb))
          {
            foothold_in_support_[limb] = robot_state_.getPositionWorldToFootInWorldFrame(limb);
            stanceForOrientation_[limb] = foothold_in_support_[limb];
            foot_sum += foothold_in_support_[limb];//get the sum of all support legs
          }
      }
    if(robot_state_.getNumberOfSupportLegs()>0)
      footprint_center_in_world = foot_sum/robot_state_.getNumberOfSupportLegs();
    return true;
  }

  bool GaitGenerateClient::generateFootHolds(const std::string frame)
  {
//    ROS_INFO("In Generate FootHolds");
    footholds_in_stance.setZero();
//    double t_stance = 0.4;
    double height = robot_state_.getPositionWorldToBaseInWorldFrame()(2);
    LinearVelocity current_vel_in_base = robot_state_.getOrientationBaseToWorld().inverseRotate(robot_state_.getLinearVelocityBaseInWorldFrame());
    Position displace_in_footprint;
    Stance stance_to_reach;
    visualization_msgs::MarkerArray foot_markers;
    for(int i = 0;i<4;i++)
      {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);

        if(!robot_state_.isSupportLeg(limb) && limb_phase.at(limb).ready_to_swing)
          {
            limb_phase.at(limb).ready_to_swing = false;
            LinearVelocity desired_vel2D, current_vel2D;
            desired_vel2D = desired_linear_velocity_;
            desired_vel2D(2) = 0.0;
            current_vel2D = current_vel_in_base;
            current_vel2D(2) = 0.0;
            double z_hip = height - footprint_center_in_world(2);
//            if(pace_flag){
//                displace_in_footprint = Position(0.33*t_stance_*desired_vel2D
//                                                   + sqrt(z_hip/9.8) * (current_vel2D - desired_vel2D));
//              }
            displace_in_footprint = Position(0.5*t_stance_*desired_vel2D
                                                 + sqrt(z_hip/9.8) * (current_vel2D - desired_vel2D));
            displace_in_footprint(2) = 0.02;
            Position displace_in_baselink = displace_in_footprint;
//            displace_in_baselink(2) = -height_+ 0.025;
            double yaw_angle;
            EulerAnglesZyx ypr(robot_state_.getOrientationBaseToWorld());
            yaw_angle = ypr.setUnique().vector()(0);
            RotationQuaternion orinetationFootprintToWorld = RotationQuaternion(EulerAnglesZyx(yaw_angle,0,0));
            Position displace_in_odom = orinetationFootprintToWorld.rotate(displace_in_footprint + Position(t_stance_*desired_vel2D));
            displace_in_odom(2) = 0.02;
//            Position target_in_base = Position(robot_state_.getPositionBaseToHipInBaseFrame(limb).toImplementation()
//                                       + 0.5*t_stance*desired_linear_velocity_.toImplementation()
//                                       + sqrt(height/9.8) * (current_vel - desired_linear_velocity_).toImplementation());
            Position hip_in_odom = robot_state_.getPositionWorldToBaseInWorldFrame() +
                robot_state_.getOrientationBaseToWorld().rotate(robot_state_.getPositionBaseToHipInBaseFrame(limb));
            hip_in_odom(2) = 0.0; //project to floor plane

            Position hip_in_footprint = robot_state_.getPositionBaseToHipInBaseFrame(limb);
            Position hip_in_baselink = robot_state_.getPositionBaseToHipInBaseFrame(limb)
                + robot_state_.getOrientationBaseToWorld().inverseRotate(Position(0,0,-height_-0.03));

            Position target_in_footprint = hip_in_footprint + displace_in_footprint;
            Position target_in_baselink = displace_in_baselink + hip_in_baselink;
            //            target_in_footprint(2) = 0.0;
            Position target_in_odom = hip_in_odom + displace_in_odom;
            Position target_in_base =robot_state_.getOrientationBaseToWorld().inverseRotate(target_in_odom - robot_state_.getPositionWorldToBaseInWorldFrame());

            RotationQuaternion orinetationFootprintToBase = robot_state_.getOrientationBaseToWorld().inverted() * orinetationFootprintToWorld;
            target_in_footprint = Position(0,0,height) + orinetationFootprintToBase.inverseRotate(target_in_baselink);
//            target_in_odom = robot_state_.getPositionWorldToBaseInWorldFrame()
//                + robot_state_.getOrientationBaseToWorld().rotate(target_in_baselink);

            target_in_odom = robot_state_.getPositionWorldToBaseInWorldFrame() + Position(0,0,-height) +
                orinetationFootprintToWorld.rotate(target_in_footprint + Position(t_stance_*desired_vel2D));

            footstep_msg_.name = getLimbStringFromLimbEnum(limb);
            Position target_optimized = target_in_odom;
            if(frame=="odom")
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
                ROS_WARN_STREAM("target before optimized :"<<target_in_footprint<<std::endl);
                target_optimized = robot_state_.getPositionWorldToBaseInWorldFrame() + Position(0,0,-height) +
                    orinetationFootprintToWorld.rotate(target_in_footprint + Position(t_stance_*current_vel2D));
                stance_to_reach[limb] = target_optimized;
                if(!footstepOptimization->getOptimizedFoothold(target_optimized,
                                                           robot_state_,
                                                           limb, "odom"))
                  {
                    ROS_ERROR("Failed to Find a optimized foothold");
                    kindr_ros::convertToRosGeometryMsg(target_in_footprint,
                                                       footstep_msg_.target.point);
                    kindr_ros::convertToRosGeometryMsg(Vector(0,0,1), footstep_msg_.surface_normal.vector);

                  }else{
                    target_in_footprint = orinetationFootprintToWorld.inverseRotate(target_optimized
                        - robot_state_.getPositionWorldToBaseInWorldFrame() - Position(0,0,-height))- Position(t_stance_*current_vel2D);
                    target_in_footprint(2) = target_in_footprint(2) - 0.05;
                    kindr_ros::convertToRosGeometryMsg(target_in_footprint,
                                                       footstep_msg_.target.point);
                    kindr_ros::convertToRosGeometryMsg(footstepOptimization->getSurfaceNormal(target_optimized),
                                                       footstep_msg_.surface_normal.vector);

                   footstep_msg_.profile_type = "square";
                  }


                kindr_ros::convertToRosGeometryMsg(target_in_footprint,
                                                   footstep_msg_.target.point);
    //            footstep_msg_.target.point.z = 0;
                footstep_msg_.target.header.frame_id = "foot_print";

//                stance_to_reach[limb] = target_optimized;
                std_msgs::ColorRGBA color;
                color.a = 1;
                color.g = 1;
                foot_markers = free_gait::RosVisualization::getFootholdsMarker(stance_to_reach, "odom", color, 0.08);
                ROS_WARN_STREAM("target after optimized :"<<target_in_footprint<<std::endl);

              }

            footstep_msg_.average_velocity = sqrt(displace_in_odom.norm()*displace_in_odom.norm() + 0.15*0.15)*2/0.1;
            footstep_msg_.profile_height = 0.15;
            footstep_msg_.ignore_contact = false;
            footstep_msg_.ignore_for_pose_adaptation = false;
            kindr_ros::convertToRosGeometryMsg(robot_state_.getSurfaceNormal(limb),
                                               footstep_msg_.surface_normal.vector);

//            step_msg_.footstep[i] = footstep_msg_;
            step_msg_.footstep.push_back(footstep_msg_);
//            ROS_WARN("Generate foot hold position(%f, %f, %f) for %s ",footstep_msg_.target.point.x,footstep_msg_.target.point.y,footstep_msg_.target.point.z,
//                     getLimbStringFromLimbEnum(limb).c_str());
            stanceForOrientation_[limb] = robot_state_.getOrientationBaseToWorld().rotate(target_in_base) + robot_state_.getPositionWorldToBaseInWorldFrame();
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
    desired_linear_velocity = desired_linear_velocity_world_;
    desired_angular_velocity = desired_angular_velocity_;
    P_CoM_desired_.setZero();
    Position foot_sum;
    for(int i = 0; i<4 ;i++)
      {
        /****************
* TODO(Shunyao) : Adjust the sigma to change the move of CoM
****************/
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
      Pose optimized_base_pose;
      optimizePose(optimized_base_pose);

      P_CoM_desired_ = 0.25 * P_CoM_desired_;

//      P_CoM_desired_(2) = height_ + footprint_center_in_world(2) - 0.02;//robot_state_.getPositionWorldToBaseInWorldFrame()(2);
      P_CoM_desired_(2) = height_ + optimized_base_pose.getPosition()(2) - 0.02;


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
      kindr_ros::convertToRosGeometryMsg(RotationQuaternion(EulerAnglesZyx(yaw_angle, pitch_angle, 0)), base_target_msg_.target.pose.orientation);

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

      if(base_target_flag)
        step_msg_.base_target[0] = base_target_msg_;

      if(base_auto_flag)
        {
          base_auto_msg_.average_linear_velocity = 2*desired_linear_velocity.norm();
          base_auto_msg_.average_angular_velocity = 5*desired_angular_velocity.norm();
          base_auto_msg_.height = height_;
          base_auto_msg_.ignore_timing_of_leg_motion = false;
          base_auto_msg_.support_margin = 0.03;
          step_msg_.base_auto[0] = base_auto_msg_;
        }



//      ROS_INFO("Update base Target Position : ");
//      std::cout<<P_CoM_desired_<<std::endl;
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
          if(limb_phase.at(limb).swing_phase>t_swing_ )//&& is_contact)
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

