/*
 *  single_leg_controller.cpp
 *  Descriotion:
 *
 *  Created on: Jul, 7 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "balance_controller/ros_controler/single_leg_controller.hpp"

namespace balance_controller {

  SingleLegController::SingleLegController()
  {
    //! WSHY: initialize STAte
    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
    limbs_.push_back(free_gait::LimbEnum::RH_LEG);

    branches_.push_back(free_gait::BranchEnum::BASE);
    branches_.push_back(free_gait::BranchEnum::LF_LEG);
    branches_.push_back(free_gait::BranchEnum::RF_LEG);
    branches_.push_back(free_gait::BranchEnum::LH_LEG);
    branches_.push_back(free_gait::BranchEnum::RH_LEG);

    robot_state_.reset(new free_gait::State);
    robot_state_->initialize(limbs_, branches_);

    for(auto limb : limbs_)
      {
        real_contact_[limb] = false;
        is_cartisian_motion_[limb] = true;
        foot_positions[limb] = Vector(0.4,0.2,-0.3);
        foot_velocities[limb] = Vector(0,0,0);
      }

  };
  SingleLegController::~SingleLegController()
  {};

  bool SingleLegController::init(hardware_interface::EffortJointInterface* hardware,
            ros::NodeHandle& node_handle)
  {
    ROS_INFO("Initializing SingleLegController");
    //! WSHY: single leg controller
    single_leg_solver_.reset(new MyRobotSolver(node_handle, robot_state_));
    single_leg_solver_->model_initialization();
    if(!single_leg_solver_->loadLimbModelFromURDF())
      {
        ROS_ERROR("Failed to load model from URDF file");
      }

    if(!single_leg_solver_->loadParameters())
      {
        ROS_INFO("SWC load parameters failed");
      }

    urdf::Model urdf;
    if (!urdf.initParam("/robot_description"))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    //! WSHY: get joint handle from robot state handle
    std::string param_name = "legs";
    if(!node_handle.getParam(param_name, leg_names))
      {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << node_handle.getNamespace() << ").");
        return false;
      }
    for(auto leg : leg_names)
      {
        if(leg=="LF_LEG")
          leg_to_move.push_back(free_gait::LimbEnum::LF_LEG);
        if(leg=="RF_LEG")
          leg_to_move.push_back(free_gait::LimbEnum::RF_LEG);
        if(leg=="RH_LEG")
          leg_to_move.push_back(free_gait::LimbEnum::RH_LEG);
        if(leg=="LH_LEG")
          leg_to_move.push_back(free_gait::LimbEnum::LH_LEG);
      }

    param_name = "control_methods";
    if(!node_handle.getParam(param_name, control_methods))
      {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << node_handle.getNamespace() << ").");
        return false;
      }
    //! WSHY: get joint handle from robot state handle
    param_name = "joints";
    if(!node_handle.getParam(param_name, joint_names))
      {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << node_handle.getNamespace() << ").");
        return false;
      }
    n_joints = joint_names.size();
    if(n_joints == 0){
          ROS_ERROR_STREAM("List of joint names is empty.");
          return false;
        }

    for(unsigned int i = 0; i < n_joints; i++)
      {
        try {
          joints.push_back(hardware->getHandle(joint_names[i]));
          ROS_INFO("Get '%s' Handle", joint_names[i].c_str());
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
          ROS_ERROR_STREAM("Exception thrown : "<< ex.what());
          return false;
        }
      }

    base_command_sub_ = node_handle.subscribe<free_gait_msgs::RobotState>("/desired_robot_state", 1, &SingleLegController::baseCommandCallback, this);

    contact_sub_ = node_handle.subscribe<sim_assiants::FootContacts>("/bumper_sensor_filter_node/foot_contacts", 1, &SingleLegController::footContactsCallback, this);


  }

  void SingleLegController::update(const ros::Time& time, const ros::Duration& period)
  {
    //! WSHY: update joint state
    free_gait::JointPositions all_joint_positions;
    free_gait::JointVelocities all_joint_velocities;
    free_gait::JointEfforts all_joint_efforts;
    //! WSHY: get joint postions from robot state handle
    for(unsigned int i=0; i<joints.size(); i++)
      {
        all_joint_positions(i) = joints[i].getPosition();
        all_joint_velocities(i) = joints[i].getPosition();
        all_joint_efforts(i) = joints[i].getEffort();
      }

    //! WSHY: update current base state from robot state handle
    robot_state_->setCurrentLimbJoints(all_joint_positions);
    robot_state_->setCurrentLimbJointVelocities(all_joint_velocities);

    ros::Duration real_time_period = ros::Duration(period.toSec());
    for(unsigned int i =0; i<leg_to_move.size(); i++)
      {
//        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        free_gait::LimbEnum limb = leg_to_move[i];
        int j = static_cast<int>(leg_to_move[i]);
        robot_state_->setTargetFootPositionInBaseForLimb(Position(foot_positions.at(limb).vector()), limb);
        robot_state_->setTargetFootVelocityInBaseForLimb(LinearVelocity(foot_velocities.at(limb).vector()), limb);
        single_leg_solver_->setvecQAct(all_joint_positions.vector().segment(3*j, 3), limb);
        single_leg_solver_->setvecQDotAct(all_joint_velocities.vector().segment(3*j, 3), limb);

        if(real_contact_.at(limb))
          robot_state_->setSupportLeg(limb, true);
        else
          robot_state_->setSupportLeg(limb, false);

      }

    free_gait::Force gravity_in_base = free_gait::Force(0,0,-9.8);
    for(unsigned int i =0; i<leg_to_move.size(); i++)
      {
        free_gait::LimbEnum limb = leg_to_move[i];
//        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        single_leg_solver_->update(time, real_time_period, limb);

        if(robot_state_->isSupportLeg(limb))
          {
            if(control_methods[i] == "contact_force")
              {
                Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(limb);
                Force contactForce;
                free_gait::JointEffortsLeg jointTorques = free_gait::JointEffortsLeg(jacobian.transpose() * contactForce.toImplementation());
                free_gait::JointPositionsLeg joint_position_leg = robot_state_->getJointPositionFeedbackForLimb(limb);
                jointTorques += robot_state_->getGravityCompensationForLimb(limb, joint_position_leg, gravity_in_base);
                for(int j = 0;j<3;j++)
                  {
                    joints[j].setCommand(jointTorques(j));
                  }
              }
            if(control_methods[i] == "end_position")
              {
                for(int j = 0;j<3;j++)
                  {
//                    double effort_command;
//                    if(is_cartisian_motion_.at(limb))
//                      effort_command = single_leg_solver_->getVecTauAct()[j];
                    joints[j].setCommand(0.0);
                  }
              }
          }
        else if (!robot_state_->isSupportLeg(limb)) {
            if(control_methods[i] == "end_position")
              {
                for(int j = 0;j<3;j++)
                  {
                    double effort_command;
                    if(is_cartisian_motion_.at(limb))
                      effort_command = single_leg_solver_->getVecTauAct()[j];
                    joints[j].setCommand(effort_command);
                  }
              }
            if(control_methods[i] == "contact_force")
              {
                Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(limb);
                Force contactForce;
                free_gait::JointEffortsLeg jointTorques = free_gait::JointEffortsLeg(jacobian.transpose() * contactForce.toImplementation());
                free_gait::JointPositionsLeg joint_position_leg = robot_state_->getJointPositionFeedbackForLimb(limb);
                jointTorques += robot_state_->getGravityCompensationForLimb(limb, joint_position_leg, gravity_in_base);
                for(int j = 0;j<3;j++)
                  {
                    joints[j].setCommand(jointTorques(j));
                  }
              }
          }
      }

  }

  void SingleLegController::starting(const ros::Time& time)
  {

  }

  void SingleLegController::stopping(const ros::Time& time)
  {

  }

  /**
   * @brief baseCommandCallback, ros subscriber callback
   * @param robot_state
   */
  void SingleLegController::baseCommandCallback(const free_gait_msgs::RobotStateConstPtr& robot_state_msg)
  {

    Position foot_position, foot_velocity, foot_acceleration;

    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lf_target.target_position[0].point,
        foot_position);
    foot_positions[free_gait::LimbEnum::LF_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lf_target.target_velocity[0].vector,
        foot_velocity);
    foot_velocities[free_gait::LimbEnum::LF_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lf_target.target_acceleration[0].vector,
        foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::LF_LEG] = Vector(foot_acceleration.toImplementation());

    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rf_target.target_position[0].point,
        foot_position);
    foot_positions[free_gait::LimbEnum::RF_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rf_target.target_velocity[0].vector,
        foot_velocity);
    foot_velocities[free_gait::LimbEnum::RF_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rf_target.target_acceleration[0].vector,
        foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::RF_LEG] = Vector(foot_acceleration.toImplementation());

    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rh_target.target_position[0].point,
        foot_position);
    foot_positions[free_gait::LimbEnum::RH_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rh_target.target_velocity[0].vector,
        foot_velocity);
    foot_velocities[free_gait::LimbEnum::RH_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->rh_target.target_acceleration[0].vector,
        foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::RH_LEG] = Vector(foot_acceleration.toImplementation());

    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lh_target.target_position[0].point,
        foot_position);
    foot_positions[free_gait::LimbEnum::LH_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lh_target.target_velocity[0].vector,
        foot_velocity);
    foot_velocities[free_gait::LimbEnum::LH_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msg->lh_target.target_acceleration[0].vector,
        foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::LH_LEG] = Vector(foot_acceleration.toImplementation());

    if(robot_state_msg->lf_leg_mode.name == "joint")
      is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
    if(robot_state_msg->lf_leg_mode.name == "cartesian")
      is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = true;

    if(robot_state_msg->rf_leg_mode.name == "joint")
      is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
    if(robot_state_msg->rf_leg_mode.name == "cartesian")
      is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = true;

    if(robot_state_msg->rh_leg_mode.name == "joint")
      is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
    if(robot_state_msg->rh_leg_mode.name == "cartesian")
      is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = true;

    if(robot_state_msg->lh_leg_mode.name == "joint")
      is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
    if(robot_state_msg->lh_leg_mode.name == "cartesian")
      is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = true;

    if(robot_state_msg->lf_leg_mode.support_leg)
        robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, true);
    else
      robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, false);
    if(robot_state_msg->rf_leg_mode.support_leg)
        robot_state_->setSupportLeg(free_gait::LimbEnum::RF_LEG, true);
    else
      robot_state_->setSupportLeg(free_gait::LimbEnum::RF_LEG, false);
    if(robot_state_msg->rh_leg_mode.support_leg)
        robot_state_->setSupportLeg(free_gait::LimbEnum::RH_LEG, true);
    else
      robot_state_->setSupportLeg(free_gait::LimbEnum::RH_LEG, false);
    if(robot_state_msg->lh_leg_mode.support_leg)
        robot_state_->setSupportLeg(free_gait::LimbEnum::LH_LEG, true);
    else
      robot_state_->setSupportLeg(free_gait::LimbEnum::LH_LEG, false);

  }

  void SingleLegController::footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts)
  {
    unsigned int i = 0;

    for(auto contact : foot_contacts->foot_contacts)
      {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        real_contact_.at(limb) = contact.is_contact;
        i++;
      }
  }


}

PLUGINLIB_EXPORT_CLASS(balance_controller::SingleLegController, controller_interface::ControllerBase)

