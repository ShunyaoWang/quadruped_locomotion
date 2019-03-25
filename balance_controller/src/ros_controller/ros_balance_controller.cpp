/*
 *  ros_balance_controller.cpp
 *  Descriotion:
 *
 *  Created on: Mar 18, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#include "balance_controller/ros_controler/ros_balance_controller.hpp"
#include "controller_manager/controller_manager.h"
namespace balance_controller{

  RosBalanceController::RosBalanceController()
  {
//    state_.reset();
//    robot_state_-
    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
    limbs_.push_back(free_gait::LimbEnum::RH_LEG);

    branches_.push_back(free_gait::BranchEnum::BASE);
    branches_.push_back(free_gait::BranchEnum::LF_LEG);
    branches_.push_back(free_gait::BranchEnum::RF_LEG);
    branches_.push_back(free_gait::BranchEnum::LH_LEG);
    branches_.push_back(free_gait::BranchEnum::RH_LEG);
    //! WSHY: initialize STAte
    robot_state_.reset(new free_gait::State);
    robot_state_->initialize(limbs_, branches_);

  };
  RosBalanceController::~RosBalanceController()
  {
    base_command_sub_.shutdown();
    contact_sub_.shutdown();
  };

  bool RosBalanceController::init(hardware_interface::RobotStateInterface* hardware,
                             ros::NodeHandle& node_handle)
  {
    ROS_INFO("Initializing RosBalanceController");
    contact_distribution_.reset(new ContactForceDistribution(node_handle, robot_state_));
    virtual_model_controller_.reset(new VirtualModelController(node_handle, robot_state_, contact_distribution_));
    //! WSHY: Load parameters for VMC controller
    if(!contact_distribution_->loadParameters())
      {
        ROS_INFO("CFD load parameters failed");
      }
    if(!virtual_model_controller_->loadParameters())
      {
        ROS_INFO("VMC load parameters failed");
      }
    //! WSHY: get joint handle from robot state handle
    std::string param_name = "joints";
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
//          joints.push_back(hardware->getHandle(joint_names[i]));
          joints.push_back(hardware->joint_effort_interfaces_.getHandle(joint_names[i]));
          ROS_INFO("Get '%s' Handle", joint_names[i].c_str());
//          hardware->g
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
          ROS_ERROR_STREAM("Exception thrown : "<< ex.what());
          return false;
        }
      }
    //! WSHY: get robot state handle
    robot_state_handle = hardware->getHandle("base_controller");

    for(unsigned int i=0;i<12;i++)
      robot_state_handle.getJointEffortWrite()[i] = 0;

    commands_buffer.writeFromNonRT(std::vector<double>(n_joints, 0.0));

    base_command_sub_ = node_handle.subscribe<free_gait_msgs::RobotState>("/desired_robot_state", 1, &RosBalanceController::baseCommandCallback, this);
    //! WSHY: having problem with update foot contact in gazebo_state_hardware_interface, so
    //! use subscribe, for real hardware interface, there should no problem fro this.
    contact_sub_ = node_handle.subscribe<sim_assiants::FootContacts>("/bumper_sensor_filter_node/foot_contacts", 1, &RosBalanceController::footContactsCallback, this);
    return true;
  };
  /**
   * @brief RosBalanceController::update, controller update loop
   * @param time
   * @param period
   */
  void RosBalanceController::update(const ros::Time& time, const ros::Duration& period)
  {
    ROS_DEBUG("Balance Controller Update Once");
    //! WSHY: update joint state
    free_gait::JointPositions all_joint_positions;
    //! WSHY: get joint postions from robot state handle
    for(unsigned int i=0; i<12; i++)
      {
        all_joint_positions(i) = robot_state_handle.getJointPositionRead()[i];
      }
//    ROS_INFO("Set current joint positions : \n");
//    std::cout<<all_joint_positions<<std::endl;
//    ROS_INFO("Get base position X : %f Y :%f Z : %f",robot_state_handle.getPosition()[0],robot_state_handle.getPosition()[1],robot_state_handle.getPosition()[2]);

    robot_state_->setCurrentLimbJoints(all_joint_positions);

    //! WSHY: update current base state from robot state handle

    robot_state_->setPoseBaseToWorld(Pose(Position(robot_state_handle.getPosition()[0],
                                                   robot_state_handle.getPosition()[1],
                                                   robot_state_handle.getPosition()[2]),
                                          RotationQuaternion(robot_state_handle.getOrientation()[0],
                                                             robot_state_handle.getOrientation()[1],
                                                             robot_state_handle.getOrientation()[2],
                                                             robot_state_handle.getOrientation()[3])));
    robot_state_->setBaseStateFromFeedback(LinearVelocity(robot_state_handle.getLinearVelocity()[0],
                                                          robot_state_handle.getLinearVelocity()[1],
                                                          robot_state_handle.getLinearVelocity()[2]),
                                           LocalAngularVelocity(robot_state_handle.getAngularVelocity()[0],
                                                                robot_state_handle.getAngularVelocity()[1],
                                                                robot_state_handle.getAngularVelocity()[2]));

    /****************
* TODO(Shunyao) : set support leg and surface normal, update from robot state handle
****************/

    //! WSHY: compute joint torque
    if(!virtual_model_controller_->compute())
      {
        ROS_INFO("VMC compute failed");
      }
    ROS_DEBUG_STREAM(*virtual_model_controller_);
    for(unsigned int i = 0; i<12; i++)
      {
        /****************
         * TODO(Shunyao) :  decide support leg to apply joint torque
         ****************/
//        if(robot_state_)
        ROS_DEBUG("Torque computed of joint %d is : %f\n",i,robot_state_->getAllJointEfforts()(i));
        double joint_torque_command = robot_state_->getAllJointEfforts()(i);
        robot_state_handle.getJointEffortWrite()[i] = joint_torque_command;
        joints[i].setCommand(joint_torque_command);
      }
  }
  /**
   * @brief RosBalanceController::baseCommandCallback, convert desired state, compute joint efforts
   * @param robot_state
   */
  void RosBalanceController::baseCommandCallback(const free_gait_msgs::RobotStateConstPtr& robot_state)
  {
    Position base_desired_position = Position(robot_state->base_pose.pose.pose.position.x,
                                      robot_state->base_pose.pose.pose.position.y,
                                      robot_state->base_pose.pose.pose.position.z);
    RotationQuaternion base_desired_rotation = RotationQuaternion(robot_state->base_pose.pose.pose.orientation.w,
                                                          robot_state->base_pose.pose.pose.orientation.x,
                                                          robot_state->base_pose.pose.pose.orientation.y,
                                                          robot_state->base_pose.pose.pose.orientation.z);
    LinearVelocity base_desired_linear_velocity = LinearVelocity(robot_state->base_pose.twist.twist.linear.x,
                                                                 robot_state->base_pose.twist.twist.linear.y,
                                                                 robot_state->base_pose.twist.twist.linear.z);
    LocalAngularVelocity base_desired_angular_velocity = LocalAngularVelocity(robot_state->base_pose.twist.twist.angular.x,
                                                                              robot_state->base_pose.twist.twist.angular.y,
                                                                              robot_state->base_pose.twist.twist.angular.z);
    ROS_INFO("Desired base pose: Position: ");
    std::cout<<base_desired_position<<std::endl;
    kindr::EulerAnglesZyxPD rotation(base_desired_rotation);
    ROS_INFO("Desired base pose: Rotation: ");
    std::cout<<rotation<<std::endl;

    free_gait::JointPositions all_joint_positions;
//    for(int i = 0;i<3;i++)
//    {
//        /****************
//        * TODO(Shunyao) : only for the non-support leg to follow the joint position and
//        *velocity command
//        ****************/
//      all_joint_positions(i) = robot_state->lf_leg_joints.position[i];
//      all_joint_positions(i+3) = robot_state->rf_leg_joints.position[i];
//      all_joint_positions(i+6) = robot_state->rh_leg_joints.position[i];
//      all_joint_positions(i+9) = robot_state->lh_leg_joints.position[i];
//    }


    robot_state_->setPositionWorldToBaseInWorldFrame(base_desired_position);
    robot_state_->setOrientationBaseToWorld(RotationQuaternion(base_desired_rotation));
    robot_state_->setLinearVelocityBaseInWorldFrame(base_desired_linear_velocity);
    robot_state_->setAngularVelocityBaseInBaseFrame(base_desired_angular_velocity);

    if(robot_state->lf_leg_mode.support_leg){
        robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, true);
        robot_state_->setSurfaceNormal(free_gait::LimbEnum::LF_LEG,
                                       Vector(robot_state->lf_leg_mode.surface_normal.vector.x,
                                              robot_state->lf_leg_mode.surface_normal.vector.y,
                                              robot_state->lf_leg_mode.surface_normal.vector.z));
      } else {
        robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, true);
        robot_state_->setSurfaceNormal(free_gait::LimbEnum::LF_LEG,Vector(0,0,0));
      };
    if(robot_state->rf_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(1), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(1),
                                       Vector(robot_state->rf_leg_mode.surface_normal.vector.x,
                                              robot_state->rf_leg_mode.surface_normal.vector.y,
                                              robot_state->rf_leg_mode.surface_normal.vector.z));
      } else {
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(1), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(1),Vector(0,0,0));
      };
    if(robot_state->rh_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(2), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(2),
                                       Vector(robot_state->rh_leg_mode.surface_normal.vector.x,
                                              robot_state->rh_leg_mode.surface_normal.vector.y,
                                              robot_state->rh_leg_mode.surface_normal.vector.z));
      } else {
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(2), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(2),Vector(0,0,0));
      };
    if(robot_state->lh_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(3), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(3),
                                       Vector(robot_state->lh_leg_mode.surface_normal.vector.x,
                                              robot_state->lh_leg_mode.surface_normal.vector.y,
                                              robot_state->lh_leg_mode.surface_normal.vector.z));
      } else {
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(3), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(3),Vector(0,0,0));
      };



  }


  void RosBalanceController::footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts)
  {
    /****************
* TODO(Shunyao) : change contact state for the early or late contact
****************/
    unsigned int i = 0;
    for(auto contact : foot_contacts->foot_contacts)
      {
        if(contact.is_contact)
          {
            robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(i), true);
            robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(i), Vector(contact.surface_normal.vector.x,
                                                                                       contact.surface_normal.vector.y,
                                                                                       contact.surface_normal.vector.z));

          }else{
            robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(i), false);
            robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(i), Vector(0, 0, 1));
          }
        i++;
      }
  }

  void RosBalanceController::starting(const ros::Time& time)
  {};

  void RosBalanceController::stopping(const ros::Time& time)
  {};



}

PLUGINLIB_EXPORT_CLASS(balance_controller::RosBalanceController, controller_interface::ControllerBase)
