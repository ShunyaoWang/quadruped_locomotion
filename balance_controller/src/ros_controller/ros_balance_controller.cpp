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
    log_length_ = 10000;
    log_index_ = log_length_;
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
    robot_state.reset(new free_gait::State);
    robot_state->initialize(limbs_, branches_);

    for(auto limb : limbs_)
      {
        limbs_state[limb].reset(new StateSwitcher);
        limbs_state.at(limb)->initialize(0);
        limbs_desired_state[limb].reset(new StateSwitcher);
        limbs_desired_state.at(limb)->initialize(0);
        t_sw0[limb] = ros::Time::now();
        t_st0[limb] = ros::Time::now();
        sw_phase[limb] = 0.0;
        st_phase[limb] = 0.0;
        sw_flag[limb] = false;
        st_flag[limb] = false;
        surface_normals[limb] = Vector(0,0,1);
        foot_positions[limb] = Vector(0,0,0);
        foot_velocities[limb] = Vector(0,0,0);
        foot_accelerations[limb] = Vector(0,0,0);
        store_current_joint_state_flag_[limb] = false;
        update_surface_normal_flag[limb] = false;
        real_contact_[limb] = false;
        is_cartisian_motion_[limb] = false;
        is_footstep_[limb] = false;
        is_legmode_[limb] = false;
      }
//    store_current_joint_state_flag_ = false;
//    stored_limb_joint_position_.resize(3);

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
    //! WSHY: base balance QP controller
    contact_distribution_.reset(new ContactForceDistribution(node_handle, robot_state));
    virtual_model_controller_.reset(new VirtualModelController(node_handle, robot_state, contact_distribution_));
    //! WSHY: single leg controller
    single_leg_solver_.reset(new MyRobotSolver(node_handle, robot_state));
//    single_leg_solver_->model_initialization();
    if(!single_leg_solver_->loadLimbModelFromURDF())
      {
        ROS_ERROR("Failed to load model from URDF file");
      }

    //! WSHY: Load parameters for VMC controller
    if(!contact_distribution_->loadParameters())
      {
        ROS_INFO("CFD load parameters failed");
      }
    if(!virtual_model_controller_->loadParameters())
      {
        ROS_INFO("VMC load parameters failed");
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
    pid_controllers_.resize(n_joints);
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
        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names[i]);
        if (!joint_urdf)
        {
          ROS_ERROR("Could not find joint '%s' in urdf", joint_names[i].c_str());
          return false;
        }
        joint_urdfs_.push_back(joint_urdf);

        // Load PID Controller using gains set on parameter server
        if (!pid_controllers_[i].init(ros::NodeHandle(node_handle, joint_names[i] + "/pid")))
        {
          ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names[i] + "/pid");
          return false;
        }


      }



    //! WSHY: get robot state handle
    robot_state_handle = hardware->getHandle("base_controller");

    for(unsigned int i=0;i<12;i++)
      robot_state_handle.getJointEffortWrite()[i] = 0;

    commands_buffer.writeFromNonRT(std::vector<double>(n_joints, 0.0));

//    command_foot_buffer.writeFromNonRT(LimbVector({{free_gait::LimbEnum::LF_LEG, Vector(0,0,0)},
//                                                   {free_gait::LimbEnum::RF_LEG, Vector(0,0,0)},
//                                                   {free_gait::LimbEnum::RH_LEG, Vector(0,0,0)},
//                                                   {free_gait::LimbEnum::LH_LEG, Vector(0,0,0)}}));
//    command_foot_vel_buffer.writeFromNonRT(LimbVector({{free_gait::LimbEnum::LF_LEG, Vector(0,0,0)},
//                                                   {free_gait::LimbEnum::RF_LEG, Vector(0,0,0)},
//                                                   {free_gait::LimbEnum::RH_LEG, Vector(0,0,0)},
//                                                   {free_gait::LimbEnum::LH_LEG, Vector(0,0,0)}}));

//    Eigen::VectorXd joints_positions;
//    for(unsigned int i = 0; i<12; i++)
//      joints_positions =
//    single_leg_solver_->setvecQAct(Eigen::Vector3d(0,0,0), free_gait::LimbEnum::LF_LEG);
//    single_leg_solver_->setvecQAct(Eigen::Vector3d(0,0,0), free_gait::LimbEnum::RF_LEG);
//    single_leg_solver_->setvecQAct(Eigen::Vector3d(0,0,0), free_gait::LimbEnum::RH_LEG);
//    single_leg_solver_->setvecQAct(Eigen::Vector3d(0,0,0), free_gait::LimbEnum::LH_LEG);

    log_data_srv_ = node_handle.advertiseService("/capture_log_data", &RosBalanceController::logDataCapture, this);

    base_command_sub_ = node_handle.subscribe<free_gait_msgs::RobotState>("/desired_robot_state", 1, &RosBalanceController::baseCommandCallback, this);
    //! WSHY: having problem with update foot contact in gazebo_state_hardware_interface, so
    //! use subscribe, for real hardware interface, there should no problem fro this.
    contact_sub_ = node_handle.subscribe<sim_assiants::FootContacts>("/bumper_sensor_filter_node/foot_contacts", 1, &RosBalanceController::footContactsCallback, this);

    //! WSHY: Logged data publisher
//    joint_command_pub_ = node_handle.advertise<sensor_msgs::JointState>("/balance_controller/joint_command", 1);
//    base_command_pub_ = node_handle.advertise<nav_msgs::Odometry>("/log/base_command", log_length_);
//    base_actual_pub_ = node_handle.advertise<nav_msgs::Odometry>("/log/base_actual", log_length_);
    leg_state_pub_ = node_handle.advertise<std_msgs::Int8MultiArray>("/log/leg_state", log_length_);
    joint_command_pub_ = node_handle.advertise<sensor_msgs::JointState>("/log/joint_command", log_length_);
    joint_actual_pub_ = node_handle.advertise<sensor_msgs::JointState>("/log/joint_state", log_length_);
    contact_desired_pub_ = node_handle.advertise<sim_assiants::FootContacts>("/log/desired_foot_contact", log_length_);
    leg_phase_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/log/leg_phase", log_length_);
    desired_robot_state_pub_ = node_handle.advertise<free_gait_msgs::RobotState>("/log/desired_robot_state", log_length_);
    actual_robot_state_pub_ = node_handle.advertise<free_gait_msgs::RobotState>("/log/actual_robot_state", log_length_);
    return true;
  };
  /**
   * @brief RosBalanceController::update, controller update loop
   * @param time
   * @param period
   */
  void RosBalanceController::update(const ros::Time& time, const ros::Duration& period)
  {
//    double real_time_factor = 0.001/period.toSec();
//    ROS_WARN_STREAM("Real Time Factor :"<<real_time_factor<<std::endl);
    ROS_DEBUG("Balance Controller Update Once");
    //! WSHY: update joint state
    free_gait::JointPositions all_joint_positions;
    free_gait::JointVelocities all_joint_velocities;
    free_gait::JointEfforts all_joint_efforts;
    //! WSHY: get joint postions from robot state handle
    for(unsigned int i=0; i<12; i++)
      {
        all_joint_positions(i) = robot_state_handle.getJointPositionRead()[i];
        all_joint_velocities(i) = robot_state_handle.getJointVelocityRead()[i];
        all_joint_efforts(i) = robot_state_handle.getJointEffortRead()[i];
      }

    boost::recursive_mutex::scoped_lock lock(r_mutex_);
    //! WSHY: read joint position command
    std::vector<double> & commands = *commands_buffer.readFromRT();
//    LimbVector & foot_posiion_commands = *command_foot_buffer.readFromRT();
//    LimbVector & foot_velocity_commands = *command_foot_vel_buffer.readFromRT();

    std_msgs::Int8MultiArray leg_state;
    std_msgs::Float64MultiArray leg_phase;
    leg_phase.data.resize(8);
    leg_state.data.resize(4);
    RotationQuaternion base_orinetation = RotationQuaternion(robot_state_handle.getOrientation()[0],
         robot_state_handle.getOrientation()[1],
         robot_state_handle.getOrientation()[2],
         robot_state_handle.getOrientation()[3]);
//    Vector surface_normal = base_orinetation.rotate(Vector(0,0,1));
    /****************
* TODO(Shunyao) : Rotate surface normals with base orinetation
****************/
    for(unsigned int i=0;i<4;i++)
      {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        //! WSHY: set foot cartesian motion for single leg controller
        robot_state->setTargetFootPositionInBaseForLimb(Position(foot_positions.at(limb).vector()), limb);
        robot_state->setTargetFootVelocityInBaseForLimb(LinearVelocity(foot_velocities.at(limb).vector()), limb);
        single_leg_solver_->setvecQAct(all_joint_positions.vector().segment(3*i, 3), limb);
        single_leg_solver_->setvecQDotAct(all_joint_velocities.vector().segment(3*i, 3), limb);

        switch (limbs_state.at(limb)->getState()) {
          case StateSwitcher::States::SwingNormal:
            {
              robot_state->setSupportLeg(limb, false);
              //            surface_normals.at(limb) = base_orinetation.rotate(Vector(0,0,1));
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              //            robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
              leg_state.data[i] = 0;
              store_current_joint_state_flag_.at(limb) = false;
              update_surface_normal_flag.at(limb) = false;
              ROS_INFO("Leg '%d' is in SwingNormal mode", i);
              break;
            }
          case StateSwitcher::States::StanceNormal:
            {
              robot_state->setSupportLeg(limb, true);

              //            if(!update_surface_normal_flag.at(limb))
              //              {
              //                update_surface_normal_flag.at(limb) = true;
              //                Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(limb);
              //                free_gait::JointEffortsLeg joint_effort = free_gait::JointEffortsLeg(all_joint_efforts.vector().segment(i*3, 3));
              //                free_gait::Force contact_force = free_gait::Force(jacobian * joint_effort.toImplementation());
              //                surface_normals.at(limb) = Vector(contact_force.normalize());
              //                if(limb == free_gait::LimbEnum::LF_LEG || limb == free_gait::LimbEnum::RH_LEG)
              //                  surface_normals.at(limb) = Vector(-contact_force.normalize());
              //              }
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              //            robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
              leg_state.data[i] = 2;
              store_current_joint_state_flag_.at(limb) = false;
              ROS_INFO("Leg '%d' is in StanceNormal mode", i);
              break;
            }
          case StateSwitcher::States::SwingEarlyTouchDown:
            {
              robot_state->setSupportLeg(limb, true);
              //              if(!update_surface_normal_flag.at(limb))
              //                {
              //                  update_surface_normal_flag.at(limb) = true;
              //                  Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(limb);
              //                  free_gait::JointEffortsLeg joint_effort = free_gait::JointEffortsLeg(all_joint_efforts.vector().segment(i*3, 3));
              //                  free_gait::Force contact_force = free_gait::Force(jacobian * joint_effort.toImplementation());
              //                  surface_normals.at(limb) = Vector(contact_force.normalize());
              //                  if(limb == free_gait::LimbEnum::LF_LEG || limb == free_gait::LimbEnum::RH_LEG)
              //                    surface_normals.at(limb) = Vector(-contact_force.normalize());
              //                }
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              //              robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
              //! WSHY: keep end effort position when early touch down
              ROS_WARN("Leg '%d' is in SwingEarlyTouchDown mode", i);
              leg_state.data[i] = 1;
              break;
            }
          case StateSwitcher::States::SwingBumpedIntoObstacle:
            {
              robot_state->setSupportLeg(limb, false);
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              LinearVelocity base_velocity_in_base = base_orinetation.inverseRotate(base_desired_linear_velocity);

              //! WSHY: move back(-x) and upward(+z)
              foot_positions.at(limb)(0) -= 0.005;
              foot_positions.at(limb)(2) += 0.02;
              robot_state->setTargetFootPositionInBaseForLimb(Position(foot_positions.at(limb).vector()), limb);

              break;
            }
          case StateSwitcher::States::SwingLatelyTouchDown:
            {
              /****************
            * TODO(Shunyao) : Directly move down?
            ****************/
              robot_state->setSupportLeg(limb, false);
              //            surface_normals.at(limb) = base_orinetation.rotate(Vector(0,0,1));
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              //! WSHY: directly move down to ground
              foot_positions.at(limb)(2) -= 0.01;
              robot_state->setTargetFootPositionInBaseForLimb(Position(foot_positions.at(limb).vector()), limb);
              //            robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
              //! WSHY: keep end effort position when early touch down
              if(!store_current_joint_state_flag_.at(limb)){
                  store_current_joint_state_flag_.at(limb) = true;
                  //                stored_limb_joint_position_[0] = robot_state_handle.getJointPositionRead()[3*i];
                  //                stored_limb_joint_position_[1] = robot_state_handle.getJointPositionRead()[3*i + 1];
                  //                stored_limb_joint_position_[2] = robot_state_handle.getJointPositionRead()[3*i + 2];
                  stored_limb_joint_position_.vector().segment(3*i,3) = all_joint_positions.vector().segment(3*i,3);
                } else {
                  commands[3*i] = stored_limb_joint_position_(3*i);
                  commands[3*i + 1] = stored_limb_joint_position_(3*i+1);
                  commands[3*i + 2] = stored_limb_joint_position_(3*i+2);
                }
              ROS_WARN("Leg '%d' is in SwingLatelyTouchDown mode", i);
              leg_state.data[i] = 3;
              break;
            }
          case StateSwitcher::States::StanceLostContact:
            {
              robot_state->setSupportLeg(limb, false);
              //            surface_normals.at(limb) = base_orinetation.rotate(Vector(0,0,1));
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              //            robot_state->setSurfaceNormal(limb, robot_state_->getSurfaceNormal(limb));
              //! WSHY: keep end effort position when early touch down
              if(!store_current_joint_state_flag_.at(limb)){
                  store_current_joint_state_flag_.at(limb) = true;
                  //                stored_limb_joint_position_[0] = robot_state_handle.getJointPositionRead()[3*i];
                  //                stored_limb_joint_position_[1] = robot_state_handle.getJointPositionRead()[3*i + 1];
                  //                stored_limb_joint_position_[2] = robot_state_handle.getJointPositionRead()[3*i + 2];
                  stored_limb_joint_position_.vector().segment(3*i,3) = all_joint_positions.vector().segment(3*i,3);
                } else {
                  commands[3*i] = stored_limb_joint_position_(3*i);
                  commands[3*i + 1] = stored_limb_joint_position_(3*i+1);
                  commands[3*i + 2] = stored_limb_joint_position_(3*i+2);
                }
              ROS_WARN("Leg '%d' is Lost Contact!!!", i);
              leg_state.data[i] = -1;
              break;
            }
          case StateSwitcher::States::Init:
            {
              robot_state->setSupportLeg(limb, true);
              //            surface_normals.at(limb) = base_orinetation.rotate(Vector(0,0,1));
              robot_state->setSurfaceNormal(limb, surface_normals.at(limb));
              //            robot_state->setSurfaceNormal(limb, Vector(0, 0, 1));
              break;
            }
          default:
            ROS_WARN("Unspecificed Limb State");

          }
        std::cout<<"surface normal for leg "<<i<<"("<<surface_normals.at(limb)<<")"<<std::endl;

//        robot_state->setSupportLeg(limb, true);
//       RotationQuaternion base_orinetation = RotationQuaternion(robot_state_handle.getOrientation()[0],
//            robot_state_handle.getOrientation()[1],
//            robot_state_handle.getOrientation()[2],
//            robot_state_handle.getOrientation()[3]);
        robot_state->setSurfaceNormal(limb, base_orinetation.rotate(Vector(0,0,1)));
//        robot_state->setSurfaceNormal(limb, Vector(0,0,1));
      }

    lock.unlock();
    //! WSHY: set the desired state
    robot_state->setPositionWorldToBaseInWorldFrame(base_desired_position);
    robot_state->setOrientationBaseToWorld(base_desired_rotation);
    robot_state->setLinearVelocityBaseInWorldFrame(base_desired_linear_velocity);
    robot_state->setAngularVelocityBaseInBaseFrame(base_desired_angular_velocity);



    //! WSHY: update current base state from robot state handle
    robot_state->setCurrentLimbJoints(all_joint_positions);
    robot_state->setCurrentLimbJointVelocities(all_joint_velocities);

//    ROS_ERROR("State Estimate Position in X : %f",robot_state_handle.getPosition()[0]);
    Pose current_base_pose = Pose(Position(robot_state_handle.getPosition()[0],
                                  robot_state_handle.getPosition()[1],
                                  robot_state_handle.getPosition()[2]),
                         RotationQuaternion(robot_state_handle.getOrientation()[0],
                                            robot_state_handle.getOrientation()[1],
                                            robot_state_handle.getOrientation()[2],
                                            robot_state_handle.getOrientation()[3]));

    robot_state->setPoseBaseToWorld(current_base_pose);
    robot_state->setBaseStateFromFeedback(LinearVelocity(robot_state_handle.getLinearVelocity()[0],
                                                          robot_state_handle.getLinearVelocity()[1],
                                                          robot_state_handle.getLinearVelocity()[2]),
                                           LocalAngularVelocity(robot_state_handle.getAngularVelocity()[0],
                                                                robot_state_handle.getAngularVelocity()[1],
                                                                robot_state_handle.getAngularVelocity()[2]));


    /****************
* TODO(Shunyao) : set support leg and surface normal, update from robot state handle
****************/

    //! WSHY: compute joint torque
    bool keep_flag = false;
    if(!virtual_model_controller_->compute())
      {
        ROS_ERROR("VMC compute failed");
        ROS_WARN_STREAM(*virtual_model_controller_);
        keep_flag = true;
      }
//    ROS_WARN_STREAM("norminal : "<<contact_distribution_->getNormalDirectionOfFrictionPyramidInWorldFrame(free_gait::LimbEnum::LF_LEG).toImplementation()<<std::endl);
//    ROS_WARN_STREAM("First : "<<contact_distribution_->getFirstDirectionOfFrictionPyramidInWorldFrame(free_gait::LimbEnum::LF_LEG).toImplementation()<<std::endl);
//    ROS_WARN_STREAM("Second : "<<contact_distribution_->getSecondDirectionOfFrictionPyramidInWorldFrame(free_gait::LimbEnum::LF_LEG).toImplementation()<<std::endl);


//    ROS_DEBUG_STREAM(*virtual_model_controller_);
    //! WSHY: for logging data
    sensor_msgs::JointState joint_command, joint_actual;
    joint_command.effort.resize(12);
    joint_command.position.resize(12);
    joint_command.name.resize(12);
    joint_actual.name.resize(12);
    joint_actual.position.resize(12);
    joint_actual.velocity.resize(12);
    joint_actual.effort.resize(12);

    for(unsigned int i = 0; i<12; i++)
      {
        /****************
         * TODO(Shunyao) :  decide support leg to apply joint torque
         ****************/
//        if(robot_state_)
        double joint_torque_command = robot_state->getAllJointEfforts()(i);
        ROS_DEBUG("Torque computed of joint %d is : %f\n",i,joint_torque_command);

        robot_state_handle.getJointEffortWrite()[i] = joint_torque_command;
        if(joint_torque_command>300)
          joint_torque_command = 300;
        if(joint_torque_command<-300)
          joint_torque_command = -300;
//        if(!keep_flag)
//          {
//            joints[i].setCommand(joint_torque_command);
//          }
        joint_command.name[i] = joint_names[i];
        joint_command.effort[i] = joint_torque_command;
        joint_command.position[i] = commands[i];
        joint_actual.name[i] = joint_names[i];
        joint_actual.position[i] = all_joint_positions(i);
        joint_actual.velocity[i] = all_joint_velocities(i);
        joint_actual.effort[i] = 0;//all_joint_efforts(i);
      }
//    joint_command_pub_.publish(joint_command);
    //! WSHY: for Swing Leg Control
//    std::vector<double> & commands = *commands_buffer.readFromRT();
    ros::Duration real_time_period = ros::Duration(period.toSec());
    free_gait::Force gravity_in_base = base_orinetation.rotate(free_gait::Force(0,0,-9.8));
    if(!robot_state->isSupportLeg(free_gait::LimbEnum::LF_LEG))
      {
        //! WSHY: compute gravity compensation
        free_gait::JointPositionsLeg joint_position_leg = free_gait::JointPositionsLeg(all_joint_positions.vector().segment(0,3));
        free_gait::JointEffortsLeg gravity_compensation_torque = robot_state->getGravityCompensationForLimb(free_gait::LimbEnum::LF_LEG,
                                                                                joint_position_leg,
                                                                                gravity_in_base);
//        single_leg_solver_->setvecQAct(joint_position_leg.vector(),free_gait::LimbEnum::LF_LEG);
        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::LF_LEG);

        for(int i = 0;i<3;i++)
          {
            double effort_command = computeTorqueFromPositionCommand(commands[i], i, real_time_period);
            if(is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) || is_footstep_.at(free_gait::LimbEnum::LF_LEG))
              effort_command = single_leg_solver_->getVecTauAct()[i];
            else if(!is_legmode_.at(free_gait::LimbEnum::LF_LEG))
              effort_command += gravity_compensation_torque(i);
            else
              effort_command = gravity_compensation_torque(i);
            joints[i].setCommand(effort_command);
            //! WSHY: data logging
            joint_command.effort[i] = effort_command;
            joint_actual.position[i] = single_leg_solver_->getQAcutal().row(1)[i];
            joint_actual.velocity[i] = single_leg_solver_->getQDotAcutal().row(1)[i];
            joint_actual.effort[i] = single_leg_solver_->getQDDotAcutal().row(1)[i];
          }
        //        ROS_INFO("LF_LEG is NOT Contacted");
      }else{ // for stance leg
        for(unsigned int i=0;i<3;i++)
          {
            joints[i].setCommand(joint_command.effort[i]);
          }
      }
    if(!robot_state->isSupportLeg(free_gait::LimbEnum::RF_LEG))
      {

        free_gait::JointPositionsLeg joint_position_leg = free_gait::JointPositionsLeg(all_joint_positions.vector().segment(3,3));
        free_gait::JointEffortsLeg gravity_compensation_torque = robot_state->getGravityCompensationForLimb(free_gait::LimbEnum::RF_LEG,
                                                                                joint_position_leg,
                                                                                gravity_in_base);

//        single_leg_solver_->setvecQAct(joint_position_leg.vector(),free_gait::LimbEnum::RF_LEG);
        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::RF_LEG);

        for(int i = 0;i<3;i++)
          {
            double effort_command = computeTorqueFromPositionCommand(commands[i+3], i+3, real_time_period);
            if(is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) || is_footstep_.at(free_gait::LimbEnum::RF_LEG))
              effort_command = single_leg_solver_->getVecTauAct()[i];
            else if(!is_legmode_.at(free_gait::LimbEnum::RF_LEG))
              effort_command += gravity_compensation_torque(i);
            else
              effort_command = gravity_compensation_torque(i);
            joints[i+3].setCommand(effort_command);
            joint_command.effort[i+3] = effort_command;

            joint_actual.position[i+3] = single_leg_solver_->getQAcutal().row(1)[i];
            joint_actual.velocity[i+3] = single_leg_solver_->getQDotAcutal().row(1)[i];
            joint_actual.effort[i+3] = single_leg_solver_->getTauAcutal().row(0)[i]; //single_leg_solver_->getQDDotAcutal().row(1)[i];
          }
        //        ROS_INFO("RF_LEG is NOT Contacted");
      }else{
        for(unsigned int i=0;i<3;i++)
          {
            joints[i+3].setCommand(joint_command.effort[i+3]);
          }
      }
    if(!robot_state->isSupportLeg(free_gait::LimbEnum::RH_LEG))
      {

        free_gait::JointPositionsLeg joint_position_leg = free_gait::JointPositionsLeg(all_joint_positions.vector().segment(6,3));
        free_gait::JointEffortsLeg gravity_compensation_torque = robot_state->getGravityCompensationForLimb(free_gait::LimbEnum::RH_LEG,
                                                                                joint_position_leg,
                                                                                gravity_in_base);

//        single_leg_solver_->setvecQAct(joint_position_leg.vector(),free_gait::LimbEnum::RH_LEG);
        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::RH_LEG);
        for(int i = 0;i<3;i++)
          {
            double effort_command =computeTorqueFromPositionCommand(commands[i+6], i+6, real_time_period);
            if(is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) || is_footstep_.at(free_gait::LimbEnum::RH_LEG))
              effort_command = single_leg_solver_->getVecTauAct()[i];
            else if(!is_legmode_.at(free_gait::LimbEnum::RH_LEG))
              effort_command += gravity_compensation_torque(i);
            else
              effort_command = gravity_compensation_torque(i);
            joints[i+6].setCommand(effort_command);
            joint_command.effort[i+6] = effort_command;

            joint_actual.position[i+6] = single_leg_solver_->getQAcutal().row(1)[i];
            joint_actual.velocity[i+6] = single_leg_solver_->getQDotAcutal().row(1)[i];
            joint_actual.effort[i+6] = single_leg_solver_->getTauAcutal().row(0)[i]; //single_leg_solver_->getQDDotAcutal().row(1)[i];
          }
        //        ROS_INFO("RH_LEG is NOT Contacted");
      }else{
        for(unsigned int i=0;i<3;i++)
          {
            joints[i+6].setCommand(joint_command.effort[i+6]);
          }
      }
    if(!robot_state->isSupportLeg(free_gait::LimbEnum::LH_LEG))
      {
        free_gait::JointPositionsLeg joint_position_leg = free_gait::JointPositionsLeg(all_joint_positions.vector().segment(9,3));
        free_gait::JointEffortsLeg gravity_compensation_torque = robot_state->getGravityCompensationForLimb(free_gait::LimbEnum::LH_LEG,
                                                                                joint_position_leg,
                                                                                gravity_in_base);
//        single_leg_solver_->setvecQAct(joint_position_leg.vector(),free_gait::LimbEnum::LH_LEG);
        single_leg_solver_->update(time, real_time_period, free_gait::LimbEnum::LH_LEG);

        for(int i = 0;i<3;i++)
          {
            double effort_command =computeTorqueFromPositionCommand(commands[i+9], i+9, real_time_period);
            if(is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) || is_footstep_.at(free_gait::LimbEnum::LH_LEG))
              effort_command = single_leg_solver_->getVecTauAct()[i];
            else if(!is_legmode_.at(free_gait::LimbEnum::LH_LEG))
              effort_command += gravity_compensation_torque(i);
            else
              effort_command = gravity_compensation_torque(i);
            joints[i+9].setCommand(effort_command);
            joint_command.effort[i+9] = effort_command;

            joint_actual.position[i+9] = single_leg_solver_->getQAcutal().row(1)[i];
            joint_actual.velocity[i+9] = single_leg_solver_->getQDotAcutal().row(1)[i];
            joint_actual.effort[i+9] = single_leg_solver_->getTauAcutal().row(0)[i]; //single_leg_solver_->getQDDotAcutal().row(1)[i];
          }
        //        ROS_INFO("LH_LEG is NOT Contacted");
      }else{
        for(unsigned int i=0;i<3;i++)
          {
            joints[i+9].setCommand(joint_command.effort[i+9]);
          }
      }
//    lock.unlock();
    //! WSHY: data logging
    if(base_actual_pose_.size()<log_length_)
      {
        free_gait_msgs::RobotState desired_robot_state, actual_robot_state;
        desired_robot_state.lf_target.target_position.resize(1);
        desired_robot_state.lf_target.target_velocity.resize(1);
        actual_robot_state.lf_target.target_position.resize(1);
        actual_robot_state.lf_target.target_velocity.resize(1);
        desired_robot_state.rf_target.target_position.resize(1);
        desired_robot_state.rf_target.target_velocity.resize(1);
        actual_robot_state.rf_target.target_position.resize(1);
        actual_robot_state.rf_target.target_velocity.resize(1);
        desired_robot_state.rh_target.target_position.resize(1);
        desired_robot_state.rh_target.target_velocity.resize(1);
        actual_robot_state.rh_target.target_position.resize(1);
        actual_robot_state.rh_target.target_velocity.resize(1);
        desired_robot_state.lh_target.target_position.resize(1);
        desired_robot_state.lh_target.target_velocity.resize(1);
        actual_robot_state.lh_target.target_position.resize(1);
        actual_robot_state.lh_target.target_velocity.resize(1);
        geometry_msgs::Pose desired_pose, actual_pose;
        geometry_msgs::Twist desired_twist, actual_twist;
        nav_msgs::Odometry desire_odom, actual_odom;
        kindr_ros::convertToRosGeometryMsg(base_desired_position, desired_pose.position);
        kindr_ros::convertToRosGeometryMsg(base_desired_rotation, desired_pose.orientation);
        kindr_ros::convertToRosGeometryMsg(base_desired_linear_velocity, desired_twist.linear);
        kindr_ros::convertToRosGeometryMsg(base_desired_angular_velocity, desired_twist.angular);
        kindr_ros::convertToRosGeometryMsg(current_base_pose, actual_pose);
        actual_twist.linear.x = robot_state_handle.getLinearVelocity()[0];
        actual_twist.linear.y = robot_state_handle.getLinearVelocity()[1];
        actual_twist.linear.z = robot_state_handle.getLinearVelocity()[2];
        actual_twist.angular.x = robot_state_handle.getAngularVelocity()[0];
        actual_twist.angular.y = robot_state_handle.getAngularVelocity()[1];
        actual_twist.angular.z = robot_state_handle.getAngularVelocity()[2];
        desire_odom.pose.pose = desired_pose;
        desire_odom.twist.twist = desired_twist;
        actual_odom.pose.pose = actual_pose;
        actual_odom.twist.twist = actual_twist;

//        base_actual_pose_.push_back(actual_odom);
//        base_command_pose_.push_back(desire_odom);
        leg_states_.push_back(leg_state);
        joint_command_.push_back(joint_command);
        joint_actual_.push_back(joint_actual);

        sim_assiants::FootContacts desired_contact;
        desired_contact.foot_contacts.resize(4);
        for(int i = 0; i<4; i++)
          {
            free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
            Force contact_force_in_base = contact_distribution_->getLegInfo(limb).desiredContactForce_;
            Force contact_force_in_world = robot_state->getOrientationBaseToWorld().rotate(contact_force_in_base);
            desired_contact.foot_contacts[i].contact_force.wrench.force.x = contact_force_in_world(0);
            desired_contact.foot_contacts[i].contact_force.wrench.force.y = contact_force_in_world(1);
            desired_contact.foot_contacts[i].contact_force.wrench.force.z = contact_force_in_world(2);
//            ROS_INFO("Log surface normals");
            desired_contact.foot_contacts[i].surface_normal.vector.x = surface_normals.at(limb)(0);
            desired_contact.foot_contacts[i].surface_normal.vector.y = surface_normals.at(limb)(1);
            desired_contact.foot_contacts[i].surface_normal.vector.z = surface_normals.at(limb)(2);

//            if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::StanceNormal)
//              desired_contact.foot_contacts[i].is_contact = true;
//            if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::SwingNormal)
//              desired_contact.foot_contacts[i].is_contact = false;
            desired_contact.foot_contacts[i].is_contact = real_contact_.at(limb);

            leg_phase.data[2*i] = st_phase.at(limb);
            leg_phase.data[2*i + 1] = sw_phase.at(limb);
          }
        leg_phases_.push_back(leg_phase);
        foot_desired_contact_.push_back(desired_contact);

        desired_robot_state.base_pose = desire_odom;
        kindr_ros::convertToRosGeometryMsg(Position(foot_positions.at(free_gait::LimbEnum::LF_LEG).vector()),
                                           desired_robot_state.lf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(Position(foot_positions.at(free_gait::LimbEnum::RF_LEG).vector()),
                                           desired_robot_state.rf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(Position(foot_positions.at(free_gait::LimbEnum::RH_LEG).vector()),
                                           desired_robot_state.rh_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(Position(foot_positions.at(free_gait::LimbEnum::LH_LEG).vector()),
                                           desired_robot_state.lh_target.target_position[0].point);

        kindr_ros::convertToRosGeometryMsg(LinearVelocity(foot_velocities.at(free_gait::LimbEnum::LF_LEG).vector()),
                                           desired_robot_state.lf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(LinearVelocity(foot_velocities.at(free_gait::LimbEnum::RF_LEG).vector()),
                                           desired_robot_state.rf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(LinearVelocity(foot_velocities.at(free_gait::LimbEnum::RH_LEG).vector()),
                                           desired_robot_state.rh_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(LinearVelocity(foot_velocities.at(free_gait::LimbEnum::LH_LEG).vector()),
                                           desired_robot_state.lh_target.target_velocity[0].vector);
        desired_robot_state_.push_back(desired_robot_state);

        actual_robot_state.base_pose = actual_odom;
        kindr_ros::convertToRosGeometryMsg(robot_state->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LF_LEG),
                                           actual_robot_state.lf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(robot_state->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::RF_LEG),
                                           actual_robot_state.rf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(robot_state->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::RH_LEG),
                                           actual_robot_state.rh_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(robot_state->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LH_LEG),
                                           actual_robot_state.lh_target.target_position[0].point);

        kindr_ros::convertToRosGeometryMsg(robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LF_LEG),
                                           actual_robot_state.lf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RF_LEG),
                                           actual_robot_state.rf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RH_LEG),
                                           actual_robot_state.rh_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(robot_state->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LH_LEG),
                                           actual_robot_state.lh_target.target_velocity[0].vector);
        actual_robot_state_.push_back(actual_robot_state);
      }

  }

  double RosBalanceController::computeTorqueFromPositionCommand(double command, int i, const ros::Duration& period)
  {
//    ros::Duration real_time_period = ros::Duration(period.toSec()/0.55);
    double command_position = command;

    double error; //, vel_error;
    double commanded_effort;

    double current_position = joints[i].getPosition();

    // Make sure joint is within limits if applicable
    enforceJointLimits(command_position, i);

    // Compute position error
    if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
    {
     angles::shortest_angular_distance_with_limits(
        current_position,
        command_position,
        joint_urdfs_[i]->limits->lower,
        joint_urdfs_[i]->limits->upper,
        error);
    }
    else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
    {
      error = angles::shortest_angular_distance(current_position, command_position);
    }
    else //prismatic
    {
      error = command_position - current_position;
    }

    // Set the PID error and compute the PID command with nonuniform
    // time step size.
    commanded_effort = pid_controllers_[i].computeCommand(error, period);
    return commanded_effort;
  }
  /**
   * @brief RosBalanceController::baseCommandCallback, convert desired state, compute joint efforts
   * @param robot_state
   */
  void RosBalanceController::baseCommandCallback(const free_gait_msgs::RobotStateConstPtr& robot_state_msg)
  {
    base_desired_position = Position(robot_state_msg->base_pose.pose.pose.position.x,
                                      robot_state_msg->base_pose.pose.pose.position.y,
                                      robot_state_msg->base_pose.pose.pose.position.z);
    base_desired_rotation = RotationQuaternion(robot_state_msg->base_pose.pose.pose.orientation.w,
                                                          robot_state_msg->base_pose.pose.pose.orientation.x,
                                                          robot_state_msg->base_pose.pose.pose.orientation.y,
                                                          robot_state_msg->base_pose.pose.pose.orientation.z);
    base_desired_linear_velocity = LinearVelocity(robot_state_msg->base_pose.twist.twist.linear.x,
                                                                 robot_state_msg->base_pose.twist.twist.linear.y,
                                                                 robot_state_msg->base_pose.twist.twist.linear.z);
    base_desired_angular_velocity = LocalAngularVelocity(robot_state_msg->base_pose.twist.twist.angular.x,
                                                                              robot_state_msg->base_pose.twist.twist.angular.y,
                                                                              robot_state_msg->base_pose.twist.twist.angular.z);

//    Pose current_base_pose = Pose(Position(robot_state_handle.getPosition()[0],
//                                  robot_state_handle.getPosition()[1],
//                                  robot_state_handle.getPosition()[2]),
//                         RotationQuaternion(robot_state_handle.getOrientation()[0],
//                                            robot_state_handle.getOrientation()[1],
//                                            robot_state_handle.getOrientation()[2],
//                                            robot_state_handle.getOrientation()[3]));
//    ROS_INFO("Desired base pose: Position: ");
//    std::cout<<base_desired_position<<std::endl;
//    kindr::EulerAnglesZyxPD rotation_desired(base_desired_rotation);
//    ROS_INFO("Desired base pose: Rotation: ");
//    std::cout<<rotation_desired<<std::endl;
//    ROS_INFO("Current base pose: Position: ");
//    std::cout<<current_base_pose.getPosition()<<std::endl;
//    kindr::EulerAnglesZyxPD rotation_current(current_base_pose.getRotation());
//    ROS_INFO("Current base pose: Rotation: ");
//    std::cout<<rotation_current<<std::endl;
//    ROS_INFO("Orienitaion Error is : ");
//    std::cout<<base_desired_rotation.boxMinus(current_base_pose.getRotation())<<std::endl;

//    free_gait::JointPositions all_joint_positions;
    std::vector<double> joint_commands;
    joint_commands.resize(12);
    for(unsigned int i = 0;i<3;i++)
    {
        /****************
        * TODO(Shunyao) : only for the non-support leg to follow the joint position and
        *velocity command
        ****************/
      joint_commands[i] = robot_state_msg->lf_leg_joints.position[i];
      joint_commands[i+3] = robot_state_msg->rf_leg_joints.position[i];
      joint_commands[i+6] = robot_state_msg->rh_leg_joints.position[i];
      joint_commands[i+9] = robot_state_msg->lh_leg_joints.position[i];
    }
    commands_buffer.writeFromNonRT(joint_commands);

//    LimbVector foot_positions, foot_velocities, foot_accelerations;
    //! WSHY: update cartesian foot motion command
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


//    command_foot_buffer.writeFromNonRT(foot_positions);
//    command_foot_vel_buffer.writeFromNonRT(foot_velocities);

    robot_state_->setPositionWorldToBaseInWorldFrame(base_desired_position);
    robot_state_->setOrientationBaseToWorld(RotationQuaternion(base_desired_rotation));
    robot_state_->setLinearVelocityBaseInWorldFrame(base_desired_linear_velocity);
    robot_state_->setAngularVelocityBaseInBaseFrame(base_desired_angular_velocity);

    if(robot_state_msg->lf_leg_mode.name == "joint")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
      }
    if(robot_state_msg->lf_leg_mode.name == "leg_mode")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = true;
      }
    if(robot_state_msg->lf_leg_mode.name == "cartesian")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
      }
    if(robot_state_msg->lf_leg_mode.name == "footstep")
      {
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = true;
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
      }

    if(robot_state_msg->rf_leg_mode.name == "joint")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
      }
    if(robot_state_msg->rf_leg_mode.name == "leg_mode")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = true;
      }
    if(robot_state_msg->rf_leg_mode.name == "cartesian")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
      }
    if(robot_state_msg->rf_leg_mode.name == "footstep")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = true;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
      }

    if(robot_state_msg->rh_leg_mode.name == "joint")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
      }
    if(robot_state_msg->rh_leg_mode.name == "leg_mode")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = true;
      }
    if(robot_state_msg->rh_leg_mode.name == "cartesian")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
      }
    if(robot_state_msg->rh_leg_mode.name == "footstep")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = true;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
      }

    if(robot_state_msg->lh_leg_mode.name == "joint")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
      }
    if(robot_state_msg->lh_leg_mode.name == "leg_mode")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = true;
      }
    if(robot_state_msg->lh_leg_mode.name == "cartesian")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
      }
    if(robot_state_msg->lh_leg_mode.name == "footstep")
      {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = true;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
      }

    if(robot_state_msg->lf_leg_mode.support_leg){
        robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, true);
        robot_state_->setSurfaceNormal(free_gait::LimbEnum::LF_LEG,
                                       Vector(robot_state_msg->lf_leg_mode.surface_normal.vector.x,
                                              robot_state_msg->lf_leg_mode.surface_normal.vector.y,
                                              robot_state_msg->lf_leg_mode.surface_normal.vector.z));

        limbs_desired_state.at(free_gait::LimbEnum::LF_LEG)->setState(StateSwitcher::States::StanceNormal);
        if(st_flag.at(free_gait::LimbEnum::LF_LEG)){
            st_flag.at(free_gait::LimbEnum::LF_LEG) = false;
            t_st0.at(free_gait::LimbEnum::LF_LEG)= ros::Time::now();
          }
        sw_flag.at(free_gait::LimbEnum::LF_LEG) = false;
        st_phase.at(free_gait::LimbEnum::LF_LEG) = robot_state_msg->lf_leg_mode.phase;
        sw_phase.at(free_gait::LimbEnum::LF_LEG) = 0;
      } else {
//        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

        robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, false);
        robot_state_->setSurfaceNormal(free_gait::LimbEnum::LF_LEG,Vector(0,0,1));
        limbs_desired_state.at(free_gait::LimbEnum::LF_LEG)->setState(StateSwitcher::States::SwingNormal);
        if(!sw_flag.at(free_gait::LimbEnum::LF_LEG)){
          t_sw0.at(free_gait::LimbEnum::LF_LEG) = ros::Time::now();
          sw_flag.at(free_gait::LimbEnum::LF_LEG) = true;
          }
        st_flag.at(free_gait::LimbEnum::LF_LEG) = true;
        sw_phase.at(free_gait::LimbEnum::LF_LEG) = robot_state_msg->lf_leg_mode.phase;
        st_phase.at(free_gait::LimbEnum::LF_LEG) = 0;

      };
    if(robot_state_msg->rf_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(1), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(1),
                                       Vector(robot_state_msg->rf_leg_mode.surface_normal.vector.x,
                                              robot_state_msg->rf_leg_mode.surface_normal.vector.y,
                                              robot_state_msg->rf_leg_mode.surface_normal.vector.z));
        limbs_desired_state.at(free_gait::LimbEnum::RF_LEG)->setState(StateSwitcher::States::StanceNormal);
        if(st_flag.at(free_gait::LimbEnum::RF_LEG)){
            st_flag.at(free_gait::LimbEnum::RF_LEG) = false;
            t_st0.at(free_gait::LimbEnum::RF_LEG)= ros::Time::now();
          }
        sw_flag.at(free_gait::LimbEnum::RF_LEG) = false;
        st_phase.at(free_gait::LimbEnum::RF_LEG) = robot_state_msg->rf_leg_mode.phase;
        sw_phase.at(free_gait::LimbEnum::RF_LEG) = 0;
      } else {
//        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(1), false);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(1),Vector(0,0,1));
        limbs_desired_state.at(free_gait::LimbEnum::RF_LEG)->setState(StateSwitcher::States::SwingNormal);
        if(!sw_flag.at(free_gait::LimbEnum::RF_LEG)){
          t_sw0.at(free_gait::LimbEnum::RF_LEG) = ros::Time::now();
          sw_flag.at(free_gait::LimbEnum::RF_LEG) = true;
          }
        st_flag.at(free_gait::LimbEnum::RF_LEG) = true;
        sw_phase.at(free_gait::LimbEnum::RF_LEG) = robot_state_msg->rf_leg_mode.phase;
        st_phase.at(free_gait::LimbEnum::RF_LEG) = 0;

      };
    if(robot_state_msg->rh_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(2), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(2),
                                       Vector(robot_state_msg->rh_leg_mode.surface_normal.vector.x,
                                              robot_state_msg->rh_leg_mode.surface_normal.vector.y,
                                              robot_state_msg->rh_leg_mode.surface_normal.vector.z));
        limbs_desired_state.at(free_gait::LimbEnum::RH_LEG)->setState(StateSwitcher::States::StanceNormal);
        if(st_flag.at(free_gait::LimbEnum::RH_LEG)){
            st_flag.at(free_gait::LimbEnum::RH_LEG) = false;
            t_st0.at(free_gait::LimbEnum::RH_LEG)= ros::Time::now();
          }
        sw_flag.at(free_gait::LimbEnum::RH_LEG) = false;
        st_phase.at(free_gait::LimbEnum::RH_LEG) = robot_state_msg->rh_leg_mode.phase;
        sw_phase.at(free_gait::LimbEnum::RH_LEG) = 0;
      } else {
//        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(2), false);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(2),Vector(0,0,1));
        limbs_desired_state.at(free_gait::LimbEnum::RH_LEG)->setState(StateSwitcher::States::SwingNormal);
        if(!sw_flag.at(free_gait::LimbEnum::RH_LEG)){
          t_sw0.at(free_gait::LimbEnum::RH_LEG) = ros::Time::now();
          sw_flag.at(free_gait::LimbEnum::RH_LEG) = true;
          }
        st_flag.at(free_gait::LimbEnum::RH_LEG) = true;
        sw_phase.at(free_gait::LimbEnum::RH_LEG) = robot_state_msg->rh_leg_mode.phase;
        st_phase.at(free_gait::LimbEnum::RH_LEG) = 0;

      };
    if(robot_state_msg->lh_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(3), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(3),
                                       Vector(robot_state_msg->lh_leg_mode.surface_normal.vector.x,
                                              robot_state_msg->lh_leg_mode.surface_normal.vector.y,
                                              robot_state_msg->lh_leg_mode.surface_normal.vector.z));
        limbs_desired_state.at(free_gait::LimbEnum::LH_LEG)->setState(StateSwitcher::States::StanceNormal);
        if(st_flag.at(free_gait::LimbEnum::LH_LEG)){
            st_flag.at(free_gait::LimbEnum::LH_LEG) = false;
            t_st0.at(free_gait::LimbEnum::LH_LEG)= ros::Time::now();
          }
        sw_flag.at(free_gait::LimbEnum::LH_LEG) = false;
        st_phase.at(free_gait::LimbEnum::LH_LEG) = robot_state_msg->lh_leg_mode.phase;
        sw_phase.at(free_gait::LimbEnum::LH_LEG) = 0;
      } else {
//        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(3), false);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(3),Vector(0,0,1));
        limbs_desired_state.at(free_gait::LimbEnum::LH_LEG)->setState(StateSwitcher::States::SwingNormal);
        if(!sw_flag.at(free_gait::LimbEnum::LH_LEG)){
          t_sw0.at(free_gait::LimbEnum::LH_LEG) = ros::Time::now();
          sw_flag.at(free_gait::LimbEnum::LH_LEG) = true;
          }
        st_flag.at(free_gait::LimbEnum::LH_LEG) = true;
        sw_phase.at(free_gait::LimbEnum::LH_LEG) = robot_state_msg->lh_leg_mode.phase;
        st_phase.at(free_gait::LimbEnum::LH_LEG) = 0;

      };

//  std::cout<<*robot_state_<<std::endl;

  }


  void RosBalanceController::footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts)
  {
    /****************
* TODO(Shunyao) : change contact state for the early or late contact
****************/
    unsigned int i = 0;
    for(auto contact : foot_contacts->foot_contacts)
      {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
//        ROS_INFO("swing time for leg %d is : %f", i, t_sw0.at(limb).toSec());
        real_contact_.at(limb) = contact.is_contact;
        if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::SwingNormal)
          {
            limbs_state.at(limb)->setState(StateSwitcher::States::SwingNormal);
            if(!is_footstep_.at(limb))
              continue;
            if(sw_phase.at(limb)>0.5)//(ros::Time::now().toSec() - t_sw0.at(limb).toSec()) > 0.2)
              {
                if(contact.is_contact)
                  {
                    limbs_state.at(limb)->setState(StateSwitcher::States::SwingEarlyTouchDown);
                  }/*else{
                    limbs_state.at(limb)->setState(StateSwitcher::States::SwingBumpedIntoObstacle);
                  }*/
              }else if(sw_phase.at(limb)>0.2){
                if(contact.is_contact)
                  {
                    limbs_state.at(limb)->setState(StateSwitcher::States::SwingBumpedIntoObstacle);
                  }
              }

          }

        if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::StanceNormal)
          {
            if(!is_footstep_.at(limb))
              {
                limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
                continue;
              }
            if(contact.is_contact){
                limbs_state.at(limb)->setState(StateSwitcher::States::StanceNormal);
              } else if(st_phase.at(limb) < 0.1){
                limbs_state.at(limb)->setState(StateSwitcher::States::SwingLatelyTouchDown);
              }
            if(st_phase.at(limb) > 0.5)//(ros::Time::now().toSec() - t_st0.at(limb).toSec()) > 0.2)
              {
                if(!contact.is_contact){
                    limbs_state.at(limb)->setState(StateSwitcher::States::StanceLostContact);
                  }
              }
          }
        i++;
      }
  }

  void RosBalanceController::starting(const ros::Time& time)
  {
//    base_command_pose_.clear();
//    base_actual_pose_.clear();
    leg_states_.clear();
    joint_actual_.clear();
    joint_command_.clear();
    foot_desired_contact_.clear();
    leg_phases_.clear();
    desired_robot_state_.clear();
    actual_robot_state_.clear();
  };

  void RosBalanceController::stopping(const ros::Time& time)
  {};

  void RosBalanceController::enforceJointLimits(double &command, unsigned int index)
    {
      // Check that this joint has applicable limits
      if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
      {
        if( command > joint_urdfs_[index]->limits->upper ) // above upper limnit
        {
          command = joint_urdfs_[index]->limits->upper;
        }
        else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
        {
          command = joint_urdfs_[index]->limits->lower;
        }
      }
    }
  bool RosBalanceController::logDataCapture(std_srvs::Empty::Request& req,
                      std_srvs::Empty::Response& res)
  {
    ROS_INFO("Call to Capture Log Data");
    for(int index = 0; index<desired_robot_state_.size(); index++)
      {
//        base_command_pub_.publish(base_command_pose_[index]);
//        base_actual_pub_.publish(base_actual_pose_[index]);
        leg_state_pub_.publish(leg_states_[index]);
        joint_command_pub_.publish(joint_command_[index]);
        joint_actual_pub_.publish(joint_actual_[index]);
        contact_desired_pub_.publish(foot_desired_contact_[index]);
        leg_phase_pub_.publish(leg_phases_[index]);
        desired_robot_state_pub_.publish(desired_robot_state_[index]);
        actual_robot_state_pub_.publish(actual_robot_state_[index]);
        ros::Duration(0.001).sleep();
      }
    return true;
  }

}



PLUGINLIB_EXPORT_CLASS(balance_controller::RosBalanceController, controller_interface::ControllerBase)
