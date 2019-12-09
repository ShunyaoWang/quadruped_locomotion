#include "balance_controller/ros_controler/configure_change_controller.hpp"

#include "quadruped_model/quadrupedkinematics.h"
#include "ros/ros.h"

namespace balance_controller{
configure_change_controller::configure_change_controller()
{
    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
    limbs_.push_back(free_gait::LimbEnum::RH_LEG);
    branches_.push_back(free_gait::BranchEnum::BASE);
    branches_.push_back(free_gait::BranchEnum::LF_LEG);
    branches_.push_back(free_gait::BranchEnum::RF_LEG);
    branches_.push_back(free_gait::BranchEnum::RH_LEG);
    branches_.push_back(free_gait::BranchEnum::LH_LEG);
    robot_state_.reset(new free_gait::State);
    robot_state_->initialize(limbs_, branches_);
    log_length_ = 100000;
    times_.resize(3);
    values_lf_.resize(3);
    values_rf_.resize(3);
    values_rh_.resize(3);
    values_lh_.resize(3);

    for (auto limbs : limbs_) {
        is_legmode_[limbs] = false;
        is_footstep_[limbs] = false;
        is_cartisian_motion_[limbs] = false;
        limbs_state_[limbs].reset(new StateSwitcher);
        limbs_state_.at(limbs)->initialize(0);
        limbs_last_state_[limbs].reset(new StateSwitcher);
        limbs_last_state_.at(limbs)->initialize(0);
        limbs_desired_state_[limbs].reset(new StateSwitcher);
        limbs_desired_state_.at(limbs)->initialize(0);
        t_sw0_[limbs] = ros::Time::now();
        t_st0_[limbs] = ros::Time::now();
        st_phase_[limbs] = 0;
        sw_phase_[limbs] = 0;
        sw_flag_[limbs] = false;
        st_flag_[limbs] = false;
        foot_positions[limbs] = Vector(0, 0, 1);
        foot_velocities[limbs] = Vector(0, 0, 0);
        foot_accelerations[limbs] = Vector(0, 0, 0);
        footholdsInSupport[limbs].setZero();
    }
    dt_ = 0.0;
    l1 = 0.308;
    l2 = 0.308;
    configure_time_start_flag_ = false;

}//configure_change_controller

configure_change_controller::~configure_change_controller()
{
    base_command_sub_.shutdown();
}//~configure_change_controller

bool configure_change_controller::init(hardware_interface::RobotStateInterface *hardware, ros::NodeHandle &nodehandle)
{
    ROS_INFO("Initializing Balance Position Controller");

    urdf::Model urdf;
    if (!urdf.initParam("/robot_description"))
    {
        ROS_ERROR("failed to prase urdf file");
        return false;
    }

    std::string param_name = "joints";
    if(!nodehandle.getParam(param_name, joint_names_))
    {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nodehandle.getNamespace() << ").");
        return false;
    }

    if(!nodehandle.getParam("ignore_contact_sensor", ignore_contact_sensor))
    {
        ROS_ERROR("Can't find parameter of 'ignore_contact_sensor'");
        return false;
    }

    n_joints_ = joint_names_.size();
    if(n_joints_ == 0 )
    {
        ROS_ERROR_STREAM("List of joint names is empty.");
        return false;
    }

    for (unsigned int i = 0; i < n_joints_; i++) {
        try {
            joints_.push_back(hardware->joint_position_interfaces_.getHandle(joint_names_[i]));
            ROS_INFO("Get %s Handle", joint_names_[i].c_str());
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("Exception Thrown :" << ex.what());
            return false;
        }

        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
        if (!joint_urdf)
        {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
            return false;
        }
        joint_urdfs_.push_back(joint_urdf);
    }
    robot_state_handle_ = hardware->getHandle("base_controller");
    ROS_INFO("Balance Position Controller is to be initialized");
    for (unsigned int i = 0; i < 12; i++) {
        robot_state_handle_.getJointEffortWrite()[i] = 0;
        robot_state_handle_.motor_status_word_[i] = 0;
    }
    for (unsigned int i = 0; i < 4; i++) {
        robot_state_handle_.foot_contact_[i] = 1;
    }
    //    base_command_sub_ = nodehandle.subscribe<free_gait_msgs::RobotState>("/desired_robot_state", 1, &configure_change_controller::baseCommandCallback, this);
    base_command_sub_ = nodehandle.subscribe("/desired_robot_state", 1, &configure_change_controller::baseCommandCallback, this);

    ROS_INFO("configure change Controller initialized");

    log_data_srv_  = nodehandle.advertiseService("/capture_log_data_configure", &configure_change_controller::logDataCapture, this);
    leg_state_pub_ = nodehandle.advertise<std_msgs::Int8MultiArray>("/log/leg_state_kp", log_length_);
    contact_desired_pub_ = nodehandle.advertise<sim_assiants::FootContacts>("/log/desired_foot_contact_kp", log_length_);
    leg_phase_pub_ = nodehandle.advertise<std_msgs::Float64MultiArray>("/log/leg_phase_kp", log_length_);
    desired_robot_state_pub_ = nodehandle.advertise<free_gait_msgs::RobotState>("/log/desired_robot_state_kp", log_length_);
    actual_robot_state_pub_ = nodehandle.advertise<free_gait_msgs::RobotState>("/log/actual_robot_state_kp", log_length_);    
    x_configure_sub_ = nodehandle.subscribe("/x_configure_change", 1, &configure_change_controller::change_to_X_configure_CB, this);
    left_configure_sub_ = nodehandle.subscribe("/left_configure_change", 1, &configure_change_controller::change_to_left_configure_CB, this);
    right_configure_sub_ = nodehandle.subscribe("/right_configure_change", 1, &configure_change_controller::change_to_right_configure_CB, this);
    anti_x_configure_sub_ = nodehandle.subscribe("/anti_x_configure_change", 1, &configure_change_controller::change_to_anti_x_configure_CB, this);
}

void configure_change_controller::update(const ros::Time &time, const ros::Duration &period)
{
    sensor_msgs::JointState joint_command, joint_actual;
    joint_command.effort.resize(12);
    joint_command.position.resize(12);
    joint_command.name.resize(12);
    joint_actual.name.resize(12);
    joint_actual.position.resize(12);
    joint_actual.velocity.resize(12);
    joint_actual.effort.resize(12);

    std::vector<bool> leg_state;
    std_msgs::Float64MultiArray leg_phase;
    leg_phase.data.resize(8);
    leg_state.resize(4);

//    free_gait_msgs::FootInBase foot_in_base;
//    foot_in_base.name.resize(4);
//    foot_in_base.position_in_base.resize(4);
//    foot_in_base.position_in_world.resize(4);

//    foot_in_base.name[0] = "LF_LEG";
//    foot_in_base.name[1] = "RF_LEG";
//    foot_in_base.name[2] = "RH_LEG";
//    foot_in_base.name[3] = "LH_LEG";

    for (unsigned int i = 0; i < 12; i++) {
        joint_actual.position[i] = robot_state_handle_.getJointPositionRead()[i];//actual joint
        joint_actual.name[i] = joint_names_.at(i);
        joint_command.name[i] = joint_names_.at(i);
        joint_command.position[i] = joint_actual.position[i];
//        ROS_INFO_STREAM("joint_actual_position of joint " << i << " is " << joint_actual.position[i]);
    }
//    ROS_INFO_STREAM("joint_actual_position of joint 3 is " << joint_actual.position[2]);

    robot_state_->setPositionWorldToBaseInWorldFrame(base_desired_position_);//target? is target!
    robot_state_->setOrientationBaseToWorld(base_desired_rotation_);
    robot_state_->setLinearVelocityBaseInWorldFrame(base_desired_linear_velocity_);
    robot_state_->setAngularVelocityBaseInBaseFrame(base_desired_angular_velocity_);
    //    robot_state_->setCurrentLimbJoints(all_joint_positions_);

    Pose current_base_pose = Pose(Position(robot_state_handle_.getPosition()[0],
                                           robot_state_handle_.getPosition()[1],
                                           robot_state_handle_.getPosition()[2]),
                                  RotationQuaternion(robot_state_handle_.getOrientation()[0],
                                                     robot_state_handle_.getOrientation()[1],
                                                     robot_state_handle_.getOrientation()[2],
                                                     robot_state_handle_.getOrientation()[3]));
    Pose desired_pose = Pose(base_desired_position_, base_desired_rotation_);
    robot_state_->setPoseBaseToWorld(desired_pose);
    robot_state_->setBaseStateFromFeedback(LinearVelocity(robot_state_handle_.getLinearVelocity()[0],
                                                          robot_state_handle_.getLinearVelocity()[1],
                                                          robot_state_handle_.getLinearVelocity()[2]),
                                           LocalAngularVelocity(robot_state_handle_.getAngularVelocity()[0],
                                                                robot_state_handle_.getAngularVelocity()[1],
                                                                robot_state_handle_.getAngularVelocity()[2]));

    free_gait::JointPositionsLeg joint_positions;

    if(configure_time_start_flag_)
    {
        trajectory_lf_.evaluate(values_joint_1, dt_);
        trajectory_rf_.evaluate(values_joint_2, dt_);
        trajectory_rh_.evaluate(values_joint_3, dt_);
        trajectory_lh_.evaluate(values_joint_4, dt_);
        double values_joint_1_3, values_joint_2_3, values_joint_3_3, values_joint_4_3;
        values_joint_1_3 = - 2 * values_joint_1;
        values_joint_2_3 = - 2 * values_joint_2;
        values_joint_3_3 = - 2 * values_joint_3;
        values_joint_4_3 = - 2 * values_joint_4;
        dt_ = dt_ + 0.001;

        joint_command.position[0] = 0;
        joint_command.position[1] = values_joint_1;
        joint_command.position[2] = values_joint_1_3;

        joint_command.position[3] = 0;
        joint_command.position[4] = values_joint_2;
        joint_command.position[5] = values_joint_2_3;

        joint_command.position[6] = 0;
        joint_command.position[7] = values_joint_3;
        joint_command.position[8] = values_joint_3_3;

        joint_command.position[9]  = 0;
        joint_command.position[10] = values_joint_4;
        joint_command.position[11] = values_joint_4_3;
//        ROS_INFO_STREAM("In time " << dt_ );
    }else {
        ROS_INFO_STREAM("in the default loop");
        for (unsigned int i = 0; i < 12; i++) {
            joint_command.position[i] = 0;
        }
        ROS_INFO_STREAM("joint_command.position[i] is " << joint_command.position[11]);
    }

//    ROS_INFO_STREAM("In time " << dt_ << " joint_command is " << joint_command.position[2]);
    free_gait::JointPositionsLeg LF_leg_joints, RF_leg_joints, RH_leg_joints, LH_leg_joints;
    LF_leg_joints(0) = joint_command.position[0];
    LF_leg_joints(1) = joint_command.position[1];
    LF_leg_joints(2) = joint_command.position[2];

    RF_leg_joints(0) = joint_command.position[3];
    RF_leg_joints(1) = joint_command.position[4];
    RF_leg_joints(2) = joint_command.position[5];

    RH_leg_joints(0) = joint_command.position[6];
    RH_leg_joints(1) = joint_command.position[7];
    RH_leg_joints(2) = joint_command.position[8];

    LH_leg_joints(0) = joint_command.position[9];
    LH_leg_joints(1) = joint_command.position[10];
    LH_leg_joints(2) = joint_command.position[11];

    robot_state_->setJointPositionsForLimb(free_gait::LimbEnum::LF_LEG, LF_leg_joints);
    robot_state_->setJointPositionsForLimb(free_gait::LimbEnum::RF_LEG, RF_leg_joints);
    robot_state_->setJointPositionsForLimb(free_gait::LimbEnum::RH_LEG, RH_leg_joints);
    robot_state_->setJointPositionsForLimb(free_gait::LimbEnum::LH_LEG, LH_leg_joints);

    for (unsigned int joint_num = 0; joint_num < 12; joint_num++) {
        robot_state_handle_.mode_of_joint_[joint_num] = 1;
        joints_[joint_num].setCommand(joint_command.position[joint_num]);
        all_joint_positions_(joint_num) = joint_command.position[joint_num];
    }

    robot_state_->setCurrentLimbJoints(all_joint_positions_);

    if(actual_robot_state_.size()<log_length_)
    {

        free_gait_msgs::RobotState desired_robot_state, actual_robot_state;
        desired_robot_state.lf_target.target_position.resize(1);
        desired_robot_state.lf_target.target_velocity.resize(1);
        desired_robot_state.lf_target.target_force.resize(1);

        actual_robot_state.lf_target.target_position.resize(1);
        actual_robot_state.lf_target.target_velocity.resize(1);
        actual_robot_state.lf_target.target_force.resize(1);

        desired_robot_state.rf_target.target_position.resize(1);
        desired_robot_state.rf_target.target_velocity.resize(1);
        desired_robot_state.rf_target.target_force.resize(1);

        actual_robot_state.rf_target.target_position.resize(1);
        actual_robot_state.rf_target.target_velocity.resize(1);
        actual_robot_state.rf_target.target_force.resize(1);

        desired_robot_state.rh_target.target_position.resize(1);
        desired_robot_state.rh_target.target_velocity.resize(1);
        desired_robot_state.rh_target.target_force.resize(1);

        actual_robot_state.rh_target.target_position.resize(1);
        actual_robot_state.rh_target.target_velocity.resize(1);
        actual_robot_state.rh_target.target_force.resize(1);

        desired_robot_state.lh_target.target_position.resize(1);
        desired_robot_state.lh_target.target_velocity.resize(1);
        desired_robot_state.lh_target.target_force.resize(1);

        actual_robot_state.lh_target.target_position.resize(1);
        actual_robot_state.lh_target.target_velocity.resize(1);
        actual_robot_state.lh_target.target_force.resize(1);

        std::vector<sensor_msgs::JointState> joint_states_leg, joint_commands_leg;
        joint_states_leg.resize(4);
        joint_commands_leg.resize(4);
        for (unsigned int leg_number = 0; leg_number < 4; leg_number++) {
            for (unsigned int joint_number = 0; joint_number < 3; joint_number++) {
                joint_states_leg[leg_number].position.push_back(joint_actual.position[3 * leg_number + joint_number]);
                joint_commands_leg[leg_number].position.push_back(joint_command.position[3 * leg_number + joint_number]);
                joint_commands_leg[leg_number].effort.push_back(dt_);
            }
        }
        actual_robot_state.lf_leg_mode.support_leg = leg_state[0];
        actual_robot_state.rf_leg_mode.support_leg = leg_state[1];
        actual_robot_state.rh_leg_mode.support_leg = leg_state[2];
        actual_robot_state.lh_leg_mode.support_leg = leg_state[3];

        actual_robot_state.lf_leg_joints = joint_states_leg[0];
        actual_robot_state.rf_leg_joints = joint_states_leg[1];
        actual_robot_state.rh_leg_joints = joint_states_leg[2];
        actual_robot_state.lh_leg_joints = joint_states_leg[3];

        desired_robot_state.lf_leg_joints = joint_commands_leg[0];
        desired_robot_state.rf_leg_joints = joint_commands_leg[1];
        desired_robot_state.rh_leg_joints = joint_commands_leg[2];
        desired_robot_state.lh_leg_joints = joint_commands_leg[3];


        geometry_msgs::Pose desired_pose, actual_pose;
        geometry_msgs::Twist desired_twist, actual_twist;
        nav_msgs::Odometry desire_odom, actual_odom;
        kindr_ros::convertToRosGeometryMsg(base_desired_position_, desired_pose.position);
        kindr_ros::convertToRosGeometryMsg(base_desired_rotation_, desired_pose.orientation);
        kindr_ros::convertToRosGeometryMsg(base_desired_linear_velocity_, desired_twist.linear);
        kindr_ros::convertToRosGeometryMsg(base_desired_angular_velocity_, desired_twist.angular);
        kindr_ros::convertToRosGeometryMsg(current_base_pose, actual_pose);
        actual_twist.linear.x = robot_state_handle_.getLinearVelocity()[0];
        actual_twist.linear.y = robot_state_handle_.getLinearVelocity()[1];
        actual_twist.linear.z = robot_state_handle_.getLinearVelocity()[2];
        actual_twist.angular.x = robot_state_handle_.getAngularVelocity()[0];
        actual_twist.angular.y = robot_state_handle_.getAngularVelocity()[1];
        actual_twist.angular.z = robot_state_handle_.getAngularVelocity()[2];
        desire_odom.pose.pose = desired_pose;
        desire_odom.twist.twist = desired_twist;
        actual_odom.pose.pose = actual_pose;
        actual_odom.twist.twist = actual_twist;
//        foot_in_base_.push_back(foot_in_base);
        joint_command_.push_back(joint_command);
        joint_actual_.push_back(joint_actual);
        leg_phases_.push_back(leg_phase);

        std::vector<free_gait::Force> real_contact_forces;
        real_contact_forces.resize(4);
        desired_robot_state.base_pose = desire_odom;

        kindr_ros::convertToRosGeometryMsg(Position(robot_state_->getTargetFootPositionInBaseForLimb(free_gait::LimbEnum::LF_LEG)),
                                           desired_robot_state.lf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(Position(robot_state_->getTargetFootPositionInBaseForLimb(free_gait::LimbEnum::RF_LEG)),
                                           desired_robot_state.rf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(Position(robot_state_->getTargetFootPositionInBaseForLimb(free_gait::LimbEnum::RH_LEG)),
                                           desired_robot_state.rh_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(Position(robot_state_->getTargetFootPositionInBaseForLimb(free_gait::LimbEnum::LH_LEG)),
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
        kindr_ros::convertToRosGeometryMsg(robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LF_LEG),
                                           actual_robot_state.lf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::RF_LEG),
                                           actual_robot_state.rf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::RH_LEG),
                                           actual_robot_state.rh_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LH_LEG),
                                           actual_robot_state.lh_target.target_position[0].point);

        kindr_ros::convertToRosGeometryMsg(robot_state_->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LF_LEG),
                                           actual_robot_state.lf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(robot_state_->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RF_LEG),
                                           actual_robot_state.rf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(robot_state_->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RH_LEG),
                                           actual_robot_state.rh_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(robot_state_->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LH_LEG),
                                           actual_robot_state.lh_target.target_velocity[0].vector);

        actual_robot_state.lf_leg_joints = joint_states_leg[0];
        actual_robot_state.rf_leg_joints = joint_states_leg[1];
        actual_robot_state.rh_leg_joints = joint_states_leg[2];
        actual_robot_state.lh_leg_joints = joint_states_leg[3];

        actual_robot_state_.push_back(actual_robot_state);
    }
}

void configure_change_controller::baseCommandCallback(const free_gait_msgs::RobotStateConstPtr &robot_state_msgs)
{

    base_desired_position_ = Position(robot_state_msgs->base_pose.pose.pose.position.x,
                                      robot_state_msgs->base_pose.pose.pose.position.y,
                                      robot_state_msgs->base_pose.pose.pose.position.z);
    base_desired_rotation_ = RotationQuaternion(robot_state_msgs->base_pose.pose.pose.orientation.w,
                                                robot_state_msgs->base_pose.pose.pose.orientation.x,
                                                robot_state_msgs->base_pose.pose.pose.orientation.y,
                                                robot_state_msgs->base_pose.pose.pose.orientation.z);
    base_desired_linear_velocity_ = LinearVelocity(robot_state_msgs->base_pose.twist.twist.linear.x,
                                                   robot_state_msgs->base_pose.twist.twist.linear.y,
                                                   robot_state_msgs->base_pose.twist.twist.linear.z);
    base_desired_angular_velocity_ = LocalAngularVelocity(robot_state_msgs->base_pose.twist.twist.angular.x,
                                                          robot_state_msgs->base_pose.twist.twist.angular.y,
                                                          robot_state_msgs->base_pose.twist.twist.angular.z);

    Position foot_position, foot_velocity, foot_acceleration;

    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->lf_target.target_position[0].point, foot_position);
    foot_positions[free_gait::LimbEnum::LF_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->lf_target.target_velocity[0].vector, foot_velocity);
    foot_velocities[free_gait::LimbEnum::LF_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->lf_target.target_acceleration[0].vector, foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::LF_LEG] = Vector(foot_acceleration.toImplementation());

    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->rf_target.target_position[0].point, foot_position);
    foot_positions[free_gait::LimbEnum::RF_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->rf_target.target_velocity[0].vector, foot_velocity);
    foot_velocities[free_gait::LimbEnum::RF_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->rf_target.target_acceleration[0].vector, foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::RF_LEG] = Vector(foot_acceleration.toImplementation());

    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->rh_target.target_position[0].point, foot_position);
    foot_positions[free_gait::LimbEnum::RH_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->rh_target.target_velocity[0].vector, foot_velocity);
    foot_velocities[free_gait::LimbEnum::RH_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->rh_target.target_acceleration[0].vector, foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::RH_LEG] = Vector(foot_acceleration.toImplementation());

    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->lh_target.target_position[0].point, foot_position);
    foot_positions[free_gait::LimbEnum::LH_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->lh_target.target_velocity[0].vector, foot_velocity);
    foot_velocities[free_gait::LimbEnum::LH_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->lh_target.target_acceleration[0].vector, foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::LH_LEG] = Vector(foot_acceleration.toImplementation());

    robot_state_->setPositionWorldToBaseInWorldFrame(base_desired_position_);
    robot_state_->setOrientationBaseToWorld(RotationQuaternion(base_desired_rotation_));

    robot_state_->setLinearVelocityBaseInWorldFrame(base_desired_linear_velocity_);
    robot_state_->setAngularVelocityBaseInBaseFrame(base_desired_angular_velocity_);

    if(robot_state_msgs->lf_leg_mode.name == "joint")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
    }
    if(robot_state_msgs->lf_leg_mode.name == "leg_mode")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = true;
    }
    if(robot_state_msgs->lf_leg_mode.name == "cartesian")//foottarget
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
    }
    if(robot_state_msgs->lf_leg_mode.name == "footstep")//is footstep
    {
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = true;
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
    }

    if(robot_state_msgs->rf_leg_mode.name == "joint")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
    }
    if(robot_state_msgs->rf_leg_mode.name == "leg_mode")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = true;
    }
    if(robot_state_msgs->rf_leg_mode.name == "cartesian")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
    }
    if(robot_state_msgs->rf_leg_mode.name == "footstep")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = true;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
    }

    if(robot_state_msgs->rh_leg_mode.name == "joint")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
    }
    if(robot_state_msgs->rh_leg_mode.name == "leg_mode")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = true;
    }
    if(robot_state_msgs->rh_leg_mode.name == "cartesian")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
    }
    if(robot_state_msgs->rh_leg_mode.name == "footstep")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = true;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
    }

    if(robot_state_msgs->lh_leg_mode.name == "joint")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
    }
    if(robot_state_msgs->lh_leg_mode.name == "leg_mode")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = true;
    }
    if(robot_state_msgs->lh_leg_mode.name == "cartesian")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
    }
    if(robot_state_msgs->lh_leg_mode.name == "footstep")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = true;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
    }

    if(robot_state_msgs->lf_leg_mode.support_leg)
    {
        robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, true);
        robot_state_->setSurfaceNormal(free_gait::LimbEnum::LF_LEG, Vector(robot_state_msgs->lf_leg_mode.surface_normal.vector.x,
                                                                           robot_state_msgs->lf_leg_mode.surface_normal.vector.y,
                                                                           robot_state_msgs->lf_leg_mode.surface_normal.vector.z));
        limbs_desired_state_.at(free_gait::LimbEnum::LF_LEG)->setState(StateSwitcher::States::StanceNormal);
        if(st_flag_.at(free_gait::LimbEnum::LF_LEG)){
            st_flag_.at(free_gait::LimbEnum::LF_LEG) = false;
            t_st0_.at(free_gait::LimbEnum::LF_LEG) = ros::Time::now();
        }
        sw_flag_.at(free_gait::LimbEnum::LF_LEG) = false;
        st_phase_.at(free_gait::LimbEnum::LF_LEG) = robot_state_msgs->lf_leg_mode.phase;
        sw_phase_.at(free_gait::LimbEnum::LF_LEG) = 0;
    }else {
        robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, false);
        robot_state_->setSurfaceNormal(free_gait::LimbEnum::LF_LEG, Vector(0,0,1));
        limbs_desired_state_.at(free_gait::LimbEnum::LF_LEG)->setState(StateSwitcher::States::SwingNormal);
        if(!sw_flag_.at(free_gait::LimbEnum::LF_LEG))
        {
            t_sw0_.at(free_gait::LimbEnum::LF_LEG) = ros::Time::now();
            sw_flag_.at(free_gait::LimbEnum::LF_LEG) = true;
        }
        st_flag_.at(free_gait::LimbEnum::LF_LEG) = true;
        sw_phase_.at(free_gait::LimbEnum::LF_LEG) = robot_state_msgs->lf_leg_mode.phase;
        st_phase_.at(free_gait::LimbEnum::LF_LEG) = 0;
    };

    if(robot_state_msgs->rf_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(1), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(1),
                                       Vector(robot_state_msgs->rf_leg_mode.surface_normal.vector.x,
                                              robot_state_msgs->rf_leg_mode.surface_normal.vector.y,
                                              robot_state_msgs->rf_leg_mode.surface_normal.vector.z));
        limbs_desired_state_.at(free_gait::LimbEnum::RF_LEG)->setState(StateSwitcher::States::StanceNormal);
        if(st_flag_.at(free_gait::LimbEnum::RF_LEG)){
            st_flag_.at(free_gait::LimbEnum::RF_LEG) = false;
            t_st0_.at(free_gait::LimbEnum::RF_LEG)= ros::Time::now();
        }
        sw_flag_.at(free_gait::LimbEnum::RF_LEG) = false;
        st_phase_.at(free_gait::LimbEnum::RF_LEG) = robot_state_msgs->rf_leg_mode.phase;
        sw_phase_.at(free_gait::LimbEnum::RF_LEG) = 0;
    } else {
        //        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(1), false);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(1),Vector(0,0,1));
        limbs_desired_state_.at(free_gait::LimbEnum::RF_LEG)->setState(StateSwitcher::States::SwingNormal);
        if(!sw_flag_.at(free_gait::LimbEnum::RF_LEG)){
            t_sw0_.at(free_gait::LimbEnum::RF_LEG) = ros::Time::now();
            sw_flag_.at(free_gait::LimbEnum::RF_LEG) = true;
        }
        st_flag_.at(free_gait::LimbEnum::RF_LEG) = true;
        sw_phase_.at(free_gait::LimbEnum::RF_LEG) = robot_state_msgs->rf_leg_mode.phase;
        st_phase_.at(free_gait::LimbEnum::RF_LEG) = 0;

    };
    if(robot_state_msgs->rh_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(2), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(2),
                                       Vector(robot_state_msgs->rh_leg_mode.surface_normal.vector.x,
                                              robot_state_msgs->rh_leg_mode.surface_normal.vector.y,
                                              robot_state_msgs->rh_leg_mode.surface_normal.vector.z));
        limbs_desired_state_.at(free_gait::LimbEnum::RH_LEG)->setState(StateSwitcher::States::StanceNormal);
        if(st_flag_.at(free_gait::LimbEnum::RH_LEG)){
            st_flag_.at(free_gait::LimbEnum::RH_LEG) = false;
            t_st0_.at(free_gait::LimbEnum::RH_LEG)= ros::Time::now();
        }
        sw_flag_.at(free_gait::LimbEnum::RH_LEG) = false;
        st_phase_.at(free_gait::LimbEnum::RH_LEG) = robot_state_msgs->rh_leg_mode.phase;
        sw_phase_.at(free_gait::LimbEnum::RH_LEG) = 0;
    } else {
        //        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(2), false);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(2),Vector(0,0,1));
        limbs_desired_state_.at(free_gait::LimbEnum::RH_LEG)->setState(StateSwitcher::States::SwingNormal);
        if(!sw_flag_.at(free_gait::LimbEnum::RH_LEG)){
            t_sw0_.at(free_gait::LimbEnum::RH_LEG) = ros::Time::now();
            sw_flag_.at(free_gait::LimbEnum::RH_LEG) = true;
        }
        st_flag_.at(free_gait::LimbEnum::RH_LEG) = true;
        sw_phase_.at(free_gait::LimbEnum::RH_LEG) = robot_state_msgs->rh_leg_mode.phase;
        st_phase_.at(free_gait::LimbEnum::RH_LEG) = 0;

    };
    if(robot_state_msgs->lh_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(3), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(3),
                                       Vector(robot_state_msgs->lh_leg_mode.surface_normal.vector.x,
                                              robot_state_msgs->lh_leg_mode.surface_normal.vector.y,
                                              robot_state_msgs->lh_leg_mode.surface_normal.vector.z));
        limbs_desired_state_.at(free_gait::LimbEnum::LH_LEG)->setState(StateSwitcher::States::StanceNormal);
        if(st_flag_.at(free_gait::LimbEnum::LH_LEG)){
            st_flag_.at(free_gait::LimbEnum::LH_LEG) = false;
            t_st0_.at(free_gait::LimbEnum::LH_LEG)= ros::Time::now();
        }
        sw_flag_.at(free_gait::LimbEnum::LH_LEG) = false;
        st_phase_.at(free_gait::LimbEnum::LH_LEG) = robot_state_msgs->lh_leg_mode.phase;
        sw_phase_.at(free_gait::LimbEnum::LH_LEG) = 0;
    } else {
        //        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(3), false);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(3),Vector(0,0,1));
        limbs_desired_state_.at(free_gait::LimbEnum::LH_LEG)->setState(StateSwitcher::States::SwingNormal);
        if(!sw_flag_.at(free_gait::LimbEnum::LH_LEG)){
            t_sw0_.at(free_gait::LimbEnum::LH_LEG) = ros::Time::now();
            sw_flag_.at(free_gait::LimbEnum::LH_LEG) = true;
        }
        st_flag_.at(free_gait::LimbEnum::LH_LEG) = true;
        sw_phase_.at(free_gait::LimbEnum::LH_LEG) = robot_state_msgs->lh_leg_mode.phase;
        st_phase_.at(free_gait::LimbEnum::LH_LEG) = 0;

    };
}

void configure_change_controller::starting(const ros::Time &time)
{
    ROS_INFO("staring configure change controller");
    foot_desired_contact_.clear();
    leg_phases_.clear();
    desired_robot_state_.clear();
    actual_robot_state_.clear();
    log_time_.clear();
    for (unsigned int i = 0; i < 12; i++) {
        all_joint_positions_(i) = robot_state_handle_.getJointPositionRead()[i];
        joints_[i].setCommand(all_joint_positions_(i));
    }
    robot_state_->setCurrentLimbJoints(all_joint_positions_);//current joint
    std::cout << "success started the configure controller" << std::endl;
}

void configure_change_controller::stopping(const ros::Time &time)
{
    std::cout << "the length of data is " << actual_robot_state_.size() << std::endl;
    ROS_INFO("stop configure change controller");
    configure_time_start_flag_ = false;
}
bool configure_change_controller::logDataCapture(std_srvs::Empty::Request& req,
                                                 std_srvs::Empty::Response& res)
{
    ROS_INFO("Call to Capture Log Data");
    for(int index = 0; index<desired_robot_state_.size(); index++)
    {
        //        leg_state_pub_.publish(leg_states_[index]);
        //        joint_command_pub_.publish(joint_command_[index]);
        //        joint_actual_pub_.publish(joint_actual_[index]);
        contact_desired_pub_.publish(foot_desired_contact_[index]);
        leg_phase_pub_.publish(leg_phases_[index]);
        desired_robot_state_pub_.publish(desired_robot_state_[index]);
        actual_robot_state_pub_.publish(actual_robot_state_[index]);
        //        foot_in_base_pub_.publish(foot_in_base_[index]);
        ros::Duration(0.001).sleep();
    }
    return true;
}



void configure_change_controller::change_to_X_configure_CB(const std_msgs::BoolConstPtr &X_Configure)
{
    ROS_INFO_STREAM("in the x configure");
    dt_ = 0;
    trajectory_lf_.clear();
    trajectory_rf_.clear();
    trajectory_rh_.clear();
    trajectory_lh_.clear();
    for (unsigned int i = 0; i < 12; i++) {
        all_joint_positions_(i) = robot_state_handle_.getJointPositionRead()[i];
    }
    robot_state_->setCurrentLimbJoints(all_joint_positions_);//current joint

    times_[0] = 0.0;
    times_[1] = 1.5;
    times_[2] = 3.0;

    values_lf_[0] = all_joint_positions_(1);
    values_lf_[1] = (all_joint_positions_(1) + 0.8) / 2;
    values_lf_[2] = 0.8;
    trajectory_lf_.fitCurve(times_,values_lf_);

    values_rf_[0] = all_joint_positions_(4);
    values_rf_[1] = (all_joint_positions_(4) - 0.8) / 2;
    values_rf_[2] = -0.8;
    trajectory_rf_.fitCurve(times_,values_rf_);

    values_rh_[0] = all_joint_positions_(7);
    values_rh_[1] = (all_joint_positions_(7) + 0.8) / 2;
    values_rh_[2] = 0.8;
    trajectory_rh_.fitCurve(times_,values_rh_);

    values_lh_[0] = all_joint_positions_(10);
    values_lh_[1] = (all_joint_positions_(10) - 0.8) / 2;
    values_lh_[2] = -0.8;
    trajectory_lh_.fitCurve(times_,values_lh_);
    configure_time_start_flag_ = true;
    ROS_INFO_STREAM("in the x configure_end");
}

void configure_change_controller::change_to_left_configure_CB(const std_msgs::BoolConstPtr &left_Configure)
{
    ROS_INFO_STREAM("in the left configure");
    dt_ = 0;
    trajectory_lf_.clear();
    trajectory_rf_.clear();
    trajectory_rh_.clear();
    trajectory_lh_.clear();
    for (unsigned int i = 0; i < 12; i++) {
        all_joint_positions_(i) = robot_state_handle_.getJointPositionRead()[i];
    }
    robot_state_->setCurrentLimbJoints(all_joint_positions_);//current joint

    times_[0] = 0.0;
    times_[1] = 1.5;
    times_[2] = 3.0;

    values_lf_[0] = all_joint_positions_(1);
    values_joint_1 = values_lf_[0];
    values_lf_[1] = (all_joint_positions_(1) + 0.8) / 2;
    values_lf_[2] = 0.8;
    trajectory_lf_.fitCurve(times_,values_lf_);

    values_rf_[0] = all_joint_positions_(4);
    values_joint_2 = values_rf_[0];
    values_rf_[1] = (all_joint_positions_(4) - 0.8) / 2;
    values_rf_[2] = -0.8;
    trajectory_rf_.fitCurve(times_,values_rf_);

    values_rh_[0] = all_joint_positions_(7);
    values_joint_2 = values_rh_[0];
    values_rh_[1] = (all_joint_positions_(7) - 0.8) / 2;
    values_rh_[2] = - 0.8;
    trajectory_rh_.fitCurve(times_,values_rh_);

    values_lh_[0] = all_joint_positions_(10);
    values_joint_2 = values_lh_[0];
    values_lh_[1] = (all_joint_positions_(10) + 0.8) / 2;
    values_lh_[2] = 0.8;
    trajectory_lh_.fitCurve(times_,values_lh_);

    configure_time_start_flag_ = true;
    ROS_INFO_STREAM("in the left configure_end");
}

void configure_change_controller::change_to_right_configure_CB(const std_msgs::BoolConstPtr &right_Configure)
{
    ROS_INFO_STREAM("in the left configure");
    dt_ = 0;
    trajectory_lf_.clear();
    trajectory_rf_.clear();
    trajectory_rh_.clear();
    trajectory_lh_.clear();
    for (unsigned int i = 0; i < 12; i++) {
        all_joint_positions_(i) = robot_state_handle_.getJointPositionRead()[i];
    }
    robot_state_->setCurrentLimbJoints(all_joint_positions_);//current joint

    times_[0] = 0.0;
    times_[1] = 1.5;
    times_[2] = 3.0;

    values_lf_[0] = all_joint_positions_(1);
    values_joint_1 = values_lf_[0];
    values_lf_[2] = -0.8;
    values_lf_[1] = (all_joint_positions_(1) + values_lf_[2]) / 2;
    trajectory_lf_.fitCurve(times_,values_lf_);

    values_rf_[0] = all_joint_positions_(4);
    values_joint_2 = values_rf_[0];
    values_rf_[2] = 0.8;
    values_rf_[1] = (all_joint_positions_(4) + values_rf_[2]) / 2;
    trajectory_rf_.fitCurve(times_,values_rf_);

    values_rh_[0] = all_joint_positions_(7);
    values_joint_2 = values_rh_[0];
    values_rh_[2] = 0.8;
    values_rh_[1] = (all_joint_positions_(7) + values_rh_[2]) / 2;

    trajectory_rh_.fitCurve(times_,values_rh_);

    values_lh_[0] = all_joint_positions_(10);
    values_joint_2 = values_lh_[0];
    values_lh_[2] = -0.8;
    values_lh_[1] = (all_joint_positions_(10) + values_lh_[2]) / 2;
    trajectory_lh_.fitCurve(times_,values_lh_);

    configure_time_start_flag_ = true;
    ROS_INFO_STREAM("in the right configure_end");
}

void configure_change_controller::change_to_anti_x_configure_CB(const std_msgs::BoolConstPtr &anti_x_Configure)
{
    ROS_INFO_STREAM("in the anti x configure");
    dt_ = 0;
    trajectory_lf_.clear();
    trajectory_rf_.clear();
    trajectory_rh_.clear();
    trajectory_lh_.clear();
    for (unsigned int i = 0; i < 12; i++) {
        all_joint_positions_(i) = robot_state_handle_.getJointPositionRead()[i];
    }
    robot_state_->setCurrentLimbJoints(all_joint_positions_);//current joint

    times_[0] = 0.0;
    times_[1] = 1.5;
    times_[2] = 3.0;

    values_lf_[0] = all_joint_positions_(1);
    values_joint_1 = values_lf_[0];
    values_lf_[2] = -0.8;
    values_lf_[1] = (all_joint_positions_(1) + values_lf_[2]) / 2;
    trajectory_lf_.fitCurve(times_,values_lf_);

    values_rf_[0] = all_joint_positions_(4);
    values_joint_2 = values_rf_[0];
    values_rf_[2] = 0.8;
    values_rf_[1] = (all_joint_positions_(4) + values_rf_[2]) / 2;
    trajectory_rf_.fitCurve(times_,values_rf_);

    values_rh_[0] = all_joint_positions_(7);
    values_joint_2 = values_rh_[0];
    values_rh_[2] = -0.8;
    values_rh_[1] = (all_joint_positions_(7) + values_rh_[2]) / 2;

    trajectory_rh_.fitCurve(times_,values_rh_);

    values_lh_[0] = all_joint_positions_(10);
    values_joint_2 = values_lh_[0];
    values_lh_[2] = 0.8;
    values_lh_[1] = (all_joint_positions_(10) + values_lh_[2]) / 2;
    trajectory_lh_.fitCurve(times_,values_lh_);

    configure_time_start_flag_ = true;
    ROS_INFO_STREAM("in the anti x configure_end");
}

}//namespace
PLUGINLIB_EXPORT_CLASS(balance_controller::configure_change_controller, controller_interface::ControllerBase)

