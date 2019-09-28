/*
 * StateRosPublisher.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_ros/StateRosPublisher.hpp>

// KDL
#include <kdl_parser/kdl_parser.hpp>

// Kindr
#include <kindr_ros/kindr_ros.hpp>

// STD
#include <string>
#include <vector>

#include <urdf/model.h>

namespace free_gait {

StateRosPublisher::StateRosPublisher(ros::NodeHandle& nodeHandle,
                                     AdapterBase& adapter)
    : nodeHandle_(nodeHandle),
      adapter_(adapter)
{
  tfPrefix_ = nodeHandle_.param("/free_gait/preview_tf_prefix", std::string(""));
  initializeRobotStatePublisher();
}

StateRosPublisher::~StateRosPublisher()
{
}

StateRosPublisher::StateRosPublisher(const StateRosPublisher& other) :
    nodeHandle_(other.nodeHandle_),
    tfPrefix_(other.tfPrefix_),
    adapter_(other.adapter_),
    tfBroadcaster_(other.tfBroadcaster_)
{
  if (other.robotStatePublisher_) {
    robotStatePublisher_.reset(
        new robot_state_publisher::RobotStatePublisher(*other.robotStatePublisher_));
  }
}

void StateRosPublisher::setTfPrefix(const std::string tfPrefix)
{
  tfPrefix_ = tfPrefix;
}

bool StateRosPublisher::initializeRobotStatePublisher()
{
  std::string robotDescriptionPath;
  if (nodeHandle_.hasParam("/free_gait/robot_description")) {
    nodeHandle_.getParam("/free_gait/robot_description", robotDescriptionPath);
  } else {
    ROS_ERROR("Did not find ROS parameter for robot description '/free_gait/robot_description'.");
    return false;
  }

  urdf::Model model;
//  if (!model.initParam(robotDescriptionPath)) return false;initFile
  if (!model.initFile(robotDescriptionPath)) return false;

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)){
    ROS_ERROR("Failed to extract KDL tree from XML robot description");
    return false;
  }

  robotStatePublisher_.reset(new robot_state_publisher::RobotStatePublisher(tree));

  robot_state_pub_ = nodeHandle_.advertise<free_gait_msgs::RobotState>("/desired_robot_state", 1);
  stance_marker_pub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("optimized_footholds",1);

  robot_state_.lf_leg_joints.position.resize(3);
  robot_state_.rf_leg_joints.position.resize(3);
  robot_state_.rh_leg_joints.position.resize(3);
  robot_state_.lh_leg_joints.position.resize(3);

  robot_state_.lf_target.target_position.resize(1);
  robot_state_.lf_target.target_velocity.resize(1);
  robot_state_.lf_target.target_acceleration.resize(1);

  robot_state_.rf_target.target_position.resize(1);
  robot_state_.rf_target.target_velocity.resize(1);
  robot_state_.rf_target.target_acceleration.resize(1);

  robot_state_.rh_target.target_position.resize(1);
  robot_state_.rh_target.target_velocity.resize(1);
  robot_state_.rh_target.target_acceleration.resize(1);

  robot_state_.lh_target.target_position.resize(1);
  robot_state_.lh_target.target_velocity.resize(1);
  robot_state_.lh_target.target_acceleration.resize(1);

  return true;
}

bool StateRosPublisher::publish(const State& state)
{
  const ros::Time time = ros::Time::now();

  // Publish joint states.
  std::vector<std::string> jointNames;

  state.getAllJointNames(jointNames);

  JointPositions jointPositions = state.getJointPositionFeedback();//.getJointPositions();

  if (jointNames.size() != jointPositions.vector().size()) {
    ROS_ERROR("Joint name vector and joint position are not of equal size!");
    return false;
  }
  std::map<std::string, double> jointPositionsMap;//joint name and position
  for (size_t i = 0; i < jointNames.size(); ++i) {
    jointPositionsMap[jointNames[i]] = jointPositions(i);
//    std::cout<<jointNames[i]<<" position is : "<<jointPositions(i)<<std::endl;
  }

  robotStatePublisher_->publishTransforms(jointPositionsMap, time, tfPrefix_);
  robotStatePublisher_->publishFixedTransforms(tfPrefix_);
//ROS_INFO("In ros state publisher");
  // Publish base position.
  geometry_msgs::TransformStamped tfTransform;
  tfTransform.header.stamp = time;
  tfTransform.header.frame_id = adapter_.getWorldFrameId();
  tfTransform.child_frame_id = tf::resolve(tfPrefix_, adapter_.getBaseFrameId());
  kindr_ros::convertToRosGeometryMsg(state.getPositionWorldToBaseInWorldFrame(), tfTransform.transform.translation);
  kindr_ros::convertToRosGeometryMsg(state.getOrientationBaseToWorld(), tfTransform.transform.rotation);
  tfBroadcaster_.sendTransform(tfTransform);

  // Publish frame transforms.
  std::vector<std::string> frameTransforms;
  //TODO(Shunyao) : what else tranform needed to be broadcasted?
//  adapter_.getAvailableFrameTransforms(frameTransforms);
  for (const auto& frameId : frameTransforms) {
    geometry_msgs::TransformStamped tfTransform;
    tfTransform.header.stamp = time;
    tfTransform.header.frame_id = adapter_.getWorldFrameId();
    tfTransform.child_frame_id = tf::resolve(tfPrefix_, frameId);
    kindr_ros::convertToRosGeometryMsg(adapter_.getFrameTransform(frameId), tfTransform.transform);
    tfBroadcaster_.sendTransform(tfTransform);
  }
  //! WSHY: set to publish robot state to the balance controller
//  ROS_INFO("=============================In ros state publisher : \n");
//  std::cout<<state<<std::endl;
  kindr_ros::convertToRosGeometryMsg(Pose(Position(state.getTargetPositionWorldToBaseInWorldFrame()),
                                          RotationQuaternion(state.getTargetOrientationBaseToWorld())),
                                     robot_state_.base_pose.pose.pose);
  std::cout<<state.getTargetOrientationBaseToWorld()<<std::endl;
  kindr_ros::convertToRosGeometryMsg(Twist(LinearVelocity(state.getTargetLinearVelocityBaseInWorldFrame()),
                                           LocalAngularVelocity(state.getTargetAngularVelocityBaseInBaseFrame())),
                                     robot_state_.base_pose.twist.twist);
//  ROS_INFO("In ros state publisher");
  if(state.isSupportLeg(LimbEnum::LF_LEG))
      {
        robot_state_.lf_leg_mode.support_leg = true;
        if(state.hasSurfaceNormal(LimbEnum::LF_LEG)){
            kindr_ros::convertToRosGeometryMsg(state.getSurfaceNormal(LimbEnum::LF_LEG),
                                               robot_state_.lf_leg_mode.surface_normal.vector);
          } else {
            kindr_ros::convertToRosGeometryMsg(Vector(0,0,1),
                                               robot_state_.lf_leg_mode.surface_normal.vector);
          }

        /****************
          * TODO(Shunyao) : Store Duration information in the state
          ****************/
//        ROS_INFO("In ros state publisher");
      }else {
        robot_state_.lf_leg_mode.support_leg = false;
//        ROS_INFO("In ros state publisher");
//        std::cout<<state.getJointPositionsForLimb(LimbEnum::LF_LEG)<<std::endl;
        robot_state_.lf_leg_joints.position[0] = state.getJointPositionsForLimb(LimbEnum::LF_LEG)(0);
        robot_state_.lf_leg_joints.position[1] = state.getJointPositionsForLimb(LimbEnum::LF_LEG)(1);
        robot_state_.lf_leg_joints.position[2] = state.getJointPositionsForLimb(LimbEnum::LF_LEG)(2);
        /****************
        * TODO(Shunyao) : velocities command
        ****************/
//        ROS_INFO("In ros state publisher");
    }
    if(state.isSupportLeg(LimbEnum::RF_LEG))
      {
        robot_state_.rf_leg_mode.support_leg = true;
        if(state.hasSurfaceNormal(LimbEnum::RF_LEG)){
            kindr_ros::convertToRosGeometryMsg(state.getSurfaceNormal(LimbEnum::RF_LEG),
                                               robot_state_.rf_leg_mode.surface_normal.vector);
          } else {
            kindr_ros::convertToRosGeometryMsg(Vector(0,0,1),
                                               robot_state_.rf_leg_mode.surface_normal.vector);
          }
      }else {
        robot_state_.rf_leg_mode.support_leg = false;
        robot_state_.rf_leg_joints.position[0] = state.getJointPositionsForLimb(LimbEnum::RF_LEG)(0);
        robot_state_.rf_leg_joints.position[1] = state.getJointPositionsForLimb(LimbEnum::RF_LEG)(1);
        robot_state_.rf_leg_joints.position[2] = state.getJointPositionsForLimb(LimbEnum::RF_LEG)(2);
    }
    if(state.isSupportLeg(LimbEnum::RH_LEG))
      {
        robot_state_.rh_leg_mode.support_leg = true;
        if(state.hasSurfaceNormal(LimbEnum::RH_LEG)){
            kindr_ros::convertToRosGeometryMsg(state.getSurfaceNormal(LimbEnum::RH_LEG),
                                               robot_state_.rh_leg_mode.surface_normal.vector);
          } else {
            kindr_ros::convertToRosGeometryMsg(Vector(0,0,1),
                                               robot_state_.rh_leg_mode.surface_normal.vector);
          }
      }else {
        robot_state_.rh_leg_mode.support_leg = false;
        robot_state_.rh_leg_joints.position[0] = state.getJointPositionsForLimb(LimbEnum::RH_LEG)(0);
        robot_state_.rh_leg_joints.position[1] = state.getJointPositionsForLimb(LimbEnum::RH_LEG)(1);
        robot_state_.rh_leg_joints.position[2] = state.getJointPositionsForLimb(LimbEnum::RH_LEG)(2);
    }
    if(state.isSupportLeg(LimbEnum::LH_LEG))
      {
        robot_state_.lh_leg_mode.support_leg = true;
        if(state.hasSurfaceNormal(LimbEnum::LH_LEG)){
            kindr_ros::convertToRosGeometryMsg(state.getSurfaceNormal(LimbEnum::LH_LEG),
                                               robot_state_.lh_leg_mode.surface_normal.vector);
          } else {
            kindr_ros::convertToRosGeometryMsg(Vector(0,0,1),
                                               robot_state_.lh_leg_mode.surface_normal.vector);
          }
      }else {
        robot_state_.lh_leg_mode.support_leg = false;
        robot_state_.lh_leg_joints.position[0] = state.getJointPositionsForLimb(LimbEnum::LH_LEG)(0);
        robot_state_.lh_leg_joints.position[1] = state.getJointPositionsForLimb(LimbEnum::LH_LEG)(1);
        robot_state_.lh_leg_joints.position[2] = state.getJointPositionsForLimb(LimbEnum::LH_LEG)(2);
    }
  robot_state_pub_.publish(robot_state_);
//  ROS_INFO("Publised robot state once");
  return true;
}

bool StateRosPublisher::publish(const State& state, const StepQueue& step_queue)
{
  const ros::Time time = ros::Time::now();

  // Publish joint states.
  std::vector<std::string> jointNames;

  state.getAllJointNames(jointNames);

  JointPositions jointPositions = state.getJointPositionFeedback();//.getJointPositions();

  if (jointNames.size() != jointPositions.vector().size()) {
    ROS_ERROR("Joint name vector and joint position are not of equal size!");
    return false;
  }
  std::map<std::string, double> jointPositionsMap;
  for (size_t i = 0; i < jointNames.size(); ++i) {
    jointPositionsMap[jointNames[i]] = jointPositions(i);
//    std::cout<<jointNames[i]<<" position is : "<<jointPositions(i)<<std::endl;
  }

  robotStatePublisher_->publishTransforms(jointPositionsMap, time, tfPrefix_);
  robotStatePublisher_->publishFixedTransforms(tfPrefix_);
//ROS_INFO("In ros state publisher");
  // Publish base position.
  geometry_msgs::TransformStamped tfTransform;
  tfTransform.header.stamp = time;
  tfTransform.header.frame_id = adapter_.getWorldFrameId();
  tfTransform.child_frame_id = tf::resolve(tfPrefix_, adapter_.getBaseFrameId());
  kindr_ros::convertToRosGeometryMsg(state.getPositionWorldToBaseInWorldFrame(), tfTransform.transform.translation);
  kindr_ros::convertToRosGeometryMsg(state.getOrientationBaseToWorld(), tfTransform.transform.rotation);
  tfBroadcaster_.sendTransform(tfTransform);

  // Publish frame transforms.
  std::vector<std::string> frameTransforms;
  //TODO(Shunyao) : what else tranform needed to be broadcasted?
//  adapter_.getAvailableFrameTransforms(frameTransforms);
  for (const auto& frameId : frameTransforms) {
    geometry_msgs::TransformStamped tfTransform;
    tfTransform.header.stamp = time;
    tfTransform.header.frame_id = adapter_.getWorldFrameId();
    tfTransform.child_frame_id = tf::resolve(tfPrefix_, frameId);
    kindr_ros::convertToRosGeometryMsg(adapter_.getFrameTransform(frameId), tfTransform.transform);
    tfBroadcaster_.sendTransform(tfTransform);
  }

  Stance stance_to_reach;
  std::string foot_frame;
  if(!step_queue.empty())
    {
      for(auto& leg_motion : step_queue.getCurrentStep().getLegMotions())
        {
          switch (leg_motion.second->getType()) {
            case free_gait::LegMotionBase::Type::Footstep:
              {
                Footstep foot_step = dynamic_cast<Footstep&>(*leg_motion.second);
                stance_to_reach[leg_motion.first] = foot_step.getTargetPosition();
                foot_frame = foot_step.getFrameId(free_gait::ControlLevel::Position);
                break;
              }
              switch (leg_motion.second->getType()) {
                case free_gait::LegMotionBase::Type::EndEffectorTarget:
                  {
                    EndEffectorTarget end_target = dynamic_cast<EndEffectorTarget&>(*leg_motion.second);
                    stance_to_reach[leg_motion.first] = end_target.getTargetPosition();
                    break;
                  }
                default:
                  break;

                }
            }
        }
    }
//  ROS_WARN_STREAM("Footholds To Reach is : "<<stance_to_reach<<std::endl);
  std_msgs::ColorRGBA color;
  color.a = 1;
  color.r = 1;
  visualization_msgs::MarkerArray foot_markers = free_gait::RosVisualization::getFootholdsMarker(stance_to_reach, foot_frame, color, 0.08);
  stance_marker_pub_.publish(foot_markers);

  //! WSHY: set to publish robot state to the balance controller
//  ROS_INFO("=============================In ros state publisher : \n");
//  std::cout<<state<<std::endl;
  kindr_ros::convertToRosGeometryMsg(Pose(Position(state.getTargetPositionWorldToBaseInWorldFrame()),
                                          RotationQuaternion(state.getTargetOrientationBaseToWorld())),
                                     robot_state_.base_pose.pose.pose);
//  std::cout<<state.getTargetOrientationBaseToWorld()<<std::endl;
  kindr_ros::convertToRosGeometryMsg(Twist(LinearVelocity(state.getTargetLinearVelocityBaseInWorldFrame()),
                                           LocalAngularVelocity(state.getTargetAngularVelocityBaseInBaseFrame())),
                                     robot_state_.base_pose.twist.twist);
//  ROS_INFO("In ros state publisher");
  LegMotionBase::Type leg_motion_type;
  if(state.isSupportLeg(LimbEnum::LF_LEG))
      {
        robot_state_.lf_leg_mode.support_leg = true;
        if(state.hasSurfaceNormal(LimbEnum::LF_LEG)){
            kindr_ros::convertToRosGeometryMsg(state.getSurfaceNormal(LimbEnum::LF_LEG),
                                               robot_state_.lf_leg_mode.surface_normal.vector);
          } else {
            kindr_ros::convertToRosGeometryMsg(Vector(0,0,1),
                                               robot_state_.lf_leg_mode.surface_normal.vector);
          }
        robot_state_.lf_leg_mode.phase = 0;
        if(!step_queue.empty()&& step_queue.getCurrentStep().isUpdated() && step_queue.getCurrentStep().hasBaseMotion())
          robot_state_.lf_leg_mode.phase = step_queue.getCurrentStep().getBaseMotionPhase();
      }else {
      robot_state_.lf_leg_mode.phase = 0;
      if(!step_queue.empty()&& step_queue.getCurrentStep().isUpdated() && step_queue.getCurrentStep().hasLegMotion(LimbEnum::LF_LEG))
        {
          robot_state_.lf_leg_mode.phase = step_queue.getCurrentStep().getLegMotionPhase(LimbEnum::LF_LEG);
          leg_motion_type = step_queue.getCurrentStep().getLegMotion(LimbEnum::LF_LEG).getType();
        }
      robot_state_.lf_leg_mode.support_leg = false;
//        ROS_INFO("In ros state publisher");
//        std::cout<<state.getJointPositionsForLimb(LimbEnum::LF_LEG)<<std::endl;
        if(leg_motion_type == LegMotionBase::Type::JointTarget || leg_motion_type == LegMotionBase::Type::JointTrajectory)
          {
            robot_state_.lf_leg_mode.name ="joint";
            robot_state_.lf_leg_joints.position[0] = state.getJointPositionsForLimb(LimbEnum::LF_LEG)(0);
            robot_state_.lf_leg_joints.position[1] = state.getJointPositionsForLimb(LimbEnum::LF_LEG)(1);
            robot_state_.lf_leg_joints.position[2] = state.getJointPositionsForLimb(LimbEnum::LF_LEG)(2);
          }
        if(leg_motion_type == LegMotionBase::Type::LegMode)
          {
            robot_state_.lf_leg_mode.name = "leg_mode";
          }
        //! WSHY: packaging the endefector command
        if(leg_motion_type == LegMotionBase::Type::EndEffectorTarget || leg_motion_type == LegMotionBase::Type::EndEffectorTrajectory)
          {
            robot_state_.lf_leg_mode.name = "cartesian";
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootPositionInBaseForLimb(LimbEnum::LF_LEG),
                                               robot_state_.lf_target.target_position[0].point);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootVelocityInBaseForLimb(LimbEnum::LF_LEG),
                                               robot_state_.lf_target.target_velocity[0].vector);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootAccelerationInBaseForLimb(LimbEnum::LF_LEG),
                                               robot_state_.lf_target.target_acceleration[0].vector);

          }
        if(leg_motion_type == LegMotionBase::Type::Footstep)
          {
            robot_state_.lf_leg_mode.name = "footstep";
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootPositionInBaseForLimb(LimbEnum::LF_LEG),
                                               robot_state_.lf_target.target_position[0].point);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootVelocityInBaseForLimb(LimbEnum::LF_LEG),
                                               robot_state_.lf_target.target_velocity[0].vector);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootAccelerationInBaseForLimb(LimbEnum::LF_LEG),
                                               robot_state_.lf_target.target_acceleration[0].vector);

          }

        /****************
        * TODO(Shunyao) : velocities command
        ****************/
//        ROS_INFO("In ros state publisher");
    }
    if(state.isSupportLeg(LimbEnum::RF_LEG))
      {
        robot_state_.rf_leg_mode.support_leg = true;
        if(state.hasSurfaceNormal(LimbEnum::RF_LEG)){
            kindr_ros::convertToRosGeometryMsg(state.getSurfaceNormal(LimbEnum::RF_LEG),
                                               robot_state_.rf_leg_mode.surface_normal.vector);
          } else {
            kindr_ros::convertToRosGeometryMsg(Vector(0,0,1),
                                               robot_state_.rf_leg_mode.surface_normal.vector);
          }
        robot_state_.rf_leg_mode.phase = 0;
        if(!step_queue.empty() && step_queue.getCurrentStep().isUpdated() && step_queue.getCurrentStep().hasBaseMotion())
          robot_state_.rf_leg_mode.phase = step_queue.getCurrentStep().getBaseMotionPhase();
      }else {
        robot_state_.rf_leg_mode.phase = 0;
        if(!step_queue.empty()&& step_queue.getCurrentStep().isUpdated() && step_queue.getCurrentStep().hasLegMotion(LimbEnum::RF_LEG))
          {
            robot_state_.rf_leg_mode.phase = step_queue.getCurrentStep().getLegMotionPhase(LimbEnum::RF_LEG);
            leg_motion_type = step_queue.getCurrentStep().getLegMotion(LimbEnum::RF_LEG).getType();
          }
        robot_state_.rf_leg_mode.support_leg = false;
        if(leg_motion_type == LegMotionBase::Type::JointTarget || leg_motion_type == LegMotionBase::Type::JointTrajectory)
          {
            robot_state_.rf_leg_mode.name = "joint";
            robot_state_.rf_leg_joints.position[0] = state.getJointPositionsForLimb(LimbEnum::RF_LEG)(0);
            robot_state_.rf_leg_joints.position[1] = state.getJointPositionsForLimb(LimbEnum::RF_LEG)(1);
            robot_state_.rf_leg_joints.position[2] = state.getJointPositionsForLimb(LimbEnum::RF_LEG)(2);
          }
        if(leg_motion_type == LegMotionBase::Type::LegMode)
          {
            robot_state_.rf_leg_mode.name = "leg_mode";
          }

        //! WSHY: packaging the endefector command
        if(leg_motion_type == LegMotionBase::Type::EndEffectorTarget || leg_motion_type == LegMotionBase::Type::EndEffectorTrajectory)
          {
            robot_state_.rf_leg_mode.name = "cartesian";
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootPositionInBaseForLimb(LimbEnum::RF_LEG),
                                               robot_state_.rf_target.target_position[0].point);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootVelocityInBaseForLimb(LimbEnum::RF_LEG),
                                               robot_state_.rf_target.target_velocity[0].vector);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootAccelerationInBaseForLimb(LimbEnum::RF_LEG),
                                               robot_state_.rf_target.target_acceleration[0].vector);
          }
        if(leg_motion_type == LegMotionBase::Type::Footstep)
          {
            robot_state_.rf_leg_mode.name = "footstep";
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootPositionInBaseForLimb(LimbEnum::RF_LEG),
                                               robot_state_.rf_target.target_position[0].point);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootVelocityInBaseForLimb(LimbEnum::RF_LEG),
                                               robot_state_.rf_target.target_velocity[0].vector);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootAccelerationInBaseForLimb(LimbEnum::RF_LEG),
                                               robot_state_.rf_target.target_acceleration[0].vector);
          }

    }
    if(state.isSupportLeg(LimbEnum::RH_LEG))
      {
        robot_state_.rh_leg_mode.support_leg = true;
        if(state.hasSurfaceNormal(LimbEnum::RH_LEG)){
            kindr_ros::convertToRosGeometryMsg(state.getSurfaceNormal(LimbEnum::RH_LEG),
                                               robot_state_.rh_leg_mode.surface_normal.vector);
          } else {
            kindr_ros::convertToRosGeometryMsg(Vector(0,0,1),
                                               robot_state_.rh_leg_mode.surface_normal.vector);
          }
        robot_state_.rh_leg_mode.phase = 0;
        if(!step_queue.empty()&& step_queue.getCurrentStep().isUpdated() && step_queue.getCurrentStep().hasBaseMotion())
          robot_state_.rh_leg_mode.phase = step_queue.getCurrentStep().getBaseMotionPhase();
      }else {
        robot_state_.rh_leg_mode.phase = 0;
        if(!step_queue.empty()&& step_queue.getCurrentStep().isUpdated() && step_queue.getCurrentStep().hasLegMotion(LimbEnum::RH_LEG))
          {
            robot_state_.rh_leg_mode.phase = step_queue.getCurrentStep().getLegMotionPhase(LimbEnum::RH_LEG);
            leg_motion_type = step_queue.getCurrentStep().getLegMotion(LimbEnum::RH_LEG).getType();
          }
        robot_state_.rh_leg_mode.support_leg = false;
        if(leg_motion_type == LegMotionBase::Type::JointTarget || leg_motion_type == LegMotionBase::Type::JointTrajectory)
          {
            robot_state_.rh_leg_mode.name = "joint";
            robot_state_.rh_leg_joints.position[0] = state.getJointPositionsForLimb(LimbEnum::RH_LEG)(0);
            robot_state_.rh_leg_joints.position[1] = state.getJointPositionsForLimb(LimbEnum::RH_LEG)(1);
            robot_state_.rh_leg_joints.position[2] = state.getJointPositionsForLimb(LimbEnum::RH_LEG)(2);
          }
        if(leg_motion_type == LegMotionBase::Type::LegMode)
          {
            robot_state_.rh_leg_mode.name = "leg_mode";
          }

        //! WSHY: packaging the endefector command
        if(leg_motion_type == LegMotionBase::Type::EndEffectorTarget || leg_motion_type == LegMotionBase::Type::EndEffectorTrajectory)
          {
            robot_state_.rh_leg_mode.name = "cartesian";
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootPositionInBaseForLimb(LimbEnum::RH_LEG),
                                               robot_state_.rh_target.target_position[0].point);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootVelocityInBaseForLimb(LimbEnum::RH_LEG),
                                               robot_state_.rh_target.target_velocity[0].vector);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootAccelerationInBaseForLimb(LimbEnum::RH_LEG),
                                               robot_state_.rh_target.target_acceleration[0].vector);
          }
        if(leg_motion_type == LegMotionBase::Type::Footstep)
          {
            robot_state_.rh_leg_mode.name = "footstep";
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootPositionInBaseForLimb(LimbEnum::RH_LEG),
                                               robot_state_.rh_target.target_position[0].point);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootVelocityInBaseForLimb(LimbEnum::RH_LEG),
                                               robot_state_.rh_target.target_velocity[0].vector);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootAccelerationInBaseForLimb(LimbEnum::RH_LEG),
                                               robot_state_.rh_target.target_acceleration[0].vector);
          }

    }
    if(state.isSupportLeg(LimbEnum::LH_LEG))
      {
        robot_state_.lh_leg_mode.support_leg = true;
        if(state.hasSurfaceNormal(LimbEnum::LH_LEG)){
            kindr_ros::convertToRosGeometryMsg(state.getSurfaceNormal(LimbEnum::LH_LEG),
                                               robot_state_.lh_leg_mode.surface_normal.vector);
          } else {
            kindr_ros::convertToRosGeometryMsg(Vector(0,0,1),
                                               robot_state_.lh_leg_mode.surface_normal.vector);
          }
        robot_state_.lh_leg_mode.phase = 0;
        if(!step_queue.empty()&& step_queue.getCurrentStep().isUpdated() && step_queue.getCurrentStep().hasBaseMotion())
          robot_state_.lh_leg_mode.phase = step_queue.getCurrentStep().getBaseMotionPhase();
      }else {
        robot_state_.lh_leg_mode.phase = 0;
        if(!step_queue.empty()&& step_queue.getCurrentStep().isUpdated() && step_queue.getCurrentStep().hasLegMotion(LimbEnum::LH_LEG))
          {
            robot_state_.lh_leg_mode.phase = step_queue.getCurrentStep().getLegMotionPhase(LimbEnum::LH_LEG);
            leg_motion_type = step_queue.getCurrentStep().getLegMotion(LimbEnum::LH_LEG).getType();
          }
        robot_state_.lh_leg_mode.support_leg = false;
        if(leg_motion_type == LegMotionBase::Type::JointTarget || leg_motion_type == LegMotionBase::Type::JointTrajectory)
          {
            robot_state_.lh_leg_mode.name = "joint";
            robot_state_.lh_leg_joints.position[0] = state.getJointPositionsForLimb(LimbEnum::LH_LEG)(0);
            robot_state_.lh_leg_joints.position[1] = state.getJointPositionsForLimb(LimbEnum::LH_LEG)(1);
            robot_state_.lh_leg_joints.position[2] = state.getJointPositionsForLimb(LimbEnum::LH_LEG)(2);
          }
        if(leg_motion_type == LegMotionBase::Type::LegMode)
          {
            robot_state_.lh_leg_mode.name = "leg_mode";
          }

        //! WSHY: packaging the endefector command
        if(leg_motion_type == LegMotionBase::Type::EndEffectorTarget || leg_motion_type == LegMotionBase::Type::EndEffectorTrajectory)
          {
            robot_state_.lh_leg_mode.name = "cartesian";
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootPositionInBaseForLimb(LimbEnum::LH_LEG),
                                               robot_state_.lh_target.target_position[0].point);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootVelocityInBaseForLimb(LimbEnum::LH_LEG),
                                               robot_state_.lh_target.target_velocity[0].vector);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootAccelerationInBaseForLimb(LimbEnum::LH_LEG),
                                               robot_state_.lh_target.target_acceleration[0].vector);
          }
        if(leg_motion_type == LegMotionBase::Type::Footstep)
          {
            robot_state_.lh_leg_mode.name = "footstep";
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootPositionInBaseForLimb(LimbEnum::LH_LEG),
                                               robot_state_.lh_target.target_position[0].point);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootVelocityInBaseForLimb(LimbEnum::LH_LEG),
                                               robot_state_.lh_target.target_velocity[0].vector);
            kindr_ros::convertToRosGeometryMsg(state.getTargetFootAccelerationInBaseForLimb(LimbEnum::LH_LEG),
                                               robot_state_.lh_target.target_acceleration[0].vector);
          }

      }
  robot_state_pub_.publish(robot_state_);
//  ROS_INFO("Publised robot state once");
  return true;
}
//void StateRosPublisher::publishSupportRegion(const State& state)

} /* namespace free_gait */
