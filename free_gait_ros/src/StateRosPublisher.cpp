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
  robot_state_.lf_leg_joints.position.resize(3);
  robot_state_.rf_leg_joints.position.resize(3);
  robot_state_.rh_leg_joints.position.resize(3);
  robot_state_.lh_leg_joints.position.resize(3);
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
  std::map<std::string, double> jointPositionsMap;
  for (size_t i = 0; i < jointNames.size(); ++i) {
    jointPositionsMap[jointNames[i]] = jointPositions(i);
    std::cout<<jointNames[i]<<" position is : "<<jointPositions(i)<<std::endl;
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
  ROS_INFO("=============================In ros state publisher : \n");
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
  ROS_INFO("Publised robot state once");
  return true;
}

//void StateRosPublisher::publishSupportRegion(const State& state)

} /* namespace free_gait */
