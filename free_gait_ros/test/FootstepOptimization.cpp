/*
 *  FootstepOptimization.cpp
 *  Descriotion:
 *
 *  Created on: 24, Apr, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "free_gait_ros/FootstepOptimization.hpp"

FootstepOptimization::FootstepOptimization(const ros::NodeHandle& node_handle)
  : node_handle_(node_handle)
{
  initialize();
}

FootstepOptimization::~FootstepOptimization()
{

}

void FootstepOptimization::initialize()
{
  traversability_map_sub_ = node_handle_.subscribe("/grid_map_filter_demo/filtered_map", 1,
                                                   &FootstepOptimization::traversabilityMapCallback, this);
}

void FootstepOptimization::traversabilityMapCallback(const grid_map_msgs::GridMapConstPtr& traversability_map)
{
  grid_map::GridMapRosConverter::fromMessage(*traversability_map, traversability_map_);
//  grid_map::Index center_index;
//  Eigen::Vector2d map_origin;
//  traversability_map_.getPosition(grid_map::Index(0,0), map_origin);
//  traversability_map_.getIndex(Eigen::Vector2d(0.4,0.175), center_index);
//  bool is_center_valid = traversability_map_.isValid(center_index);
//  double center_height = traversability_map_.at("elevation", center_index);

}

bool FootstepOptimization::getOptimizedFoothold(free_gait::Position& nominal_foothold,
                                                 const free_gait::State& robot_state,
                                                 const free_gait::LimbEnum& limb,
                                                 const std::string frame_id)
{
  robot_state_ = robot_state;
  foot_frame_id_ = frame_id;
  Position nominal_foothold_in_map;

  if(foot_frame_id_ == "foot_print")
    {
      double yaw_angle;
      EulerAnglesZyx ypr(robot_state_.getOrientationBaseToWorld());
      yaw_angle = ypr.setUnique().vector()(0);
      RotationQuaternion orinetationFootprintToWorld = RotationQuaternion(EulerAnglesZyx(yaw_angle,0,0));
      robot_state_.getLinearVelocityBaseInWorldFrame();
      nominal_foothold_in_map = robot_state_.getPositionWorldToBaseInWorldFrame()
          + orinetationFootprintToWorld.rotate(nominal_foothold);
    }else if(foot_frame_id_ == "odom")
    {
      nominal_foothold_in_map = nominal_foothold;
    }

//  Position nominal_foothold_in_map = robot_state_.getOrientationBaseToWorld().inverseRotate(nominal_foothold - robot_state.getPositionWorldToBaseInWorldFrame());
  Eigen::Vector2d center(nominal_foothold_in_map(0), nominal_foothold_in_map(1));
//    return true;
  double radius = 0.08;
  int i = 0;
    for(grid_map::CircleIterator iterator(traversability_map_, center, radius); !iterator.isPastEnd(); ++iterator)
    {
      if(traversability_map_.at("traversability", *iterator) > 0.9)
        {
          if(checkKinematicsConstriants(limb, *iterator))
            {
              Position optimazed_foothold_in_map;
              traversability_map_.getPosition3("elevation_inpainted", *iterator, optimazed_foothold_in_map.vector());
//              ROS_INFO("Find a foothold");

              if(i>0)
                {
                  nominal_foothold = optimazed_foothold_in_map;
                  ROS_INFO("Find a new foothold");
                }else{
                  ROS_INFO("Keep the foothold");
                }
              return true;
            }
        }
      i++;
    }
    return false;



}

double FootstepOptimization::getMaxObstacleHeight(free_gait::Position& nominal_foothold,
                                                   free_gait::State& robot_state,
                                                   const free_gait::LimbEnum& limb)
{
  Position start_foot_position = robot_state.getPositionWorldToFootInWorldFrame(limb);
  Position end_foot_position = nominal_foothold;

  grid_map::Index start, end;

  traversability_map_.getIndex(grid_map::Position(start_foot_position(0),start_foot_position(1)),
                               start);
  traversability_map_.getIndex(grid_map::Position(end_foot_position(0), end_foot_position(1)),
                               end);
  double max_height = 0;
  double height = 0;
  for(grid_map::LineIterator iterator(traversability_map_, start, end);
      !iterator.isPastEnd(); ++iterator)
    {
      height = traversability_map_.at("elevation_inpainted", *iterator);
      if(height > max_height)
        {
          max_height = height;
        }
    }
  return max_height - 0.5*(end_foot_position(2) + start_foot_position(2));

}

bool FootstepOptimization::checkKinematicsConstriants(const free_gait::LimbEnum& limb,
                                                      const grid_map::Index& index)
{

  double leg_lenth = 0.5;
  Position hip_in_base = robot_state_.getPositionBaseToHipInBaseFrame(limb);
  Position foothold_in_map, hip_in_map;//, hip_in_world;
//  Position center_of_map;
//  traversability_map_.getPosition3("elevation", traversability_map_.getStartIndex(), center_of_map.vector());
  grid_map::Position3 position;
  traversability_map_.getPosition3("elevation_inpainted", index, position);
  foothold_in_map = Position(position);
  hip_in_map = robot_state_.getPositionWorldToBaseInWorldFrame() + robot_state_.getOrientationBaseToWorld().rotate(hip_in_base);
//  hip_in_map = hip_in_world - center_of_map;
  if((hip_in_map - foothold_in_map).norm()>leg_lenth)
    return false;
  return true;
}

free_gait::Vector FootstepOptimization::getSurfaceNormal(const free_gait::Position& foot_position)
{
  grid_map::Index index;
  grid_map::Position position(foot_position(0), foot_position(1));

  traversability_map_.getIndex(position, index);

  free_gait::Vector surface_normal;
  surface_normal(0) = 0;//traversability_map_.at("normal_vectors_x", index);
  surface_normal(1) = 0;//traversability_map_.at("normal_vectors_y", index);
  surface_normal(2) = 1;//traversability_map_.at("normal_vectors_z", index);

  return surface_normal;
}



