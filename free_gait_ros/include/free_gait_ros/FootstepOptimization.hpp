/*
 *  FootstepOptimization.hpp
 *  Descriotion:
 *
 *  Created on: 24, Apr, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#pragma once

#include "ros/ros.h"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/GridMap.h"
#include "grid_map_ros/GridMapRosConverter.hpp"

#include "free_gait_core/free_gait_core.hpp"


class FootstepOptimization
{
public:
  FootstepOptimization(const ros::NodeHandle& node_handle);
  ~FootstepOptimization();


  bool getOptimizedFoothold(free_gait::Position& nominal_foothold,
                            const free_gait::State& robot_state,
                            const free_gait::LimbEnum& limb,
                            const std::string frame_id);
  double getMaxObstacleHeight(free_gait::Position& nominal_foothold,
                                           free_gait::State& robot_state,
                                           const free_gait::LimbEnum& limb);


  free_gait::Vector getSurfaceNormal(const free_gait::Position& foot_position);

private:

  void traversabilityMapCallback(const grid_map_msgs::GridMapConstPtr& traversability_map);

  void initialize();

  bool checkKinematicsConstriants(const free_gait::LimbEnum& limb,
                                  const grid_map::Index& index);

  ros::NodeHandle node_handle_;

  ros::Subscriber traversability_map_sub_;

  grid_map::GridMap traversability_map_;

  free_gait::State robot_state_;

  std::string foot_frame_id_;

};
