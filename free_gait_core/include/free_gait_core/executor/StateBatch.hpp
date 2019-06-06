/*
 * StateBatch.hpp
 *
 *  Created on: Dec 20, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <free_gait_core/executor/State.hpp>

#include <map>
#include <string>
#include <tuple>//rongqi

namespace free_gait {

class StateBatch
{
 public:
  StateBatch();
  virtual ~StateBatch();

  /**
   * @brief getStates
   * @return std::map<double, State> states_;
   *
   */
  const std::map<double, State>& getStates() const;
  /**
   * @brief getEndEffectorPositions
   * @return a series of endeffectorpositions
   */
  std::vector<std::map<double, Position>> getEndEffectorPositions() const;
  std::vector<std::map<double, Position>> getEndEffectorTargets() const;
  /**
   * @brief getSurfaceNormals
   * @return
   * std::tuple<Position, Vector>,means that it is a position vector, but Vector(Big V) is
   * typeless, they composed a tuple;
   * std::map<double, std::tuple<Position, Vector>>, this tuple and the double compose a map
   * std::vector<std::map<double, std::tuple<Position, Vector>>>
   */
  std::vector<std::map<double, std::tuple<Position, Vector>>> getSurfaceNormals() const;
  std::map<double, Stance> getStances() const;
  bool getEndTimeOfStep(std::string& stepId, double& endTime) const;
  void addState(const double time, const State& state);
  bool isValidTime(const double time) const;
  double getStartTime() const;
  double getEndTime() const;
  const State& getState(const double time) const;
  void clear();

  friend class StateBatchComputer;

 private:
  std::map<double, State> states_;
  std::vector<std::map<double, Position>> endEffectorPositions_;
  std::vector<std::map<double, Position>> endEffectorTargets_;
  std::vector<std::map<double, std::tuple<Position, Vector>>> surfaceNormals_;
  std::map<double, Stance> stances_;
  std::map<double, Pose> basePoses_;
  std::map<double, std::string> stepIds_;//double is the time corresponding to the specific step
};

} /* namespace free_gait */
