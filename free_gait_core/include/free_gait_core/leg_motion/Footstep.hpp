/*
 * Footstep.hpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/leg_motion/EndEffectorMotionBase.hpp"

// Curves
#include <curves/CubicHermiteE3Curve.hpp>

// STD
#include <string>
#include <memory>

namespace free_gait {

class Footstep : public EndEffectorMotionBase
{
 public:
  typedef typename curves::CubicHermiteE3Curve::ValueType ValueType;
  typedef typename curves::CubicHermiteE3Curve::DerivativeType DerivativeType;
  typedef typename curves::Time Time;

  Footstep(LimbEnum limb);
  virtual ~Footstep();

  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  std::unique_ptr<LegMotionBase> clone() const;

  /*!
   * Update the trajectory with the foot start position.
   * Do this to avoid jumps of the swing leg.
   * @param startPosition the start position of the foot in the frameId_ frame.
   * @return true if successful, false otherwise.
   */
  void updateStartPosition(const Position& startPosition);
  void updateStartVelocity(const LinearVelocity& startVelocity);
  const Position getStartPosition() const;
  const LinearVelocity getStartVelocity() const;

  const ControlSetup getControlSetup() const;

  bool compute(bool isSupportLeg);
  bool prepareComputation(const State& state, const Step& step, const AdapterBase& adapter);
  bool needsComputation() const;
  bool isComputed() const;
  void reset();

  /*!
   * Evaluate the swing foot position at a given swing phase value.
   * @param phase the swing phase value.
   * @return the position of the foot on the swing trajectory.
   */
  const Position evaluatePosition(const double time) const;

  const LinearVelocity evaluateVelocity(const double time) const;

  const LinearAcceleration evaluateAcceleration(const double time) const;

  /*!
   * Returns the total duration of the trajectory.
   * @return the duration.
   */
  double getDuration() const;
  void setMinimumDuration(const double minimumDuration);
  double getMinimumDuration() const;

  void setTargetPosition(const std::string& frameId, const Position& target);
  const Position getTargetPosition() const;
  const LinearVelocity getTargetVelocity() const;

  const std::string& getFrameId(const ControlLevel& controlLevel) const;

  void setProfileType(const std::string& profileType);
  const std::string& getProfileType() const;
  void setProfileHeight(const double profileHeight);
  double getProfileHeight() const;
  double getAverageVelocity() const;
  void setAverageVelocity(double averageVelocity);

  bool isIgnoreContact() const;
  bool isIgnoreForPoseAdaptation() const;

  const std::vector<ValueType>& getKnotValues() const;
  const std::vector<Time>& getTimes() const;

  /*!
   * Computes timing assuming equal average velocity between all knots.
   */
  static void computeTiming(const std::vector<ValueType>& values, const double averageVelocity, double minimumDuration,
                            std::vector<Time>& times);

  friend std::ostream& operator << (std::ostream& out, const Footstep& footstep);

  friend class StepCompleter;
  friend class StepRosConverter;
  friend class StepFrameConverter;

 private:
  void generateStraightKnots();
  void generateTriangleKnots();
  void generateSquareKnots();
  void generateTrapezoidKnots();
  void generateSquareKnotsAbsolute();

  Position start_;
  Position target_;
  LinearVelocity liftOffVelocity_;
  LinearVelocity touchdownVelocity_;
  std::string frameId_;
  double profileHeight_;
  std::string profileType_;
  double averageVelocity_;
  bool ignoreContact_;
  bool ignoreForPoseAdaptation_;
  double liftOffSpeed_;
  double touchdownSpeed_;
  double duration_;
  double minimumDuration_;

  ControlSetup controlSetup_;

  //! Foot trajectory.
  std::vector<ValueType> values_;
  std::vector<Time> times_;
  curves::CubicHermiteE3Curve trajectory_;

  //! If trajectory is updated.
  bool isComputed_;
};

} /* namespace */
