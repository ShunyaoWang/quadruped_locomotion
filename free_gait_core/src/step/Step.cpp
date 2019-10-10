/*
 * Step.cpp
 *
 *  Created on: Jan 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "free_gait_core/step/Step.hpp"
#include "free_gait_core/base_motion/base_motion.hpp"
#include "free_gait_core/TypeDefs.hpp"

#include <uuid/uuid.h>

namespace free_gait {

inline void boundToRange(double* v, double min, double max){
  if (*v < min) *v = min;
  if (*v > max) *v = max;
}

inline double mapTo01Range(double v, double min, double max){
  double t = v;
  if (fabs(min - max) < 0.000000001) return 1;
  boundToRange(&t, min, max);
  t = (t-min)/(max-min);
  return t;
}
/**
 * @brief Step::Step
 */
Step::Step()
    : time_(0.0),
      totalDuration_(0.0),
      isUpdated_(false),
      isComputed_(false)
{
  legMotions_.clear();
  baseMotion_.reset();
  customCommands_.clear();

  // Generate unique id.
  // TODO(Shunyao): figure out the useness of this unique id
  uuid_t id;
  uuid_generate(id);
  char* stringId = new char[36];
  uuid_unparse(id, stringId);
  id_.assign(stringId, 36);
  delete stringId;
}

Step::~Step()
{
}
/**
 * @brief Step::Step Copy Constructor
 * @param other
 */
Step::Step(const Step& other)
    : time_(other.time_),
      totalDuration_(other.totalDuration_),
      isUpdated_(other.isUpdated_),
      isComputed_(other.isComputed_),
      id_(other.id_),
      customCommands_(other.customCommands_)
{
  if (other.baseMotion_) baseMotion_ = std::move(other.baseMotion_->clone());
  legMotions_.clear();
  for (const auto& legMotion : other.legMotions_) {
    legMotions_[legMotion.first] = std::move(legMotion.second->clone());
  }
}
/**
 * @brief Step::operator =
 * @param other
 * @return
 */
Step& Step::operator=(const Step& other)
{
  time_ = other.time_;
  totalDuration_ = other.time_;
  isUpdated_ = other.isUpdated_;
  isComputed_ = other.isComputed_;
  id_ = other.id_;
  customCommands_ = other.customCommands_;
  if (other.baseMotion_) baseMotion_ = std::move(other.baseMotion_->clone());
  legMotions_.clear();
  for (const auto& legMotion : other.legMotions_) {
    legMotions_[legMotion.first] = std::move(legMotion.second->clone());
    //if legmotion.first matches an element in the container, then function returns a reference to its mapped value.
  }
  return *this;
}
/**
 * @brief Step::clone, clone a step class
 * @return unique_ptr to Step class
 */
std::unique_ptr<Step> Step::clone() const
{
  std::unique_ptr<Step> pointer(new Step(*this));
  return pointer;
}
/**gggggggggg
 * @brief Step::addLegMotion, add a LegMotionBase class to legMotions_ unorder map data type
 * @param legMotion test the stom
 */
void Step::addLegMotion(const LegMotionBase& legMotion)
{
  if (hasLegMotion(legMotion.getLimb())) {
    legMotions_.erase(legMotion.getLimb());
  }//remove this leg motion and insert
  /**
    map<int, int> maps;
    maps.insert(pair<int, int> (10, 15));
    std::pair<LimbEnum, std::unique_ptr<LegMotionBase>> pair
    ????
    */
  legMotions_.insert(std::pair<LimbEnum, std::unique_ptr<LegMotionBase>>(legMotion.getLimb(), std::move(legMotion.clone())));
  isUpdated_ = false;
  isComputed_ = false;
}
/**
 * @brief Step::addBaseMotion, add a BaseMotionBase to baseMotion_
 * @param baseMotion yes
 */
void Step::addBaseMotion(const BaseMotionBase& baseMotion)
{
  baseMotion_ = std::move(baseMotion.clone());
  isUpdated_ = false;
  isComputed_ = false;
}

void Step::addCustomCommand(const CustomCommand& customCommand)
{
  customCommands_.push_back(customCommand);
  isUpdated_ = false;
  isComputed_ = false;
}
/**
 * @brief Step::needsComputation, check each in legMotions_ and baseMotion if need computed
 * typedef std::unordered_map<LimbEnum, std::unique_ptr<LegMotionBase>, EnumClassHash> LegMotions;
 * about the definition of auto
 * @return needsComputation
 */
bool Step::needsComputation() const
{
  bool needsComputation = false;
  for (auto& legMotion : legMotions_) {
    if (legMotion.second->needsComputation())//the second term of legMotion is a ptr
      needsComputation = true;
  }

  if (baseMotion_) {
    if (baseMotion_->needsComputation())
      needsComputation = true;
  }

  return needsComputation;
}
/**
 * @brief Step::compute, call the compute method in legMotions_ and baseMotion
 * @return true if successed
 * 1 unordered_map<Key,T>::iterator it;
2 (*it).first;             // the key value (of type Key)
3 (*it).second;            // the mapped value (of type T), legmotions is a class of legmotion
4 (*it);                   // the "element value" (of type pair<const Key,T>)
https://www.cnblogs.com/evidd/p/8823092.html
*/
bool Step::compute()
{
  /**
    bianli legMotions_
    */
  for (const auto& legMotion : legMotions_) {
    if (legMotion.second->needsComputation()) {
      if (!legMotion.second->compute()) return false;
    }
  }
  if (hasBaseMotion()) {
    if (baseMotion_->needsComputation()) { //all baseMotion class do not need computation, return false
      if (!baseMotion_->compute()) return false;
    }
  }
  isComputed_ = true;
  return true;
}

bool Step::isUpdated() const
{
  return isUpdated_;
}
/**
 * @brief Step::reset, call reset method in legMotions_ and baseMotion
 */
void Step::reset()
{
  time_ = 0.0;
  totalDuration_ = 0.0;
  isUpdated_ = false;
  isComputed_ = false;
  for (const auto& legMotion : legMotions_) {
    legMotion.second->reset();
  }
  if (hasBaseMotion()) {
    baseMotion_->reset();
  }
}
/**
 * @brief Step::update, update totalDuration_, make the longest motion time as
 * total duration
 * @return
 */
bool Step::update()
{
  if (needsComputation() && !isComputed_) return false;

  totalDuration_ = 0.0;
  for (const auto& legMotion : legMotions_) {
    if (legMotion.second->getDuration() > totalDuration_)
      totalDuration_ = legMotion.second->getDuration();
  }
  if (hasBaseMotion()) {
    if (baseMotion_->getDuration() > totalDuration_)
      totalDuration_ = baseMotion_->getDuration();
  }
  for (const auto& customCommand : customCommands_) {
    if (customCommand.getDuration() > totalDuration_)
      totalDuration_ = customCommand.getDuration();
  }
  return isUpdated_ = true;
}
/**
 * @brief Step::advance, add dt to time_
 * @param dt
 * @return true if time_ reach totalDuration_
 */
bool Step::advance(double dt)
{
  if (!isUpdated_) throw std::runtime_error("Step::advance() cannot be called if step is not updated.");
  time_ += dt;
  if (time_ > getTotalDuration()) return false;
  return true;
}

bool Step::hasLegMotion() const
{
  return !legMotions_.empty();//true if the legmotions_ is empty
}

bool Step::hasLegMotion(const LimbEnum& limb) const
{
    /**
      means if it not found the leg motion, it is true.
      */
  return !(legMotions_.find(limb) == legMotions_.end());//find an element with limb and returns an iterator to it if found,
                                                        //otherwise it returns an iterator to unordered_map::end
}

/**
 * @brief Step::getLegMotion std::unordered_map<LimbEnum, std::unique_ptr<LegMotionBase>, EnumClassHash> LegMotions
 * @param limb
 * @return return the legmotionbase.
 */
const LegMotionBase& Step::getLegMotion(const LimbEnum& limb) const
{
  if (!hasLegMotion(limb)) throw std::out_of_range("No leg motion for this limb in this step!");
  return *legMotions_.at(limb);
}

const Step::LegMotions& Step::getLegMotions() const
{
  return legMotions_;
}

bool Step::hasBaseMotion() const
{
  return (bool)(baseMotion_);
}

const BaseMotionBase& Step::getBaseMotion() const
{
  if (!hasBaseMotion()) throw std::out_of_range("No base motion in this step!");
  return *baseMotion_;
}

BaseMotionBase& Step::getBaseMotion()
{
  if (!hasBaseMotion()) throw std::out_of_range("No base motion in this step!");
  return *baseMotion_;
}

bool Step::hasCustomCommand() const
{
  return !customCommands_.empty();
}

const std::vector<CustomCommand>& Step::getCustomCommands() const
{
  return customCommands_;
}

double Step::getTime() const
{
  return time_;
}

//double Step::getStateTime(const Step::State& state)
//{
//  switch (state) {
//    case State::PreStep:
//      return getTime();
//    case State::AtStep:
//      return getTime() - getStateDuration(State::PreStep);
//    case State::PostStep:
//      return getTime() - getStateDuration(State::PreStep) - getStateDuration(State::AtStep);
//    default:
//      return 0.0;
//  }
//}

double Step::getTotalDuration() const
{
  if (!isUpdated_) throw std::runtime_error("Step::getTotalDuration() cannot be called if step is not updated.");
  return totalDuration_;
}

double Step::getTotalPhase() const
{
  if (!isUpdated_) throw std::runtime_error("Step::getTotalPhase() cannot be called if step is not updated.");
  return mapTo01Range(time_, 0.0, getTotalDuration());
}

//double Step::getTotalDuration()
//{
////  switch (state) {
////    case Step::State::PreStep:
////      return hasBaseMotion(State::PreStep) ? baseMotions_.at(State::PreStep).getDuration() : 0.0;
////    case Step::State::AtStep:
////      if (!isDurationComputed_) computeDurations();
////      return atStepDuration_;
////    case Step::State::PostStep:
////      return hasBaseMotion(State::PostStep) ? baseMotions_.at(State::PostStep).getDuration() : 0.0;
////    default:
////      return 0.0;
////  }
//}
/**
 * @brief Step::getLegMotionDuration,
 * @param limb
 * @return get legMotions_ duration, 0.0 if not hasLegMotion
 */
double Step::getLegMotionDuration(const LimbEnum& limb) const
{
  if (!isUpdated_) throw std::runtime_error("Step::getLegMotionDuration() cannot be called if step is not updated.");
  if (!hasLegMotion(limb)) return 0.0;
  return legMotions_.at(limb)->getDuration();
}

double Step::getLegMotionPhase(const LimbEnum& limb) const
{
  if (!isUpdated_) throw std::runtime_error("Step::getLegMotionPhase() cannot be called if step is not updated.");
  return mapTo01Range(time_, 0.0, getLegMotionDuration(limb));
}

double Step::getBaseMotionDuration() const
{
  if (!isUpdated_) throw std::runtime_error("Step::getBaseMotionDuration() cannot be called if step is not updated.");
  if (!hasBaseMotion()) return 0.0;
  return baseMotion_->getDuration();
}

double Step::getBaseMotionPhase() const
{
  if (!isUpdated_) throw std::runtime_error("Step::getBaseMotionPhase() cannot be called if step is not updated.");
  return mapTo01Range(time_, 0.0, getBaseMotionDuration());
}

//
//double Step::getStatePhase(const Step::State& state)
//{
//  switch (state) {
//    case Step::State::PreStep:
//      return loco::mapTo01Range(getTime(), 0.0, getStateDuration(State::PreStep));
//    case Step::State::AtStep:
//      return loco::mapTo01Range(getTime() - getStateDuration(State::PreStep), 0.0, getStateDuration(State::AtStep));
//    case Step::State::PostStep:
//      return loco::mapTo01Range(getTime() - getStateDuration(State::PreStep) - getStateDuration(State::AtStep), 0.0,
//                                getStateDuration(State::PostStep));
//    default:
//      return 0.0;
//  }
//}

//double Step::getAtStepPhaseForLeg(const quadruped_model::LimbEnum& limb)
//{
//  return loco::mapTo01Range(getTime() - getStateDuration(State::PreStep), 0.0, getAtStepDurationForLeg(limb));
//}
//
//double Step::getTotalPhase()
//{
//  return loco::mapTo01Range(getTime(), 0.0, getTotalDuration());
//}
/**
 * @brief Step::isApproachingEnd, check time_ approach the totalDuration with tolerance
 * @param tolerance
 * @return ture if reach totalDuration with tolerance
 */
bool Step::isApproachingEnd(double tolerance) const
{
  if (!isUpdated_) throw std::runtime_error("Step::isApproachingEnd() cannot be called if step is not updated.");
  if (getTime() + tolerance >= getTotalDuration()) return true;
  return false;
}

const std::string& Step::getId() const
{
  return id_;
}

void Step::setId(const std::string& id)
{
  id_ = id;
}
/**
 * @brief operator <<,
 * @param out
 * @param step
 * @return
 */
std::ostream& operator<<(std::ostream& out, const Step& step)
{
  out << "------" << std::endl;
  out << "ID: " << step.id_ << std::endl;
  if (step.hasLegMotion()) {
    out << "---" << std::endl;
    out << "Leg motions (" << step.legMotions_.size() << "):" << std::endl;
    for (const auto& legMotion : step.legMotions_) out << "-" << std::endl << *(legMotion.second) << std::endl;
  }
  if (step.hasBaseMotion()) {
    out << "---" << std::endl;
    out << "Base motion: " << std::endl;
    out << *(step.baseMotion_) << std::endl;
  }
  if (step.hasCustomCommand()) {
    out << "---" << std::endl;
    out << "Custom commands (" << step.customCommands_.size() << "):" << std::endl;
    for (const auto& customCommand : step.customCommands_) out << "-" << std::endl << customCommand << std::endl;
  }
  return out;
}

} /* namespace */
