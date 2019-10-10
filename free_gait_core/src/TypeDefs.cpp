/*
 * TypeDefs.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/TypePrints.hpp"

namespace free_gait {

struct CompareByCounterClockwiseOrder
{
  bool operator()(const LimbEnum& a, const LimbEnum& b) const
  {
    const size_t aIndex = findIndex(a);
    const size_t bIndex = findIndex(b);
    return aIndex < bIndex;
  }

  size_t findIndex(const LimbEnum& limb) const
  {
    auto it = std::find(limbEnumCounterClockWiseOrder.begin(), limbEnumCounterClockWiseOrder.end(), limb);
    if (it == limbEnumCounterClockWiseOrder.end()) {
      throw std::runtime_error("Could not find index for limb!");
    }
    return std::distance(limbEnumCounterClockWiseOrder.begin(), it);
  }
};

/**
  * overload the operator with cout and stanceorder.....
  */
//std::ostream operator << (std::ostream& out, const std::map<LimbEnum, Position, CompareByCounterClockwiseOrder>& stanceOrdered)
//{
//    std::cout <<"output_1" << std::endl;
//    for ( const auto& limb : stanceOrdered) {
//        out << limb.first << ":" << limb.second <<std::endl;
//    }
//    std::cout <<"end" << std::endl;
//}

void getFootholdsCounterClockwiseOrdered(const Stance& stance, std::vector<Position>& footholds)
{
  footholds.reserve(stance.size());//the use of vector.reserve
  std::map<LimbEnum, Position, CompareByCounterClockwiseOrder> stanceOrdered;//orderd by the counter clock, mainly change the foothold variable.
  for (const auto& limb : stance)
  {
      stanceOrdered.insert(limb);//stance-> <limbEnum,Position> loop about stance
  }
//  operator<<(std::cout, stanceOrdered);
  for (const auto& limb : stanceOrdered) footholds.push_back(limb.second);
};

} // namespace
