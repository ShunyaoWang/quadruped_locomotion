#pragma once

#include "kindr/Core"
#include <quadruped_model/common/typedefs.hpp>
//#include "quadruped_model/quadrupedkinematics.h"

struct EnumClassHash
{
  template<typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

using namespace romo;
namespace quadruped_model{
typedef kindr::VectorTypeless<double,18> GeneralizedCoordinates;
typedef kindr::VectorTypeless<double,18> GeneralizedVelocities;
typedef kindr::VectorTypeless<double,18> GeneralizedAccelerations;
typedef kindr::VectorTypeless<double,3> JointPositionsLimb;
typedef kindr::VectorTypeless<double,3> JointVelocitiesLimb;
typedef kindr::VectorTypeless<double,3> JointAccelerationsLimb;
typedef kindr::VectorTypeless<double,3> JointTorquesLimb;
typedef kindr::VectorTypeless<double,12> JointPositions;
typedef kindr::VectorTypeless<double,12> JointVelocities;
typedef kindr::VectorTypeless<double,12> JointAccelerations;
typedef kindr::VectorTypeless<double,12> JointTorques;


class QuadrupedModel
{
private:
    /* data */
public:
  QuadrupedModel(/* args */);
  ~QuadrupedModel();


    class QuadrupedDescription
    {
    private:
        /* data */
    public:
        QuadrupedDescription(/* args */){};
        virtual ~QuadrupedDescription();
        enum class LimbEnum
        {
            LF_LEG,
            RF_LEG,
            RH_LEG,
            LH_LEG,
        };

        enum class BranchEnum
        {
            BASE,
            LF_LEG,
            RF_LEG,
            LH_LEG,
            RH_LEG,
        };
        enum class JointNodeEnum
        {
            LF_LEG_HAA,
            LF_LEG_HFE,
            LF_LEG_KFE,
            RF_LEG_HAA,
            RF_LEG_HFE,
            RF_LEG_KFE,
            LH_LEG_HAA,
            LH_LEG_HFE,
            LH_LEG_KFE,
            RH_LEG_HAA,
            RH_LEG_HFE,
            RH_LEG_KFE,
        };

        static BranchEnum mapEnums(LimbEnum le)
        {
          int index = static_cast<int>(le);
          index = index +1;
          return static_cast<BranchEnum>(index);
        }
        static int getNumDofLimb()
        {
          return 3;
        }

        static int getLimbStartIndexInJ(LimbEnum limb)
        {
          switch (limb) {
            case LimbEnum::LF_LEG:
              return 0;
            case LimbEnum::RF_LEG:
              return 3;
            case LimbEnum::RH_LEG:
              return 6;
            case LimbEnum::LH_LEG:
              return 9;
          }
        }
    };
    
};

//QuadrupedModel::QuadrupedModel(/* args */)
//{
//}

//QuadrupedModel::~QuadrupedModel()
//{
//}
//class QuadrupedKinematics;
//class QuadrupedState //: public QuadrupedKinematics
//{
//public:
//  QuadrupedState();
//  ~QuadrupedState();
//  const Position getPositionWorldToBaseInWorldFrame() const;
//  const RotationQuaternion getOrientationBaseToWorld() const;
//  static JointPositions getJointPositions();
//  static JointVelocities getJointVelocities();
//  const LinearVelocity getLinearVelocityBaseInWorldFrame() const;
//  const LocalAngularVelocity getAngularVelocityBaseInBaseFrame() const;


//  bool setPoseBaseToWorld(const Pose pose);
//  bool setPositionWorldToBaseInWorldFrame(const Position position);
//  bool setOrientationBaseToWorld(const RotationQuaternion rotation);
//  bool setLinearVelocityBaseInWorldFrame(const LinearVelocity linear_velocity);
//  bool setAngularVelocityBaseInBaseFrame(const LocalAngularVelocity local_angular_velocity);
//  bool setJointPositions(const JointPositions);
//  bool setJointVelocities(const JointVelocities);
//private:
////  QuadrupedKinematics QK;
//  Pose poseInWorldFrame_;
//  static JointPositions joint_positions_;
//  static JointVelocities joint_velocities_;
//  Position positionWorldToBaseInWorldFrame_;
//};
}
