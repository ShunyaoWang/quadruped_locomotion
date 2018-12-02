#pragma once

#include "kindr/Core"
#include <quadruped_model/common/typedefs.hpp>
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
  QuadrupedModel(/* args */){};
  ~QuadrupedModel(){};


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
            LH_LEG,
            RH_LEG,
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


    };
    
};

//QuadrupedModel::QuadrupedModel(/* args */)
//{
//}

//QuadrupedModel::~QuadrupedModel()
//{
//}

class QuadrupedState
{
public:
  QuadrupedState() {};
  ~QuadrupedState() {};
  Position getPositionWorldToBaseInWorldFrame()
  {
    return poseInWorldFrame_.getPosition();
  }
  bool setPoseBaseToWorld(const Pose pose)
  {
    poseInWorldFrame_ = pose;
    return true;
  }
  RotationQuaternion getOrientationBaseToWorld()
  {
    return  poseInWorldFrame_.getRotation();
  }
private:
  Pose poseInWorldFrame_;
  Position positionWorldToBaseInWorldFrame_;
};


}
