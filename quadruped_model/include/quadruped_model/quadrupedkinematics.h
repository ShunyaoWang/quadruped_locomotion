#ifndef QUADRUPEDKINEMATICS_H
#define QUADRUPEDKINEMATICS_H

#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
//#include "kdl/chainiksolver.hpp"
//#include "kdl/chainiksolverpos_nr.hpp"
//#include "kdl/chainiksolverpos_lma.hpp"
//#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/frames.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/tree.hpp"
//#include "kdl/chainiksolvervel_wdls.hpp"


#include "kindr/Core"
#include "quadruped_model/QuadrupedModel.hpp"
#include "quadruped_model/common/typedefs.hpp"
//#include "free_gait_core/TypeDefs.hpp"

#include "iostream"
#include "string.h"
#include "ros/ros.h"
#include "memory"
#include "unordered_map"
#include "utility"

using namespace romo;
namespace quadruped_model
{
//struct EnumClassHash
//{
//  template<typename T>
//  std::size_t operator()(T t) const
//  {
//    return static_cast<std::size_t>(t);
//  }
//};

using QD = quadruped_model::QuadrupedModel::QuadrupedDescription;

using LimbEnum = QD::LimbEnum;
using BranchEnum = QD::BranchEnum;
using JointNodeEnum = QD::JointNodeEnum;
typedef std::unordered_map<LimbEnum, Pose, EnumClassHash> HipPoseInBase;
class QuadrupedKinematics
{
public:
//  typedef std::unordered_map<LimbEnum, Pose, EnumClassHash> HipPoseInBase;

  QuadrupedKinematics();
  ~QuadrupedKinematics();
  QuadrupedKinematics(const QuadrupedKinematics& other);
//  QuadrupedKinematics& operator=(const QuadrupedKinematics& other);
//  std::unique_ptr<KDL::ChainFkSolverPos_recursive> clone(s)
  bool LoadRobotDescriptionFromFile(const std::string filename);
  bool FowardKinematicsSolve(const JointPositionsLimb& joint_position, const LimbEnum& limb, Pose& cartisian_pose);
//  bool InverseKinematicsSolve(Position foot_position, JointPositionsLimb& joint_positions);
  bool InverseKinematicsSolve(const Position& foot_position, const LimbEnum& limb,
                              const JointPositionsLimb& joint_positions_last,
                              JointPositionsLimb& joint_positions,
                              const std::string LimbType = "IN_LEFT");
  bool AnalysticJacobian(const JointPositionsLimb& joint_positions, const LimbEnum& limb, Eigen::MatrixXd& jacobian);
  double MapToPI(double q);

  bool setHipPoseInBase(const KDL::Chain& kdl_chain, const LimbEnum& limb);
  Position getPositionFootToHipInHipFrame(const LimbEnum& limb, const Position& foot_position_in_base) const;
  Position getPositionBaseToHipInBaseFrame(const LimbEnum& limb) const;

  KDL::Chain LF_Chain, RF_Chain,RH_Chain,LH_Chain;
  HipPoseInBase hip_pose_in_base_;
private:
  KDL::Tree tree_;
  KDL::JntArray joint_positons_last_;
//  HipPoseInBase hip_pose_in_base_;
//  KDL::Chain LF_Chain, RF_Chain,RH_Chain,LH_Chain;
//  KDL::TreeFkSolverPos_recursive tree_fk_solver_;
//  std::unique_ptr<KDL::ChainFkSolverPos_recursive> lf_fk_solver_ptr, rf_fk_solver_ptr, rh_fk_solver_ptr, lh_fk_solver_ptr;
//  std::unique_ptr<KDL::ChainIkSolverPos_LMA> lf_ik_solver_ptr;
//  std::unique_ptr<KDL::ChainIkSolverPos_NR> lf_ik_solver_nr_ptr;

};
}//namespace
#endif // QUADRUPEDKINEMATICS_H
