/*
 *  quadrupedkinematics.cpp
 *  Descriotion:
 *
 *  Created on: Mar 18, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#include "quadruped_model/quadrupedkinematics.h"
#include <ros/package.h>
namespace quadruped_model {
using namespace std;
QuadrupedKinematics::QuadrupedKinematics()
{
  // Load robot description

//  joint_positons_last_.resize(6);
//  for(unsigned int i = 0;i<6;i++)
//    joint_positons_last_(i) = 0.0;
//  string urdf_dir = ros::package::getPath("quadruped_model") + "/urdf/simpledog_m.urdf";
  string urdf_dir = ros::package::getPath("quadruped_model") + "/urdf/quadruped_model.urdf";
  LoadRobotDescriptionFromFile(urdf_dir);
  std::cout<<urdf_dir<<std::endl;
  //  LoadRobotDescriptionFromFile("/home/hitstar/catkin_ws/src/quadruped_locomotion-dev/quadruped_model/urdf/simpledog_m.urdf");
//  std::cout<<"Constructor QuadrupedKinematics"<<std::endl;
}

QuadrupedKinematics::~QuadrupedKinematics()
{
//  std::cout<<"QuadrupedKinematics Destroied"<<std::endl;
}
//QuadrupedKinematics& QuadrupedKinematics::operator=(const QuadrupedKinematics& other)
//{
//  tree_ = other.tree_;
//  hip_pose_in_base_ = other.hip_pose_in_base_;
////  lf_fk_solver_ptr = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(LF_Chain));//std::move(other.lf_fk_solver_ptr);
////  rf_fk_solver_ptr = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(RF_Chain));//std::move(other.rf_fk_solver_ptr);
////  rh_fk_solver_ptr = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(RH_Chain));//std::move(other.rh_fk_solver_ptr);
////  lh_fk_solver_ptr = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(LH_Chain));//std::move(other.lh_fk_solver_ptr);
//  return *this;
//}
QuadrupedKinematics::QuadrupedKinematics(const QuadrupedKinematics& other)
  : tree_(other.tree_),
    hip_pose_in_base_(other.hip_pose_in_base_),
    LF_Chain(other.LF_Chain),
    RF_Chain(other.RF_Chain),
    RH_Chain(other.RH_Chain),
    LH_Chain(other.LH_Chain)
{
//  cout<<"QuadrupedKinematics Class has been Copied"<<endl;
}


bool QuadrupedKinematics::LoadRobotDescriptionFromFile(const std::string filename)
{
  if(!kdl_parser::treeFromFile(filename, tree_))
  {
    ROS_ERROR("Failed to load robot description to KDL tree");
    return false;
  }  
//  tree_fk_solver_ = new(KDL::TreeFkSolverPos_recursive(tree_));
//  fk_solver_ptr = std::unique_ptr<KDL::TreeFkSolverPos_recursive>(new KDL::TreeFkSolverPos_recursive(tree_));
//  KDL::Chain chain;
//  KDL::ChainFkSolverPos_recursive fk = KDL::ChainFkSolverPos_recursive(chain);
//  KDL::TreeFkSolverPos_recursive fkt = KDL::TreeFkSolverPos_recursive(tree_);
// KDL::Chain LF_Chain, RF_Chain,RH_Chain,LH_Chain;
 tree_.getChain("base_link", "lf_foot_Link", LF_Chain);
 tree_.getChain("base_link", "rf_foot_Link", RF_Chain);
 tree_.getChain("base_link", "rh_foot_Link", RH_Chain);
 tree_.getChain("base_link", "lh_foot_Link", LH_Chain);
 setHipPoseInBase(LF_Chain,LimbEnum::LF_LEG);
 setHipPoseInBase(RF_Chain,LimbEnum::RF_LEG);
 setHipPoseInBase(RH_Chain,LimbEnum::RH_LEG);
 setHipPoseInBase(LH_Chain,LimbEnum::LH_LEG);
// cout<<"LF : "<<endl<<"T01 : "<<endl<<LF_Chain.getSegment(0).getFrameToTip()<<endl
//                    <<"T12 : "<<endl<<LF_Chain.getSegment(1).getFrameToTip()<<endl
//                    <<"T23 : "<<endl<<LF_Chain.getSegment(2).getFrameToTip()<<endl
//                    <<"T34 : "<<endl<<LF_Chain.getSegment(3).getFrameToTip()<<endl;
// cout<<"rF : "<<endl<<"T01 : "<<endl<<RF_Chain.getSegment(0).getFrameToTip()<<endl
//                    <<"T12 : "<<endl<<RF_Chain.getSegment(1).getFrameToTip()<<endl
//                    <<"T23 : "<<endl<<RF_Chain.getSegment(2).getFrameToTip()<<endl
//                    <<"T34 : "<<endl<<RF_Chain.getSegment(3).getFrameToTip()<<endl;

// hip_positions_in_base({{LimbEnum::LF_LEG, Position(0.4,0.175,0)},
//                        {LimbEnum::RF_LEG, Position(0.4, -0.175,0)}});

// lf_fk_splver_ptr = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(LF_Chain));

 // LF_with_vitural_foot_joints_Chain = LF_Chain;
// tree_.getChain("base_link", "front_left_3_Link", LF_with_vitural_foot_joints_Chain);
// LF_with_vitural_foot_joints_Chain.addSegment(KDL::Segment("foot_roll_link",KDL::Joint("foot_roll_joint",KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.25,0.0,-0.015))));
// LF_with_vitural_foot_joints_Chain.addSegment(KDL::Segment("foot_pitch_link",KDL::Joint("foot_pitch_joint",KDL::Joint::RotY)));
// LF_with_vitural_foot_joints_Chain.addSegment(KDL::Segment("foot_yaw_link",KDL::Joint("foot_yaw_joint",KDL::Joint::RotZ)));
// double pose_error = 1E-3;
// double joint_error = 1E-5;
// int max_iterations = 500;
//  lf_fk_solver_ptr = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(LF_Chain));
//  rf_fk_solver_ptr = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(RF_Chain));
//  lh_fk_solver_ptr = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(LH_Chain));
//  rh_fk_solver_ptr = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(RH_Chain));

// lf_ik_solver_ptr = std::unique_ptr<KDL::ChainIkSolverPos_LMA>(new KDL::ChainIkSolverPos_LMA(LF_with_vitural_foot_joints_Chain, pose_error,max_iterations,joint_error));
// LF_with_vitural_foot_joints_Chain.getSegment(3).getFrameToTip();
// cout<<LF_with_vitural_foot_joints_Chain.getSegment(0).getFrameToTip()<<endl;
//    <<LF_with_vitural_foot_joints_Chain.getSegment(4).getFrameToTip()<<endl
//   <<LF_with_vitural_foot_joints_Chain.getSegment(5).getFrameToTip()<<endl;
  return true;
}
bool QuadrupedKinematics::setHipPoseInBase(const KDL::Chain& kdl_chain, const LimbEnum& limb)
{
  KDL::Frame cartisian_frame;
  cartisian_frame = kdl_chain.getSegment(0).getFrameToTip();
  Position translation(cartisian_frame(0,3), cartisian_frame(1,3), cartisian_frame(2,3));
  RotationMatrix rotation_matrix(cartisian_frame(0,0), cartisian_frame(0,1), cartisian_frame(0,2),
                                 cartisian_frame(1,0), cartisian_frame(1,1), cartisian_frame(1,2),
                                 cartisian_frame(2,0), cartisian_frame(2,1), cartisian_frame(2,2));
//  cout<<"set hip pose in Base"<<endl;
  hip_pose_in_base_[limb] = Pose(translation, RotationQuaternion(rotation_matrix));
//  cout<<"hip pose in base : "<<hip_pose_in_base_[limb]<<endl;
  return true;

}

Position QuadrupedKinematics::getPositionFootToHipInHipFrame(const LimbEnum& limb, const Position& foot_position_in_base) const
{
//  Pose hip_to_base_in_base = hip_pose_in_base_.at(limb);
////  Eigen::Matrix4d transform = hip_to_base_in_base.getTransformationMatrix();
////  cout<<transform<<endl;
//  Position transformed_position;
//  transformed_position = hip_to_base_in_base.inverseTransform(foot_position_in_base);
//  return transformed_position;
  return hip_pose_in_base_.at(limb).inverseTransform(foot_position_in_base);
//  Eigen::VectorXd transformed_position(4), foot_position(4);
//  cout<<"foot_position = "<<endl<<foot_position_in_base<<endl;
//  foot_position << foot_position_in_base(0),foot_position_in_base(1),foot_position_in_base(2),1;
//  cout<<"foot_position = "<<endl<<foot_position<<endl;
//  transformed_position = transform * foot_position;
//  return Position(transformed_position(0),transformed_position(1),transformed_position(2));
//  hip_to_base_in_base.getTransformationMatrix()

}

bool QuadrupedKinematics::FowardKinematicsSolve(const JointPositionsLimb& joint_position,
                                                const LimbEnum& limb, Pose& cartisian_pose)
{
  int number_of_joints = joint_position.vector().size();
  KDL::JntArray joints = KDL::JntArray(number_of_joints);
  KDL::Frame cartisian_frame;
  for(int i = 0; i<number_of_joints; i++)
  {
    joints(i) = joint_position(i);
  }
  switch (limb) {
    case LimbEnum::LF_LEG:
      {
        KDL::ChainFkSolverPos_recursive lf_fk_solver(LF_Chain);
        if(lf_fk_solver.JntToCart(joints, cartisian_frame)<0)
        {
          cout<<"Failed to solve Forward kinematics problem"<<endl;
          return false;
        }
        break;
      }
    case LimbEnum::RF_LEG:
      {
//        cout<<"RF : "<<RF_Chain.getSegment(0).getFrameToTip()<<endl;
        KDL::ChainFkSolverPos_recursive rf_fk_solver(RF_Chain);
        if(rf_fk_solver.JntToCart(joints, cartisian_frame)<0)
        {
          cout<<"Failed to solve Forward kinematics problem"<<endl;
          return false;
        }
        break;
      }
    case LimbEnum::LH_LEG:
    {
      KDL::ChainFkSolverPos_recursive lh_fk_solver(LH_Chain);
      if(lh_fk_solver.JntToCart(joints, cartisian_frame)<0)
      {
        cout<<"Failed to solve Forward kinematics problem"<<endl;
        return false;
      }
      break;
    }
    case LimbEnum::RH_LEG:
     {
      KDL::ChainFkSolverPos_recursive rh_fk_solver(RH_Chain);
      if(rh_fk_solver.JntToCart(joints, cartisian_frame)<0)
      {
        cout<<"Failed to solve Forward kinematics problem"<<endl;
        return false;
      }
      break;
     }

  }

  Eigen::Vector3d translation(cartisian_frame(0,3), cartisian_frame(1,3), cartisian_frame(2,3));
  RotationMatrix rotation_matrix(cartisian_frame(0,0), cartisian_frame(0,1), cartisian_frame(0,2),
                                 cartisian_frame(1,0), cartisian_frame(1,1), cartisian_frame(1,2),
                                 cartisian_frame(2,0), cartisian_frame(2,1), cartisian_frame(2,2));

//  Eigen::Vector3d leg(0.25,0,0);
//  // something wrong with rotation matrix?
//  cout<<leg<<endl;
  cartisian_pose = Pose(Position(translation), RotationQuaternion(rotation_matrix));
  return true;
}

bool QuadrupedKinematics::AnalysticJacobian(const JointPositionsLimb& joint_positions, const LimbEnum& limb, Eigen::MatrixXd& jacobian)
{
  int number_of_joints = joint_positions.vector().size();
  KDL::JntArray joints = KDL::JntArray(number_of_joints);
  KDL::Jacobian J;
  J.resize(number_of_joints);
  for(int i = 0; i<number_of_joints; i++)
  {
    joints(i) = joint_positions(i);
//    cout<<"the "<<i<<"th joint: "<<joints(i)<<endl;
  }
//  cout<<LF_Chain.getNrOfJoints()<<" joint rows"<<joints.rows()<<" joint columns"<<joints.columns()<<endl;
//  cout<<joints.data<<endl;
  int error_code = 0;
  switch (limb) {
    case LimbEnum::LF_LEG:
      {
        KDL::ChainJntToJacSolver jacobian_solver(LF_Chain);
        error_code = jacobian_solver.JntToJac(joints, J);
        if(error_code != 0)
        {
          cout<<"Failed to solve Jacobian problem"<<" error code :"<<error_code<<endl;
          return false;
        }
        break;
      }
    case LimbEnum::RF_LEG:
      {
        KDL::ChainJntToJacSolver jacobian_solver(RF_Chain);
        if(jacobian_solver.JntToJac(joints, J) != 0)
        {
          cout<<"Failed to solve Jacobian problem"<<endl;
          return false;
        }
        break;
      }
    case LimbEnum::LH_LEG:
    {
      KDL::ChainJntToJacSolver jacobian_solver(LH_Chain);
      if(jacobian_solver.JntToJac(joints, J) != 0)
      {
        cout<<"Failed to solve Jacobian problem"<<endl;
        return false;
      }
      break;
    }
    case LimbEnum::RH_LEG:
     {
      KDL::ChainJntToJacSolver jacobian_solver(RH_Chain);
      if(jacobian_solver.JntToJac(joints, J) != 0)
      {
        cout<<"Failed to solve Jacobian problem"<<endl;
        return false;
      }
      break;
     }

  }

  jacobian = J.data;
//  cout<<jacobian<<endl;
//  cout<<"size of jacobian is (rows, cols) : "<<jacobian.rows()<<","<<jacobian.cols()<<endl;
//  for(int i = 0;)
  return true;
}
bool QuadrupedKinematics::AnalysticJacobianForLink(const JointPositionsLimb& joint_positions, const LimbEnum& limb, const int& link_index, Eigen::MatrixXd& jacobian)
{
  int number_of_joints = link_index;
  KDL::JntArray joints = KDL::JntArray(number_of_joints);
  KDL::Jacobian J;
  J.resize(number_of_joints);
  for(int i = 0; i<number_of_joints; i++)
  {
    joints(i) = joint_positions(i);
  }
  int error_code = 0;
  switch (limb) {
    case LimbEnum::LF_LEG:
      {
        KDL::Chain LF_Chain_Link;
        LF_Chain_Link.addSegment(LF_Chain.getSegment(0));
        cout<<LF_Chain.getSegment(0).getName()<<endl;
        for(int i = 1; i<link_index; i++)
          {
            KDL::Vector com = LF_Chain.getSegment(i).getInertia().getCOG();
            LF_Chain_Link.addSegment(KDL::Segment(LF_Chain.getSegment(i).getName(),
                                                  LF_Chain.getSegment(i).getJoint(),
                                                  KDL::Frame(com)));
          }
        KDL::ChainJntToJacSolver jacobian_solver(LF_Chain_Link);
        error_code = jacobian_solver.JntToJac(joints, J);
        if(error_code != 0)
        {
          cout<<"Failed to solve Jacobian problem"<<" error code :"<<error_code<<endl;
          return false;
        }
        break;
      }
    case LimbEnum::RF_LEG:
      {
        KDL::ChainJntToJacSolver jacobian_solver(RF_Chain);
        if(jacobian_solver.JntToJac(joints, J) != 0)
        {
          cout<<"Failed to solve Jacobian problem"<<endl;
          return false;
        }
        break;
      }
    case LimbEnum::LH_LEG:
    {
      KDL::ChainJntToJacSolver jacobian_solver(LH_Chain);
      if(jacobian_solver.JntToJac(joints, J) != 0)
      {
        cout<<"Failed to solve Jacobian problem"<<endl;
        return false;
      }
      break;
    }
    case LimbEnum::RH_LEG:
     {
      KDL::ChainJntToJacSolver jacobian_solver(RH_Chain);
      if(jacobian_solver.JntToJac(joints, J) != 0)
      {
        cout<<"Failed to solve Jacobian problem"<<endl;
        return false;
      }
      break;
     }

  }

  jacobian = J.data;
  return true;
}

//bool QuadrupedKinematics::InverseKinematicsSolve(Position foot_position, JointPositionsLimb& joint_positions)
//{
//  KDL::JntArray joints_out;
//  KDL::JntArray q(6);
//  KDL::JntArray q_init(6);
//  joints_out.resize(6);
//  KDL::Vector trans(foot_position(0),foot_position(1),foot_position(2));
//  KDL::Rotation rotation;
//  rotation.Identity();
//  KDL::Frame foot_pose(rotation, trans);
//  cout<<foot_pose<<endl;
//  q_init(3) = 0;
//  q_init(4) = 1.5;
//  q_init(5) = 1.5;
//  if(lf_ik_solver_ptr->CartToJnt(q_init, foot_pose, joints_out)<0)
//  {
//    cout<<"Failed to solve Inverse kinematics problem"<<endl;
//    return false;
//  }
//  for(int i=0;i<3;i++)
//  {
//    joint_positions(i) = joints_out(i);
//    cout<<joints_out(i+3);
//  }
//  return true;

//}

bool QuadrupedKinematics::InverseKinematicsSolve(const Position& foot_position, const LimbEnum& limb,
                                                 const JointPositionsLimb& joint_positions_last,
                                                 JointPositionsLimb& joint_positions,
                                                 const std::string LimbType)
{
  double d,l1,l2,px,py,pz,alpha,beta1,beta2;
  d=0.1;
  l1=0.35;
  l2=0.35;
//  cout<<"px in base = "<<foot_position(0)<<endl
//      <<"py in base = "<<foot_position(1)<<endl
//      <<"pz in base = "<<foot_position(2)<<endl;
  Position foot_position_in_hip = getPositionFootToHipInHipFrame(limb, foot_position);
  px=foot_position_in_hip(0);
  py=foot_position_in_hip(1);
  pz=foot_position_in_hip(2);
//  cout<<"px in hip = "<<px<<endl
//      <<"py in hip = "<<py<<endl
//      <<"pz in hip = "<<pz<<endl;
  double cos_theta3 = (l2*l2 + l1*l1 - ((px*px + py*py + pz*pz) - d*d))/2/l1/l2;
  if(cos_theta3<-1)
    cos_theta3 = -1;
  if(cos_theta3>1)
    cos_theta3 = 1;
  Eigen::VectorXd theta3(4);
  Eigen::MatrixXd results(4,3);

  theta3(0) = M_PI - acos(cos_theta3);
  theta3(1) = M_PI - acos(cos_theta3);
  theta3(2) = -M_PI + acos(cos_theta3);
  theta3(3) = -M_PI + acos(cos_theta3);

  alpha = atan2(py,px);
  beta1 = atan2(d,sqrt(fabs(px*px + py*py - d*d)));
  beta2 = atan2(-d,-sqrt(fabs(px*px + py*py - d*d)));
  int i = 0;
  while (i<4) {
    double a,b,q1,q2,q3;
    q3=MapToPI(theta3(i));
    // Left arm configure
    q1=MapToPI(alpha - beta1);
    a = atan2(pz,-sqrt(fabs(px*px + py*py - d*d)));
    b = atan2(l2*sin(q3), l1 + l2*cos(q3));
    if(a>0)
    {
      q2 = MapToPI(a - b - M_PI);
      results.row(i) << q1,q2,q3;
    }
    if(a<0)
    {
      q2 = MapToPI(a - b + M_PI);
      results.row(i) << q1,q2,q3;
    }
    //right arm config
    i = i +1;
    q1 = MapToPI(alpha + beta2);
    a = atan2(pz,sqrt(fabs(px*px + py*py - d*d)));
    q2 = MapToPI(a - b + M_PI);
    results.row(i) << q1,q2,q3;

    i=i+1;
  }
  double min_difference = 100;
  int min_index;
//  Eigen::MatrixXd v(1,3);
//  Eigen::MatrixXd w(1,3);
//  v<< 1,2,3;
//  w<< 1,1,1;
//  cout<<v.norm()<<endl;

  /*Eigen::VectorXd joint_last_vector;
  joint_last_vector.resize(3);
  joint_last_vector << joint_positions_last(0),joint_positions_last(1),joint_positions_last(2);


  for(int j = 0;j<4;j++)
  {
    Eigen::VectorXd da = joint_last_vector - results.row(j);
    if(da.norm()< min_difference)
    {
      min_difference = da.norm();
      min_index = j;
    }
//    cout<<da.normalized()<<endl;
    //    if(da.normalized())
  }*/

  if(LimbType == "IN_LEFT")
    min_index = 2;
  if(LimbType == "IN_RIGHT")
    min_index = 1;
  if(LimbType == "OUT_LEFT")
    min_index = 0;
  if(LimbType == "OUT_RIGHT")
    min_index = 3;

  joint_positions << results(min_index,0),results(min_index,1),results(min_index,2);
//  cout<<results<<endl;

  if(!isnan(joint_positions(0))&&!isnan(joint_positions(1))&&!isnan(joint_positions(2))){
      return true;
    }else{
      ROS_WARN("Failed to Sovle Inverse Kinematics!");
      return false;
    }

}

JointTorquesLimb QuadrupedKinematics::getGravityCompensationForLimb(const LimbEnum& limb,
                                               const JointPositionsLimb& joint_positions,
                                               const Force& gravity_in_baseframe)
{
  KDL::Vector gravity_vector = KDL::Vector(gravity_in_baseframe(0),
                                           gravity_in_baseframe(1),
                                           gravity_in_baseframe(2));
  int number_of_joints = 3;
  KDL::JntArray joints = KDL::JntArray(number_of_joints);
  KDL::JntArray gravity_matrix = KDL::JntArray(number_of_joints);
  for(int i = 0; i<number_of_joints; i++)
  {
    joints(i) = joint_positions(i);
  }
  int error_code = 0;
  switch (limb) {
    case LimbEnum::LF_LEG:
      {
        KDL::ChainDynParam dynamic_param = KDL::ChainDynParam(LF_Chain, gravity_vector);
        error_code = dynamic_param.JntToGravity(joints, gravity_matrix);
        if(error_code != 0)
        {
          cout<<"Failed to solve Jacobian problem"<<" error code :"<<error_code<<endl;
        }
        break;
      }
    case LimbEnum::RF_LEG:
      {
        KDL::ChainDynParam dynamic_param = KDL::ChainDynParam(RF_Chain, gravity_vector);
        error_code = dynamic_param.JntToGravity(joints, gravity_matrix);
        if(error_code != 0)
        {
          cout<<"Failed to solve Jacobian problem"<<endl;
        }
        break;
      }
    case LimbEnum::LH_LEG:
    {
        KDL::ChainDynParam dynamic_param = KDL::ChainDynParam(LH_Chain, gravity_vector);
        error_code = dynamic_param.JntToGravity(joints, gravity_matrix);
        if(error_code != 0)
      {
        cout<<"Failed to solve Jacobian problem"<<endl;
      }
      break;
    }
    case LimbEnum::RH_LEG:
     {
        KDL::ChainDynParam dynamic_param = KDL::ChainDynParam(RH_Chain, gravity_vector);
        error_code = dynamic_param.JntToGravity(joints, gravity_matrix);
        if(error_code != 0)
      {
        cout<<"Failed to solve Jacobian problem"<<endl;
      }
      break;
     }

  }

  JointTorquesLimb gravity_compensation_torque;
  for(unsigned int i =0;i<number_of_joints;i++)
    gravity_compensation_torque(i) = gravity_matrix(i);

  return gravity_compensation_torque;



}

double QuadrupedKinematics::MapToPI(double q)
{
  double out;
  out = q;
  if(q>M_PI)
    out = 2*M_PI - q;
  if(q<-M_PI)
    out = 2*M_PI + q;
  return out;
}

Position QuadrupedKinematics::getPositionBaseToHipInBaseFrame(const LimbEnum& limb) const
{
  Position hip_position_in_base = hip_pose_in_base_.at(limb).getPosition();
  hip_position_in_base.z() = 0.0;
  return hip_position_in_base;
//  switch (limb) {
//    case LimbEnum::LF_LEG:
//      return Position(0.42, 0.075, 0.0);
//    case LimbEnum::RF_LEG:
//      return Position(0.42, -0.075, 0.0);
//    case LimbEnum::LH_LEG:
//      return Position(-0.42, 0.075, 0.0);
//    case LimbEnum::RH_LEG:
//      return Position(-0.42, -0.075, 0.0);
//    default:
//      throw std::runtime_error("QuadrupedKinematics::getPositionBaseToHipInBaseFrame something went wrong.");
//  }
}
}//namespace
