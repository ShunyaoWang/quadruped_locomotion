#include "qp_solver/quadraticproblemsolver.h"
#include "free_gait_core/pose_optimization/PoseOptimizationQP.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationGeometric.hpp"
#include "free_gait_core/pose_optimization/PoseConstraintsChecker.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationSQP.hpp"
#include "free_gait_core/TypeDefs.hpp"
#include "AdapterDummy.hpp"
//#include "Eigen/QR"
#include "iostream"
#include <memory>
//#include "nlopt.h"
#include "grid_map_core/Polygon.hpp"
#include <kindr/Core>
using namespace qp_solver;
using namespace std;
using namespace free_gait;
//double FunctionValue(E)

int main(int argc, char *argv[])
{

  QuadraticProblemSolver::parameters params;
  int x_demension = 3;
  int ieq_demension = 3;
  int eq_demension = 1;
  Eigen::MatrixXd H, A, Aeq;
  Eigen::VectorXd G, b, beq;
  params.resize(x_demension);
  params.setIdentity();
  H.resize(x_demension,x_demension);
  A.resize(x_demension,ieq_demension);
  Aeq.resize(x_demension,eq_demension);
  G.resize(x_demension);
  b.resize(ieq_demension);
  beq.resize(eq_demension);
  cout<<"min(1/2*x^t*H*x + G^t*x)"<<endl;
  cout<<"subject to :"<<endl<<"Aeq*beq = 0,"<<endl<<"A*b <=0"<<endl;
  cout<<"x is "<<x_demension<<"demension vector"<<endl;
  H << 8, 0, 0,
       0, 8, 0,
       0, 0, 10;
  cout<<"Hessian Matrix is: "<<endl<<H<<endl;
  G << -2.6327, -1.4383, 0.5;
  cout<<"Jacobian Vector is: "<<endl<<G<<endl;

  A << -1.0531, -0.5753, 0.0,
       1.4383, -2.6327, 0.0,
       -0.3852, 3.2018, 0.0;
  A=-A;
  cout<<"Inequality Constraints Jacobian is: "<<endl<<A<<endl;
  Aeq << 0.0,
         0.0,
         0.0;
  cout<<"Equality Constraints Jacobian is: "<<endl<<Aeq<<endl;
  b << 1.2, 1.5, 0.3;
  cout<<"Inequality Constraints value vector is: "<<endl<<b<<endl;
  beq << 0.0;
  cout<<"Equality Constraints value vector is: "<<endl<<beq<<endl;
  std::unique_ptr<int> up;
  std::unique_ptr<qp_solver::QuadraticProblemSolver> solver;
  auto costFunction = std::shared_ptr<qp_solver::QuadraticObjectiveFunction>(new qp_solver::QuadraticObjectiveFunction());
  auto constraints = std::shared_ptr<qp_solver::LinearFunctionConstraints>(new qp_solver::LinearFunctionConstraints());
//  costFunction->setGlobalHessian(H);
//  costFunction->setLinearTerm(G);
//  constraints->setGlobalEqualityConstraintJacobian(Aeq);
//  constraints->setGlobalInequalityConstraintJacobian(A);
//  constraints->setEqualityConstraintMaxValues(beq);
//  constraints->setInequalityConstraintMaxValues(b);

//  solver->minimize(*costFunction, *constraints, params);
//  std::cout<<"solve result x is: "<<endl<<params<<std::endl;



  AdapterDummy adapter;
  Pose result;
  LimbEnum le = LimbEnum::RF_LEG;
//  cout<<le<<endl;
  int le_int = static_cast<int>(le);
  BranchEnum be = static_cast<BranchEnum>(le_int + 1);
  BranchEnum be_map = quadruped_model::QuadrupedModel::QuadrupedDescription::mapEnums(le);
//  cout<<be<<endl;
  result = Pose(Position(0.1, 0.1, 0), RotationQuaternion());
  PoseOptimizationBase::LimbLengths minLimbLenghts_, maxLimbLenghts_;
  double supportMargin_ = 0.0;
  double base_height = 0.3;
  double nominal_height = 0.0;
  Stance nominal_stance, stance, support_stance, stance_for_orientation,
      footholdsOfNextLegMotion_,footholdsInSupport_;
//  nominal_stance = Stance({
//                            {LimbEnum::LF_LEG, Position(1.0, 0.5, -0.4)},
//                            {LimbEnum::RF_LEG, Position(1.0, -0.5, -0.4)},
//                            {LimbEnum::LH_LEG, Position(-1.0, -0.5, -0.4)},
//                            {LimbEnum::RH_LEG, Position(-1.0, 0.5, -0.4)} });


  Stance footPositions;
  kindr::EulerAnglesZyxPD rotation(0.0, 0.0, 0.0);
  Position base_position(0,0,base_height);
  footPositions[LimbEnum::LF_LEG] = base_position + rotation.rotate(Position( 0.3,  0.2, -base_height));
  footPositions[LimbEnum::RF_LEG] = base_position + rotation.rotate(Position(0.3,  -0.2, -base_height));
  footPositions[LimbEnum::LH_LEG] = base_position + rotation.rotate(Position(-0.3, 0.2, -base_height));
  footPositions[LimbEnum::RH_LEG] = base_position + rotation.rotate(Position(-0.3, -0.2, -base_height));
  for(const auto& foot:footPositions)
  {
    nominal_height += foot.second(2);
  }
  nominal_height = nominal_height/4 + base_height;
  cout<<"nominal hieght is : "<<nominal_height<<endl;
  for(const auto& limb:adapter.getLimbs())
  {
    //!robot base size is 0.6m X 0.4m
    nominal_stance[limb] = Position(adapter.getPositionBaseToHipInBaseFrame(limb)(0),
                                    adapter.getPositionBaseToHipInBaseFrame(limb)(1),
                                    -nominal_height);
  }

  //!foothold to reach
  stance = footPositions;
  stance_for_orientation = footPositions;
  //!foothold in support
//  support_stance[LimbEnum::LF_LEG] = footPositions[LimbEnum::LF_LEG];
  support_stance[LimbEnum::RF_LEG] = footPositions[LimbEnum::RF_LEG];
  support_stance[LimbEnum::LH_LEG] = footPositions[LimbEnum::LH_LEG];
  support_stance[LimbEnum::RH_LEG] = footPositions[LimbEnum::RH_LEG];


  // Define support region.
  grid_map::Polygon supportRegion;
  std::vector<Position> footholdsOrdered;
  getFootholdsCounterClockwiseOrdered(support_stance, footholdsOrdered);
  for (auto foothold : footholdsOrdered) {
    supportRegion.addVertex(foothold.vector().head<2>());
  }
  bool isLinePolygon = false;
  // if there are only 2 stance leg, treat as a line with 0.001m witdth
  if (supportRegion.nVertices() == 2) {
    supportRegion.thickenLine(0.001);
    isLinePolygon = true;
  }
  // offsets the support region
  if (!isLinePolygon) supportRegion.offsetInward(supportMargin_);

  // Define min./max. leg lengths.
  for (const auto& limb : adapter.getLimbs()) {
    minLimbLenghts_[limb] = 0.2; // TODO Make as parameters.
    if (footholdsOfNextLegMotion_.find(limb) == footholdsOfNextLegMotion_.end()) {
      maxLimbLenghts_[limb] = 0.545; // Foot stays in contact. // 0.57
    } else {
      maxLimbLenghts_[limb] = 0.565; // Foot leaves contact. // 0.6
    }
  }

  //test with PoseOptimizationGeometric
  PoseOptimizationGeometric optimization_geometric(adapter);
  optimization_geometric.setNominalStance(nominal_stance);//fix x,y, z is changed with the average leg height
  optimization_geometric.setStance(stance);// compute orientation (quaternion) with out any pose optimization
  optimization_geometric.setSupportRegion(supportRegion);
  optimization_geometric.setSupportStance(support_stance);// compute center pose (x,y)
  optimization_geometric.setStanceForOrientation(stance_for_orientation);//computer YAW angle

  optimization_geometric.optimize(result);
  EulerAnglesZyx eular_zyx(result.getRotation());

  cout<<"Geometric optimizaion result:"<<endl<<result.getPosition()<<endl<<
        "Rotation: "<<endl<<"Roll: "<<eular_zyx.roll()<<endl<<"Pitch: "<<
        eular_zyx.pitch()<<endl<<"Yaw: "<<eular_zyx.yaw()<<endl;


  // test with PoseOptimizationQP
  PoseOptimizationQP optimization(adapter);

  optimization.setNominalStance(nominal_stance);
  optimization.setStance(stance);
  optimization.setSupportRegion(supportRegion);
  optimization.setSupportStance(support_stance);

  optimization.optimize(result);
  eular_zyx(result.getRotation());
  cout<<"QP optimization result:"<<endl<<result.getPosition()<<endl<<
        "Rotation: "<<endl<<"Roll: "<<eular_zyx.roll()<<endl<<"Pitch: "<<
        eular_zyx.pitch()<<endl<<"Yaw: "<<eular_zyx.yaw()<<endl;

  PoseConstraintsChecker constraints_checker(adapter);
  constraints_checker.setNominalStance(nominal_stance);
  constraints_checker.setStance(stance);
  constraints_checker.setSupportStance(support_stance);
  constraints_checker.setSupportRegion(supportRegion);
  constraints_checker.setTolerances(0.02, 0.0);
  constraints_checker.setLimbLengthConstraints(minLimbLenghts_, maxLimbLenghts_);

  bool is_satisfied = constraints_checker.check(result);
  cout<<"check result of constraints checker : "<<is_satisfied<<endl;

  // test with SQP
  PoseOptimizationSQP sqp_optimization(adapter);
  sqp_optimization.setNominalStance(nominal_stance);
  sqp_optimization.setStance(stance);
  sqp_optimization.setSupportStance(support_stance);
  sqp_optimization.setSupportRegion(supportRegion);
  sqp_optimization.setLimbLengthConstraints(minLimbLenghts_, maxLimbLenghts_);
  Pose result2;
  sqp_optimization.optimize(result);
  eular_zyx(result.getRotation());
  cout<<"SQP optimization result:"<<endl<<result.getPosition()<<endl<<
        "Rotation: "<<endl<<"Roll: "<<eular_zyx.roll()<<endl<<"Pitch: "<<
        eular_zyx.pitch()<<endl<<"Yaw: "<<eular_zyx.yaw()<<endl;

 is_satisfied = constraints_checker.check(result);
  cout<<"check result of constraints checker : "<<is_satisfied<<endl;

  return 0;
}
