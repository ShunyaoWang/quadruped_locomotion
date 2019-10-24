#include "qp_solver/sequencequadraticproblemsolver.h"
namespace sqp_solver {
using namespace free_gait;
using namespace std;
SequenceQuadraticProblemSolver::SequenceQuadraticProblemSolver(std::shared_ptr<qp_solver::QuadraticProblemSolver>& quadratic_solver,
                                                               double tolerance, int max_iteration)
  : quadratic_solver_(quadratic_solver),
    tolerance_(tolerance),
    max_iteration_(max_iteration)
{
  std::cout<<"construct a sequence quadratic program solver class"<<std::endl;
};
SequenceQuadraticProblemSolver::~SequenceQuadraticProblemSolver()
{
  std::cout<<"destroy a sequence quadratic program solver class"<<std::endl;
}

bool SequenceQuadraticProblemSolver::minimize(const PoseOptimizationProblem& problem,
                                              PoseParameterization& params)
{
  std::cout<<"Sequence Quadratic minimizing......"<<std::endl;
  int k = 0;
  Eigen::MatrixXd H, A;
  Eigen::VectorXd G, b, b_max,d;
  Eigen::MatrixXd Aeq(params.getLocalSize(), 1);
  Eigen::VectorXd beq(1);
  Eigen::SparseMatrix<double, Eigen::RowMajor> Q, C, D;
  qp_solver::QuadraticProblemSolver::parameters result, p;
  qp_solver::QuadraticProblemSolver::Delta dp(params.getLocalSize());
//  free_gait::PoseOptimizationObjectiveFunction function = &problem.objectiveFunction_;
  // initial params

  problem.objectiveFunction_->getLocalHessian(H, params, false);
  problem.objectiveFunction_->getLocalGradient(G, params, false);
  problem.functionConstraints_->getLocalInequalityConstraintJacobian(A, params, false);
  problem.functionConstraints_->getInequalityConstraintMaxValues(b_max);
  problem.functionConstraints_->getInequalityConstraintValues(b, params, false);
  b = b_max - b;
  problem.objectiveFunction_->setGlobalHessian(H);
  problem.objectiveFunction_->setLinearTerm(G);
  problem.functionConstraints_->setGlobalInequalityConstraintJacobian(A);
  problem.functionConstraints_->setInequalityConstraintMaxValues(b);
  problem.functionConstraints_->setGlobalEqualityConstraintJacobian(Aeq.setZero());
  problem.functionConstraints_->setEqualityConstraintMaxValues(beq.setZero());

//  cout<<"Hessian Matrix is: "<<endl<<H<<endl;
//  cout<<"Jacobian Vector is: "<<endl<<G<<endl;
//  cout<<"Inequality Constraints Jacobian is: "<<endl<<A<<endl;
//  cout<<"Inequality Constraints value vector is: "<<endl<<b<<endl;

//  function.getLocalHessian(H, params, false);
//  function.getLocalGradient(G, params, false);
//  constraints.getLocalInequalityConstraintJacobian(A, params, false);
//  constraints.getInequalityConstraintMaxValues(b);

//  function.setGlobalHessian(H);
//  function.setLinearTerm(G);
//  constraints.setGlobalInequalityConstraintJacobian(A);
//  constraints.setInequalityConstraintMaxValues(b);
//  constraints.setGlobalEqualityConstraintJacobian(Aeq.setZero());
//  constraints.setEqualityConstraintMaxValues(beq.setZero());

  p = params.getParams();
  result = p;
  double cost = 1.0;
  while (k<max_iteration_) {
    k = k+1;
/**
  for QuadProg++
    min 0.5 * x H x + G x
    s.t.
        CE^T x + ce0 = 0
        CI^T x + ci0 >= 0

        CI = -A^T;
        ci0 = b;
        CE = Aeq;
        ce0 = beq;
        **/
//    quadratic_solver_->minimize(*problem.objectiveFunction_, *problem.functionConstraints_, dp);

    /**
      1/2 x'Qx + g'x,
      such that Cx = c and d <= Dx <= f
      C = Aeq^T
      c =-beq
      D =A;
      f =b
      **/
    Q = H.sparseView();
    C = Aeq.setZero().transpose().sparseView();
    D = A.sparseView();
    d = Eigen::VectorXd::Constant(b.size(), std::numeric_limits<double>::min());

    if(!ooqpei::OoqpEigenInterface::solve(Q, G, D, b, dp))
      {
        ROS_ERROR("Failed to Solve QP Subproblem With OOQP");
        return false;
      }
    params.plus(result, result, dp);
    params.setPose(Pose(Position(result.head(3)), RotationQuaternion(result.tail(4))));
    problem.objectiveFunction_->computeValue(cost, params, true);
    cout<<"dp norm at "<<k<<"iteration is : "<<dp.norm()<<endl;
    if(dp.norm() < 0.005)//tolerance_)
    {
      cout<<"Final cost :"<<cost<<endl;
      break;
    }

    // recompute params
    problem.objectiveFunction_->getLocalHessian(H, params, false);
    problem.objectiveFunction_->getLocalGradient(G, params, false);
    problem.functionConstraints_->getLocalInequalityConstraintJacobian(A, params, false);
    problem.functionConstraints_->getInequalityConstraintValues(b, params, false);
    problem.functionConstraints_->getInequalityConstraintMaxValues(b_max);

    b = b_max - b;

    problem.objectiveFunction_->setGlobalHessian(H);
    problem.objectiveFunction_->setLinearTerm(G);
    problem.functionConstraints_->setGlobalInequalityConstraintJacobian(A);
    problem.functionConstraints_->setInequalityConstraintMaxValues(b);
    problem.functionConstraints_->setGlobalEqualityConstraintJacobian(Aeq.setZero());
    problem.functionConstraints_->setEqualityConstraintMaxValues(beq.setZero());

//    cout<<"Hessian Matrix is: "<<endl<<H<<endl;
//    cout<<"Jacobian Vector is: "<<endl<<G<<endl;
//    cout<<"Inequality Constraints Jacobian is: "<<endl<<A<<endl;
//    cout<<"Inequality Constraints value vector is: "<<endl<<b<<endl;

  }
  cout<<"iterate times : "<<k<<endl;
  return true;
}
}// namespace sqp_solver
