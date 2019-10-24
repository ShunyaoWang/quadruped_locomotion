#include "qp_solver/quadraticproblemsolver.h"
#include "grid_map_core/Polygon.hpp"
namespace qp_solver {

QuadraticProblemSolver::QuadraticProblemSolver()
    : variable_dimension_(0)
{
//  grid_map::Polygon pg;
//  std::cout<<pg.nVertices()<<std::endl;
//  G_.resize(variable_dimension_,variable_dimension_);
//  CE_.resize(variable_dimension_,variable_dimension_);
//  CI_.resize(variable_dimension_,variable_dimension_);
//  g0_.resize(variable_dimension_);
//  ce0_.resize(variable_dimension_);
//  ci0_.resize(variable_dimension_);
//  x_.resize(variable_dimension_);
  std::cout<<"construct a quadratic program solver class"<<std::endl;

};
QuadraticProblemSolver::~QuadraticProblemSolver()
{
  std::cout<<"quadratic program solver class destroyed"<<std::endl;
}
bool QuadraticProblemSolver::setGlobalHessian(Eigen::MatrixXd& hessian)
{
  throw std::runtime_error("QuadraticProblemSolver::setGlobalHessian() not implemented.");
};
bool QuadraticProblemSolver::setLinearTerm(Eigen::VectorXd& jacobian)
{
  throw std::runtime_error("QuadraticProblemSolver::setLinearTerm() not implemented.");
};
bool QuadraticProblemSolver::setGlobalInequalityConstraintJacobian(Eigen::MatrixXd& A)
{
  throw std::runtime_error("QuadraticProblemSolver::setGlobalInequalityConstraintJacobian() not implemented.");
};
bool QuadraticProblemSolver::setGlobalEqualityConstraintJacobian(Eigen::MatrixXd& Aeq)
{
  throw std::runtime_error("QuadraticProblemSolver::setGlobalEqualityConstraintJacobian() not implemented.");
};
bool QuadraticProblemSolver::setInequalityConstraintMaxValues(Eigen::VectorXd& b)
{
  throw std::runtime_error("QuadraticProblemSolver::setInequalityConstraintMaxValues() not implemented.");
};
bool QuadraticProblemSolver::setEqualityConstraintMaxValues(Eigen::VectorXd& beq)
{
  throw std::runtime_error("QuadraticProblemSolver::setEqualityConstraintMaxValues() not implemented.");
};
bool QuadraticProblemSolver::getCoefficients(quadprogpp::Matrix<double>& G, quadprogpp::Vector<double>& g0,
                                             quadprogpp::Matrix<double>& CE, quadprogpp::Vector<double>& ce0,
                                             quadprogpp::Matrix<double>& CI, quadprogpp::Vector<double>& ci0,
                                             quadprogpp::Vector<double>& x)
{

  G = G_;
  std::cout<<"get coefficients"<<std::endl;
  g0 = g0_;
  CE = CE_;
  ce0 = ce0_;
  CI = CI_;
  ci0 = ci0_;
  x = x_;

};

bool QuadraticProblemSolver::minimize(const qp_solver::QuadraticObjectiveFunction& function,
                                      const qp_solver::LinearFunctionConstraints& constraints,
                                      qp_solver::QuadraticProblemSolver::parameters& params)
{

  int n = static_cast<int>(params.size());
  quadprogpp::Matrix<double> G, CE, CI;
  quadprogpp::Vector<double> g0, ce0, ci0, x;
//  std::cout<<"in minimaze"<<std::endl;
  x.resize(n);
//  std::cout<<"got here"<<variable_dimension_<<std::endl;
  for(int i = 0;i<n;i++){
    x[i] = params(i);
//    std::cout<<"test"<<x[i]<<std::endl;
  }

//  setx_(x);
  G = function.G_;
  g0 = function.g0_;
  CE = constraints.CE_;
  ce0 = constraints.ce0_;
  CI = constraints.CI_;
  ci0 = constraints.ci0_;
//  getCoefficients(G, g0, CE, ce0, CI, ci0, x);

  Eigen::VectorXd c, b, f, d, p;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Q, A, D;

//  toEigenMatrix(G,Q);
//  toEigenMatrix(CE,A);
//  toEigenMatrix(CI,D);
//  toEigenVector(g0,c);
//  toEigenVector(ce0,b);
//  toEigenVector(ci0,f);
//  d=f;
//  d.setZero();



  Q = function.hessian_;
  c = function.jacobian_;
  A = constraints.Aeq_;
  b = constraints.beq_;
  D = constraints.A_;
  f = constraints.b_;
  d = constraints.b_;
  d = -std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(D.rows());
//  p.resize(Q.rows());
  /**
    1/2 x'Qx + c'x,
    such that Ax = b and d <= Dx <= f
    C = Aeq^T
    c =-beq
    D =A;
    f =b
    **/

//    std::cout<<"G : "<<std::endl<<G<<std::endl;
//    std::cout<<"g0 : "<<std::endl<<g0<<std::endl;
//    std::cout<<"CE : "<<std::endl<<CE<<std::endl;
//    std::cout<<"ce0 : "<<std::endl<<ce0<<std::endl;
//    std::cout<<"CI : "<<std::endl<<CI<<std::endl;
//    std::cout<<"ci0: "<<std::endl<<ci0<<std::endl;

//  std::cout<<"Q : "<<std::endl<<Q<<std::endl;
//  std::cout<<"c : "<<std::endl<<c<<std::endl;
//  std::cout<<"A : "<<std::endl<<A<<std::endl;
//  std::cout<<"b : "<<std::endl<<b<<std::endl;
//  std::cout<<"D : "<<std::endl<<D<<std::endl;
//  std::cout<<"f : "<<std::endl<<f<<std::endl;
////  ooqpei::OoqpEigenInterface::setIsInDebugMode(true);
//  p = params;
//  if(!ooqpei::OoqpEigenInterface::solve(Q, c, D, f, p))
//    {
//      ROS_ERROR("Failed to Solve QP problem With OOQP");
//      return false;
//    }
//  params = p;
//  ooqpei::OoqpEigenInterface::setIsInDebugMode(true);

//  if(!ooqpei::OoqpEigenInterface::solve(Q, c, A, b, D, d, f, params))
//    {
//      ROS_ERROR("Failed to Solve QP problem With OOQP");
//    }
//  if(!ooqpei::OoqpEigenInterface::solve(hessian_, jacobian_, Aeq_.transpose(), -beq_, A_, d, b_, params))
//    {
//      ROS_ERROR("Failed to Solve QP problem With OOQP");
//    }


  /**
    for QuadProg++
min 0.5 * x G x + g0 x
s.t.
    CE^T x + ce0 = 0
    CI^T x + ci0 >= 0

          CI = -A^T;
          ci0 = b;
          CE = Aeq;
          ce0 = beq;
          **/

  quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
  for(int i = 0;i<n;i++){
    params(i) = x[i];
//    setx_(x);
  }

  return true;
};

//void QuadraticProblemSolver::toEigenMatrix(const quadprogpp::Matrix<double> mat_in, Eigen::MatrixXd& mat_out)
//{
//  int m = mat_in.nrows();
//  int n = mat_in.ncols();
//  for (int i = 0; i < m; i++)
//    for (int j = 0; j < n; j++)
//      mat_out(i,j) = mat_in[i][j];
//}
//void QuadraticProblemSolver::toEigenVector(const quadprogpp::Vector<double> vec_in, Eigen::VectorXd& vec_out)
//{

//  int n = vec_in.size();
//  for (int j = 0; j < n; j++)
//    vec_out(j) = vec_in[j];
//}

void QuadraticProblemSolver::setG_(Eigen::MatrixXd& G)
{
  hessian_ = G.sparseView();
}
void QuadraticProblemSolver::setCE_(Eigen::MatrixXd& CE)
{
  Aeq_ = CE.transpose().sparseView();
}
void QuadraticProblemSolver::setCI_(Eigen::MatrixXd& CI)
{
  A_ = CI.sparseView();
}
void QuadraticProblemSolver::setg0_(Eigen::VectorXd& g0)
{
  jacobian_ = g0;
}
void QuadraticProblemSolver::setce0_(Eigen::VectorXd& ce0)
{
  beq_ = -ce0;
}
void QuadraticProblemSolver::setci0_(Eigen::VectorXd& ci0)
{
  b_ = ci0;
}

void QuadraticProblemSolver::setG_(quadprogpp::Matrix<double>& G)
{
  G_=G;
}
void QuadraticProblemSolver::setCE_(quadprogpp::Matrix<double>& CE)
{
  CE_ = CE;
}
void QuadraticProblemSolver::setCI_(quadprogpp::Matrix<double>& CI)
{
  CI_ = CI;
}
void QuadraticProblemSolver::setg0_(quadprogpp::Vector<double>& g0)
{
  g0_ = g0;
}
void QuadraticProblemSolver::setce0_(quadprogpp::Vector<double>& ce0)
{
  ce0_ = ce0;
}
void QuadraticProblemSolver::setci0_(quadprogpp::Vector<double>& ci0)
{
  ci0_ = ci0;
}
void QuadraticProblemSolver::setx_(quadprogpp::Vector<double>& x)
{
  x_ = x;
}


QuadraticObjectiveFunction::QuadraticObjectiveFunction()
{};
QuadraticObjectiveFunction::~QuadraticObjectiveFunction()
{};
bool QuadraticObjectiveFunction::setGlobalHessian(Eigen::MatrixXd& hessian)
{
  int n = static_cast<int>(hessian.cols());
  setG_(hessian);
  quadprogpp::Matrix<double> G;
  G.resize(n, n);
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++)
      G[i][j] = hessian(i,j);
  setG_(G);
  return true;
}

bool QuadraticObjectiveFunction::setLinearTerm(Eigen::VectorXd& jacobian)
{
  setg0_(jacobian);
  int n = static_cast<int>(jacobian.size());
//  quadprogpp::Vector<double> g0;
  g0_.resize(n);
  for (int i = 0; i < n; i++){
    g0_[i] = jacobian(i);
//    std::cout<<g0_[i]<<std::endl;
  }
//  setg0_(g0);
  return true;
}

LinearFunctionConstraints::LinearFunctionConstraints()
{};
LinearFunctionConstraints::~LinearFunctionConstraints()
{};
bool LinearFunctionConstraints::setGlobalInequalityConstraintJacobian(Eigen::MatrixXd& A)
{
  setCI_(A);
  Eigen::MatrixXd A_T = -A.transpose();
  int m = static_cast<int>(A_T.rows());
  int n = static_cast<int>(A_T.cols());
  quadprogpp::Matrix<double> CI;
  CI.resize(m, n);
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      CI[i][j] = A_T(i, j);
  setCI_(CI);

}
bool LinearFunctionConstraints::setGlobalEqualityConstraintJacobian(Eigen::MatrixXd& Aeq)
{
  setCE_(Aeq);
  int m = static_cast<int>(Aeq.rows());
  int n = static_cast<int>(Aeq.cols());
  quadprogpp::Matrix<double> CE;
  CE.resize(m, n);
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      CE[i][j] = Aeq(i, j);
  setCE_(CE);
  return true;
}
bool LinearFunctionConstraints::setInequalityConstraintMaxValues(Eigen::VectorXd& b)
{
  setci0_(b);
  int n = static_cast<int>(b.size());
  quadprogpp::Vector<double> ci0;
  ci0.resize(n);
  for (int j = 0; j < n; j++)
    ci0[j] = b(j);
  setci0_(ci0);
  return true;

}
bool LinearFunctionConstraints::setEqualityConstraintMaxValues(Eigen::VectorXd& beq)
{
  setce0_(beq);
  int n = static_cast<int>(beq.size());
  quadprogpp::Vector<double> ce0;
  ce0.resize(n);
  for (int j = 0; j < n; j++)
    ce0[j] = beq(j);
  setce0_(ce0);
  return true;
}

}// namespace qp_solver
