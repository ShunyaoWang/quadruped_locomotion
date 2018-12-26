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
//  std::cout<<"call quagprog++"<<std::endl;
  quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
//  std::cout<<"got here"<<std::endl;
  for(int i = 0;i<n;i++){
    params(i) = x[i];
//    setx_(x);
  }
  return true;
};

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
  int n = static_cast<int>(beq.size());
  quadprogpp::Vector<double> ce0;
  ce0.resize(n);
  for (int j = 0; j < n; j++)
    ce0[j] = beq(j);
  setce0_(ce0);
  return true;
}

}// namespace qp_solver
