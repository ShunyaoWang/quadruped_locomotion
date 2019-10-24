/*
 *  filename.cpp
 *  Descriotion:
 *
 *  Created on: Nov 27, 2018
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#ifndef QUADRATICPROBLEMSOLVER_H
#define QUADRATICPROBLEMSOLVER_H

#include "qp_solver/QuadProg++.h"
#include "ooqp_eigen_interface/QuadraticProblemFormulation.hpp"
#include "ooqp_eigen_interface/OoqpEigenInterface.hpp"
#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <string>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "kindr/Core"
namespace qp_solver {
/**
 * @brief The QuadraticProblemSolver class
 * The problem is in the form:

    min 0.5 * x G x + g0 x
    s.t.
        CE^T x + ce0 = 0
        CI^T x + ci0 >= 0

     The matrix and vectors dimensions are as follows:
         G: n * n
        g0: n

        CE: n * p
       ce0: p

        CI: n * m
       ci0: m

         x: n
 */

class QuadraticObjectiveFunction;
class LinearFunctionConstraints;
//using namespace quadprogpp;
class QuadraticProblemSolver
{
public:
  class Param : public Eigen::VectorXd
  {
  public:
    Param() {};
    ~Param(){};
    kindr::Position3D head(size_t len) const
    {
      return kindr::Position3D(Param_(len - 3),Param_(len - 2),Param_(len - 1));
    }
    kindr::RotationQuaternionD tail(size_t len) const
    {
      return kindr::RotationQuaternionD(Param_(len - 1),Param_(len),Param_(len + 1),Param_(len + 2));
    }
    void resize(size_t n)
    {
      Param_.resize(n);
    }
  private:
    Eigen::VectorXd Param_;

  };
  typedef Eigen::VectorXd parameters;
  typedef Eigen::VectorXd Delta;
  friend class QuadraticObjectiveFunction;
  friend class LinearFunctionConstraints;
  QuadraticProblemSolver();
  virtual ~QuadraticProblemSolver();
  friend double quadprogpp::solve_quadprog(quadprogpp::Matrix<double>& G, quadprogpp::Vector<double>& g0,
                        const quadprogpp::Matrix<double>& CE, const quadprogpp::Vector<double>& ce0,
                        const quadprogpp::Matrix<double>& CI, const quadprogpp::Vector<double>& ci0,
                        quadprogpp::Vector<double>& x);
  virtual bool setGlobalHessian(Eigen::MatrixXd& hessian);
  virtual bool setLinearTerm(Eigen::VectorXd& jacobian);
  virtual bool setGlobalInequalityConstraintJacobian(Eigen::MatrixXd& A);
  virtual bool setGlobalEqualityConstraintJacobian(Eigen::MatrixXd& Aeq);
  virtual bool setInequalityConstraintMaxValues(Eigen::VectorXd& b);
  virtual bool setEqualityConstraintMaxValues(Eigen::VectorXd& beq);
  bool minimize(const qp_solver::QuadraticObjectiveFunction& function,
                const qp_solver::LinearFunctionConstraints& constraints,
                qp_solver::QuadraticProblemSolver::parameters& params);
  void setG_(quadprogpp::Matrix<double>& G);
  void setCE_(quadprogpp::Matrix<double>& CE);
  void setCI_(quadprogpp::Matrix<double>& CI);
  void setg0_(quadprogpp::Vector<double>& g0);
  void setce0_(quadprogpp::Vector<double>& ce0);
  void setci0_(quadprogpp::Vector<double>& ci0);
  void setx_(quadprogpp::Vector<double>& x);

  void setG_(Eigen::MatrixXd& G);
  void setCE_(Eigen::MatrixXd& CE);
  void setCI_(Eigen::MatrixXd& CI);
  void setg0_(Eigen::VectorXd& g0);
  void setce0_(Eigen::VectorXd& ce0);
  void setci0_(Eigen::VectorXd& ci0);

//  void toEigenMatrix(const quadprogpp::Matrix<double> mat_in, Eigen::MatrixXd& mat_out);
//  void toEigenVector(const quadprogpp::Vector<double> vec_in, Eigen::VectorXd& vec_out);

  bool getCoefficients(quadprogpp::Matrix<double>& G, quadprogpp::Vector<double>& g0,
                       quadprogpp::Matrix<double>& CE, quadprogpp::Vector<double>& ce0,
                       quadprogpp::Matrix<double>& CI, quadprogpp::Vector<double>& ci0,
                       quadprogpp::Vector<double>& x);
  Eigen::VectorXd jacobian_, b_, beq_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> hessian_, A_, Aeq_;
private:
  quadprogpp::Matrix<double> G_, CE_, CI_;
  quadprogpp::Vector<double> g0_, ce0_, ci0_, x_;

  unsigned int variable_dimension_;

};

class QuadraticObjectiveFunction : public QuadraticProblemSolver
{
public:
  QuadraticObjectiveFunction();
  virtual ~QuadraticObjectiveFunction();
  bool setGlobalHessian(Eigen::MatrixXd& hessian);
  bool setLinearTerm(Eigen::VectorXd& jacobian);


private:
//  quadprogpp::Matrix<double> G_, CE_, CI_;
//  quadprogpp::Vector<double> g0_, ce0_, ci0_, x_;
};

class LinearFunctionConstraints : public QuadraticProblemSolver
{
public:
  LinearFunctionConstraints();
  virtual ~LinearFunctionConstraints();
  size_t getNumberOfInequalityConstraints()
  {
    return  nInequalityConstraints_;
  };
  size_t setNumberOfInequalityConstraints(size_t nInequalityConstraints)
  {
    nInequalityConstraints_ =nInequalityConstraints;
  };
  bool setGlobalInequalityConstraintJacobian(Eigen::MatrixXd& A);
  bool setGlobalEqualityConstraintJacobian(Eigen::MatrixXd& Aeq);
  bool setInequalityConstraintMaxValues(Eigen::VectorXd& b);
  bool setEqualityConstraintMaxValues(Eigen::VectorXd& beq);

  size_t nInequalityConstraints_;

private:


};

} // namespace qp_solver
#endif // QUADRATICPROBLEMSOLVER_H
