#ifndef SEQUENCEQUADRATICPROBLEMSOLVER_H
#define SEQUENCEQUADRATICPROBLEMSOLVER_H
#include "qp_solver/quadraticproblemsolver.h"
namespace sqp_solver {

class QuadraticProblemSolver;
class QuadraticObjectiveFunction;
class LinearFunctionConstraints;

class SequenceQuadraticProblemSolver
{
public:
  SequenceQuadraticProblemSolver(double tolerance, int max_iteration);
  virtual ~SequenceQuadraticProblemSolver();

  bool minimize(std::unique_ptr<QuadraticProblemSolver>& QPsubproblem,
                qp_solver::QuadraticProblemSolver::parameters& params);
private:
  double tolerance_;
  int max_iteration_;
};

}// namespace sqp_solver
#endif // SEQUENCEQUADRATICPROBLEMSOLVER_H
