#include "qp_solver/sequencequadraticproblemsolver.h"
namespace sqp_solver {


SequenceQuadraticProblemSolver::SequenceQuadraticProblemSolver(double tolerance, int max_iteration)
{

};
SequenceQuadraticProblemSolver::~SequenceQuadraticProblemSolver()
{}

bool SequenceQuadraticProblemSolver::minimize(std::unique_ptr<QuadraticProblemSolver>& QPsubproblem,
                                              qp_solver::QuadraticProblemSolver::parameters& params)
{
  int k = 0;
  while (k<max_iteration_) {
    k = k+1;

  }
}
}// namespace sqp_solver
