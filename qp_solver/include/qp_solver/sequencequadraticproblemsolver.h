#ifndef SEQUENCEQUADRATICPROBLEMSOLVER_H
#define SEQUENCEQUADRATICPROBLEMSOLVER_H
#include "qp_solver/quadraticproblemsolver.h"
#include "free_gait_core/pose_optimization/PoseOptimizationProblem.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationFunctionConstraints.hpp"
#include "free_gait_core/pose_optimization/PoseOptimizationObjectiveFunction.hpp"
#include "free_gait_core/pose_optimization/poseparameterization.h"
#include <memory>
namespace sqp_solver {
using namespace free_gait;
class QuadraticProblemSolver;
class QuadraticObjectiveFunction;
class LinearFunctionConstraints;

class SequenceQuadraticProblemSolver
{
public:
  SequenceQuadraticProblemSolver(std::shared_ptr<qp_solver::QuadraticProblemSolver>& quadratic_solver,
                                 double tolerance, int max_iteration);
  virtual ~SequenceQuadraticProblemSolver();

  bool minimize(const PoseOptimizationProblem& problem,
                PoseParameterization& params);

private:
  double tolerance_;
  int max_iteration_;
  std::shared_ptr<qp_solver::QuadraticProblemSolver> quadratic_solver_;
};

}// namespace sqp_solver
#endif // SEQUENCEQUADRATICPROBLEMSOLVER_H
