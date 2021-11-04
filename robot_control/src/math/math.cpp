/*!
 * @file     math.cpp
 * @author   Giuseppe Rizzi
 * @date     17.06.2020
 * @version  1.0
 * @brief    description
 */

#include "robot_control/math/math.h"

namespace linear_algebra{

DLSSolver::DLSSolver(Eigen::JacobiSVD<Eigen::MatrixXd>& solver, double md, double smc) : solver(solver) {
  max_damping = md;
  sigma_min_clamp = smc;
}

void DLSSolver::compute(const Eigen::MatrixXd& A) {
  solver.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  sigma_inverse = solver.singularValues();

  const Eigen::VectorXd& sigma = solver.singularValues();
  const double sigma_min = sigma(solver.nonzeroSingularValues() - 1);

  double damping = 0.0;
  if (sigma_min < sigma_min_clamp) {
    damping = (1.0 - (sigma_min * sigma_min) / (sigma_min_clamp * sigma_min_clamp)) * max_damping;
  }

  for (int k = 0; k < solver.nonzeroSingularValues(); ++k) {
    sigma_inverse(k) = sigma(k) / (sigma(k) * sigma(k) + damping * damping);
  }
}

Eigen::MatrixXd DLSSolver::matrix() const {
  return solver.matrixV() * sigma_inverse.asDiagonal() * solver.matrixU().transpose();
}

Eigen::VectorXd DLSSolver::solve(const Eigen::VectorXd& x) const {
  return solver.matrixV() * (sigma_inverse.asDiagonal() * (solver.matrixU().transpose() * x));
}

Eigen::MatrixXd computeNullSpace(const Eigen::MatrixXd& A) {
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> decomp(A.rows(), A.cols());
  decomp.setThreshold(0.1);
  decomp.compute(A);
  Eigen::MatrixXd JpinvJ = decomp.pseudoInverse() * A;
  return Eigen::MatrixXd::Identity(A.cols(), A.cols()) - JpinvJ;
}

} // end of namespace linear_algebra
