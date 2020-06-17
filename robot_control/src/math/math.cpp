/*!
 * @file     math.cpp
 * @author   Giuseppe Rizzi
 * @date     17.06.2020
 * @version  1.0
 * @brief    description
 */

#include "robot_control/math/math.h"

using namespace Eigen;

namespace linear_algebra{

MatrixXd computePInvDLS(JacobiSVD<MatrixXd> &svd_solver, const MatrixXd& A, double max_damping, double sigma_min_clamp) {
  svd_solver.compute(A, ComputeThinU | ComputeThinV);
  const VectorXd& sigma = svd_solver.singularValues();

  const double sigma_min = sigma(svd_solver.nonzeroSingularValues() - 1);

  double damping = 0.0;
  if (sigma_min < sigma_min_clamp) {
    damping = (1.0 - (sigma_min * sigma_min) / (sigma_min_clamp * sigma_min_clamp)) * max_damping;
  }

  VectorXd sigma_inverse = sigma;
  for (int k = 0; k < svd_solver.nonzeroSingularValues(); ++k) {
    sigma_inverse(k) = sigma(k) / (sigma(k) * sigma(k) + damping * damping);
  }

  return svd_solver.matrixV() * sigma_inverse.asDiagonal() * svd_solver.matrixU().transpose();
}

MatrixXd computePInvDLS(const MatrixXd& A, double max_damping, double sigma_min_clamp) {
  JacobiSVD<MatrixXd> solver(A.rows(), A.cols());
  return computePInvDLS(solver, A, max_damping, sigma_min_clamp);
}

MatrixXd adaptSingularValues(const Ref<const MatrixXd>& A, double sigma_min) {
  const JacobiSVD<MatrixXd> Asvd(A, ComputeThinU | ComputeThinV);
  VectorXd sigma = Asvd.singularValues();

  for (unsigned int i = 0; i < sigma.size(); ++i) {
    if (sigma(i) < sigma_min) {
      sigma(i) = sigma_min;
    }
  }

  return Asvd.matrixU() * sigma.asDiagonal() * Asvd.matrixV().transpose();
}

MatrixXd computeNullSpace(const MatrixXd& A){
  CompleteOrthogonalDecomposition<MatrixXd> decomp(A.rows(), A.cols());
  decomp.setThreshold(0.1);
  decomp.compute(A);
  MatrixXd JpinvJ = decomp.pseudoInverse() * A;
  return MatrixXd::Identity(A.cols(), A.cols()) - JpinvJ;
}

} // end of namespace linear_algebra
