/*!
 * @file     math.h
 * @author   Giuseppe Rizzi
 * @date     09.06.2020
 * @version  1.0
 * @brief    linear algebra utilities
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/SVD>
#include <iostream>

using namespace Eigen;

namespace linear_algebra{

MatrixXd computePInvDLS(JacobiSVD<MatrixXd> &svd_solver, const MatrixXd& A, double max_damping = 0.02,
    double sigma_min_clamp = 0.06);

MatrixXd computePInvDLS(const MatrixXd& A, double max_damping = 0.02, double sigma_min_clamp = 0.06);

MatrixXd computeNullSpace(const MatrixXd& A);

} // end of namespace linear_algebra
