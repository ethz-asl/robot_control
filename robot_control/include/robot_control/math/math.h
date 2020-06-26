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

namespace linear_algebra {

class DLSSolver {
    JacobiSVD<MatrixXd>& solver;
    double max_damping;
    double sigma_min_clamp;
    VectorXd sigma_inverse;

    public:
    DLSSolver(JacobiSVD<MatrixXd>& solver, double max_damping = 0.02, double sigma_min_clamp = 0.06);
    void compute(const MatrixXd& A);
    MatrixXd matrix() const;
    VectorXd solve(const VectorXd& x) const;
};

MatrixXd computeNullSpace(const MatrixXd& A);

} // end of namespace linear_algebra
