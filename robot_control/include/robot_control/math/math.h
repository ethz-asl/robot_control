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

namespace linear_algebra {

class DLSSolver {
    Eigen::JacobiSVD<Eigen::MatrixXd>& solver;
    double max_damping;
    double sigma_min_clamp;
    Eigen::VectorXd sigma_inverse;

    public:
    DLSSolver(Eigen::JacobiSVD<Eigen::MatrixXd>& solver, double max_damping = 0.02, double sigma_min_clamp = 0.06);
    void compute(const Eigen::MatrixXd& A);
    Eigen::MatrixXd matrix() const;
    Eigen::VectorXd solve(const Eigen::VectorXd& x) const;
};

Eigen::MatrixXd computeNullSpace(const Eigen::MatrixXd& A);

} // end of namespace linear_algebra
