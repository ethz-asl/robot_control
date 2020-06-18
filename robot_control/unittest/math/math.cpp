/*!
 * @file     math.cpp
 * @author   Giuseppe Rizzi
 * @date     09.06.2020
 * @version  1.0
 * @brief    description
 */

#include "robot_control/math/math.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>

using namespace Eigen;
using namespace linear_algebra;

TEST(LinearAlgebra, computeAdaptiveDampedPseudoInverse){
  JacobiSVD<MatrixXd> svd;
  // trivial case
  MatrixXd I = MatrixXd::Identity(6, 6);
  DLSSolver solver(svd, 0.2, 0.4);
  solver.compute(I);
  MatrixXd Ipinv = solver.matrix();
  EXPECT_TRUE(I.isApprox(Ipinv));

  // one zero singular value
  VectorXd v = VectorXd::Ones(4); v(3) = 0;
  MatrixXd M = v.asDiagonal();
  DLSSolver solver2(svd, 0.2, 0.4);
  solver2.compute(M);
  MatrixXd Mpinv = solver2.matrix();
  JacobiSVD<MatrixXd> Mpinv_svd(Mpinv, ComputeThinU | ComputeThinV);
  const VectorXd& sigma = Mpinv_svd.singularValues();
  double sigma_max = sigma.head<1>()(0);
  double sigma_min = sigma.tail<1>()(0);
  EXPECT_EQ(sigma_min, 0.0);
  EXPECT_EQ(sigma_max, 1.0);

  // more cols than rows
  Matrix<double, 6, 12> Q; Q.setRandom();
  DLSSolver solver3(svd);
  ASSERT_NO_THROW(solver3.compute(Q));
  ASSERT_NO_THROW(solver3.matrix());

  // more rows than cols
  Matrix<double, 12, 6> P; P.setRandom();
  DLSSolver solver4(svd);
  VectorXd ones = VectorXd::Ones(6);
  ASSERT_NO_THROW(solver4.compute(P));
  ASSERT_NO_THROW(solver4.matrix());
  ASSERT_NO_THROW(solver4.solve(ones));
}

TEST(LinearAlgebra, ComputeNullSpace){
  MatrixXd I = MatrixXd::Identity(6, 6);
  MatrixXd O = MatrixXd::Zero(6, 6);
  MatrixXd NI = computeNullSpace(I);
  ASSERT_TRUE(O.isApprox(NI));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
