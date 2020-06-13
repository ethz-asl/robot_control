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

TEST(LinearAlgebra, ComputeAdaptiveDampedPseudoInverse){
  // trivial case
  MatrixXd I = MatrixXd::Identity(6, 6);
  MatrixXd Ipinv = computePInvDLS(I, 0.2, 0.4);
  EXPECT_TRUE(I.isApprox(Ipinv));

  // one zero singular value
  VectorXd v = VectorXd::Ones(4); v(3) = 0;
  MatrixXd M = v.asDiagonal();
  JacobiSVD<MatrixXd> solver(M.rows(), M.cols());
  MatrixXd Mpinv = computePInvDLS(solver, M, 0.2, 0.4);
  JacobiSVD<MatrixXd> Mpinv_svd(Mpinv, ComputeThinU | ComputeThinV);
  const VectorXd& sigma = Mpinv_svd.singularValues();
  double sigma_max = sigma.head<1>()(0);
  double sigma_min = sigma.tail<1>()(0);
  EXPECT_EQ(sigma_min, 0.0);
  EXPECT_EQ(sigma_max, 1.0);

  // more cols than rows
  Matrix<double, 6, 12> Q; Q.setRandom();
  ASSERT_NO_THROW(computePInvDLS(Q));

  // more rows than cols
  Matrix<double, 12, 6> P; P.setRandom();
  ASSERT_NO_THROW(computePInvDLS(P));
}

TEST(LinearAlgebra, AdaptSingularValues){
  MatrixXd A = MatrixXd::Identity(6, 6) * 0.3;
  MatrixXd A_adapted = adaptSingularValues(A, 0.5);
  VectorXd s = VectorXd::Ones(6) * 0.5;
  EXPECT_TRUE(s.isApprox(A_adapted.diagonal()));

  MatrixXd B = MatrixXd::Identity(6, 6) * 0.5;
  MatrixXd B_adapted = adaptSingularValues(B, 0.3);
  VectorXd v = VectorXd::Ones(6) * 0.5;
  EXPECT_TRUE(v.isApprox(B_adapted.diagonal()));
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
