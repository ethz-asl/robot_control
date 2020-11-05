/*!
 * @file     test_trajectyory_generator.cpp
 * @author   Giuseppe Rizzi
 * @date     05.11.2020
 * @version  1.0
 * @brief    description
 */

#include "robot_control/utils/trajectory_generator.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>


TEST(TrajectoryGenerator, computeTest){
  rc::TrajectoryGenerator generator(10.0, 100.0, 3);
  Eigen::VectorXd start(3);
  Eigen::VectorXd end(3);
  start.setZero();
  end.setConstant(1);

  generator.compute(start, end, 1.0);
  generator.compute(end, start, 1.0);
  generator.compute(start, end, 0.0);
  generator.compute(start, start, 0.0);
  generator.compute(end, end, 0.0);
}

TEST(TrajectoryGenerator, getPointTest){
  rc::TrajectoryGenerator generator(0.01, 1.0, 3);
  Eigen::VectorXd start(3);
  Eigen::VectorXd end(3);
  start.setZero();
  end.setConstant(1);

  generator.compute(start, end, 1.0);
  std::cout << "t=1.0:  " << generator.get_next_point(1.0).transpose() << std::endl;
  std::cout << "t=2.0:  " << generator.get_next_point(2.0).transpose() << std::endl;
  std::cout << "t=10.0: " << generator.get_next_point(10.0).transpose() << std::endl;
  std::cout << "t=20.0: " << generator.get_next_point(20.0).transpose() << std::endl;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
