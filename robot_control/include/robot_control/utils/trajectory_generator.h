/*!
 * @file     trajectory_generator.h
 * @author   Giuseppe Rizzi
 * @date     05.11.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

// adapted from
// https://github.com/PR2/pr2_common_actions/blob/kinetic-devel/joint_trajectory_generator/include/joint_trajectory_generator/trajectory_generation.h

#include <kdl/velocityprofile_trap.hpp>
#include <robot_control/utils/velocity_profile_atrap.h>
#include <vector>
#include <Eigen/Core>

namespace rc{
class TrajectoryGenerator
{
 public:
  TrajectoryGenerator(): TrajectoryGenerator(1.0, 10.0, 0){};
  TrajectoryGenerator(double max_vel, double max_acc, unsigned int size);
  ~TrajectoryGenerator();

  void compute(const Eigen::VectorXd& start,
               const Eigen::VectorXd& end,
               const double t_start);

  //void compute_from_initial_velocity(const Eigen::VectorXd& start,
  //                                   const Eigen::VectorXd& end,
  //                                   const Eigen::VectorXd& start_velocity,
  //                                   const double t_start);


  Eigen::VectorXd get_next_point(const double time);

 private:
  double max_time;
  double initial_time;
 public:
  std::vector<KDL::VelocityProfile_Trap*> generators_;
  //std::vector<rc::VelocityProfile_ATrap*> generators_;
};
}

std::ostream& operator<<(std::ostream& os, const rc::TrajectoryGenerator& gen);