/*!
 * @file     trajectory_generator.cpp
 * @author   Giuseppe Rizzi
 * @date     05.11.2020
 * @version  1.0
 * @brief    description
 */

#include <robot_control/utils/trajectory_generator.h>

namespace rc{

TrajectoryGenerator::TrajectoryGenerator(double max_vel, double max_acc, unsigned int size)
    : generators_(size)
{
  for (unsigned int i=0; i<size; i++){
    generators_[i] = new KDL::VelocityProfile_Trap(max_vel, max_acc);
    //generators_[i] = new rc::VelocityProfile_ATrap(max_vel, max_acc, max_acc);
  }
}

TrajectoryGenerator::~TrajectoryGenerator()
{
  for (unsigned int i=0; i<generators_.size(); i++)
    delete generators_[i];
}

//void TrajectoryGenerator::compute(const Eigen::VectorXd& start,
//                                 const Eigen::VectorXd& end,
//                                  double t_start){
//  Eigen::VectorXd start_velocity(start.size());
//  compute_from_initial_velocity(start, end, start_velocity, t_start);
//}

void TrajectoryGenerator::compute(const Eigen::VectorXd& start,
                                  const Eigen::VectorXd& end,
                                  double t_start) {

  initial_time = t_start;

  // check
  if (start.size() != generators_.size() ||
      end.size() != generators_.size()) {
    throw std::runtime_error("Point size different from generator size.");
  }

  // generate initial profiles
  for (unsigned int i = 0; i < generators_.size(); i++){
    //bool ok = generators_[i]->setProfileStartVelocity(start(i), end(i), start_velocity(i));
    //if (!ok) std::cout << "Could not set profile start velocity for start: "
    //   << start(i) << ", end: " << end(i) << ", vel: " << start_velocity(i) << std::endl;
    //generators_[i]->setProfileStartVelocity(start(i), end(i));
    generators_[i]->SetProfile(start(i), end(i));
  }

  // find profile that takes most time
  max_time = 0.001;

  for (unsigned int i = 0; i < generators_.size(); i++)
    if (generators_[i]->Duration() > max_time)
      max_time = generators_[i]->Duration();

  // generate profiles with max time
  for (unsigned int i = 0; i < generators_.size(); i++)
    generators_[i]->SetProfileDuration(start(i), end(i), max_time);
}


Eigen::VectorXd TrajectoryGenerator::get_next_point(const double time){
  Eigen::VectorXd q_out(generators_.size());
  double t = time - initial_time;
  t = (t > max_time) ? max_time : t;
  for (unsigned int i = 0; i < generators_.size(); i++) {
    q_out(i) = generators_[i]->Pos(t);
    //qd_out(i) =  generators_[i]->Vel(t);
    //qdd_out(i) = generators_[i]->Acc(t);
  }
  return q_out;
}

}

std::ostream& operator<<(std::ostream& os, const rc::TrajectoryGenerator& gen){
  for(const auto& g : gen.generators_){
    g->Write(os);
    os << std::endl;
  }
}