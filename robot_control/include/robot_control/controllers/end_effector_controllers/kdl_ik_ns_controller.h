/*!
 * @file     kdl_ik_ns_controller.h
 * @author   Giuseppe Rizzi
 * @date     28.10.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>

#include <Eigen/Core>

using namespace Eigen;

namespace rc
{
class IKNullSpaceController_KDL
{
 public:
  void initFromUrdfFile(const std::string &urdf_file, 
                        const std::string &first_link, 
                        const std::string &last_link,
                        const VectorXd& q_nullspace,
                        const VectorXd& q_nullspace_weights);
  
  void initFromXmlString(const std::string &xml_string, 
                         const std::string &root_link, 
                         const std::string &tip_link,
                         const VectorXd& q_nullspace,
                         const VectorXd& q_nullspace_weights);
  
  VectorXd computeCommand(const Matrix3d& rot, const Vector3d& pos, const VectorXd& q0);
  unsigned int getNumJoints() { return kdlChain.getNrOfJoints(); }
  ~IKNullSpaceController_KDL(){
    delete ikVelSolver;
    delete fkPosSolver;
    delete ikPosSolver;
  };

 private:
  void setup(const KDL::Tree &kdlTree, 
             const std::string &root_link, 
             const std::string &tip_link,
             const VectorXd& q_nullspace,
             const VectorXd& q_nullspace_weights);

  KDL::Chain kdlChain;
  unsigned int numJoints;
  KDL::ChainFkSolverPos_recursive *fkPosSolver;
  KDL::ChainIkSolverVel_pinv_nso *ikVelSolver;
  KDL::ChainIkSolverPos_NR_JL *ikPosSolver;
  KDL::JntArray qNullspace;
  KDL::JntArray qNullspaceWeights;
  KDL::JntArray lowerLimits, upperLimits; //lower joint limits, upper joint limits

};
} // namespace rc