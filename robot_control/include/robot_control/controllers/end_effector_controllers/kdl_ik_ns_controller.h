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
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <Eigen/Core>

using namespace Eigen;

namespace rc
{
class IKNullSpaceController_KDL
{
 public:
  void initFromUrdfFile(const std::string &urdf_file, 
                        const std::string &first_link, 
                        const std::string &last_link);
  
  void initFromXmlString(const std::string &xml_string, 
                         const std::string &root_link, 
                         const std::string &tip_link);
  
  VectorXd computeCommand(const Matrix3d& rot, const Vector3d& pos, const VectorXd& q0);
  VectorXd computeCommand2(const Matrix3d& rot, const Vector3d& pos, const VectorXd& q0);

  void forwardKinematics(const VectorXd& q, Matrix3d& rot, Vector3d& pos);
  void setJointLimits(const VectorXd& lower, const VectorXd& upper);
  void setJointLimitsFromUrdf(const std::string &xml_string, const std::vector<std::string>& joint_names);
  void setNullspaceConfiguration(const VectorXd& q_nullspace);
  void setNullspaceWeights(const VectorXd& q_nullspace_weights);

  unsigned int getNumJoints() { return kdlChain.getNrOfJoints(); }

  IKNullSpaceController_KDL() = delete;
  IKNullSpaceController_KDL(bool v=false) : verbose(v){};
  ~IKNullSpaceController_KDL(){
    delete ikVelSolver;
    delete fkPosSolver;
    delete ikPosSolver;

    delete ikVelSolver2;
    delete ikPosSolver2;
  };

 private:
  void setup(const KDL::Tree &kdlTree, 
             const std::string &root_link, 
             const std::string &tip_link);

  bool verbose;

  const int maxIter = 10000;
  const double eps = 1e-3;

  KDL::Chain kdlChain;
  unsigned int numJoints;
  KDL::ChainFkSolverPos_recursive *fkPosSolver;
  KDL::ChainIkSolverVel_pinv_nso *ikVelSolver;
  KDL::ChainIkSolverPos_NR_JL *ikPosSolver;
  KDL::JntArray qNullspace;
  KDL::JntArray qNullspaceWeights;
  KDL::JntArray lowerLimits, upperLimits; //lower joint limits, upper joint limits

  KDL::ChainIkSolverVel_pinv* ikVelSolver2;
  KDL::ChainIkSolverPos_NR* ikPosSolver2;

};
} // namespace rc