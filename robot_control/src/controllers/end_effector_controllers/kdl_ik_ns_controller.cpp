/*!
 * @file     kdl_ik_ns_controller.cpp
 * @author   Giuseppe Rizzi
 * @date     28.10.2020
 * @version  1.0
 * @brief    description
 */

#include "robot_control/controllers/end_effector_controllers/kdl_ik_ns_controller.h"

using namespace Eigen;

namespace rc
{
void IKNullSpaceController_KDL::initFromUrdfFile(const std::string &urdf_file, 
                                                 const std::string &root_link, 
                                                 const std::string &tip_link,
                                                 const VectorXd& q_nullspace, 
                                                 const VectorXd& q_nullspace_weights)
{
  KDL::Tree kdlTree;
  if (!kdl_parser::treeFromFile(urdf_file, kdlTree))
  {
    throw std::invalid_argument("Failed to load URDF");
  }

  setup(kdlTree, root_link, tip_link, q_nullspace, q_nullspace_weights);
}

void IKNullSpaceController_KDL::initFromXmlString(const std::string &xml_string, 
                                                  const std::string &root_link, 
                                                  const std::string &tip_link,
                                                  const VectorXd& q_nullspace,
                                                  const VectorXd& q_nullspace_weights)
{
  KDL::Tree kdlTree;
  if (!kdl_parser::treeFromString(xml_string, kdlTree))
  {
    throw std::invalid_argument("Failed to load URDF");
  }

  setup(kdlTree, root_link, tip_link, q_nullspace, q_nullspace_weights);
}

void IKNullSpaceController_KDL::setup(const KDL::Tree &kdlTree, 
                                      const std::string &root_link, 
                                      const std::string &tip_link,
                                      const VectorXd& q_nullspace,
                                      const VectorXd& q_nullspace_weights)
{
  if (!kdlTree.getChain(root_link, tip_link, kdlChain))
  {
    throw std::invalid_argument("Failed to extract chain");
  }


  numJoints = getNumJoints();
  qNullspace = KDL::JntArray(numJoints);
  qNullspaceWeights = KDL::JntArray(numJoints);

  if (q_nullspace.size() != numJoints){
    throw std::runtime_error("nullspace configuration has size != num of joints");
  }

  if (q_nullspace_weights.size() != numJoints){
    throw std::runtime_error("nullspace weights have size != num of joints");
  }

  for (size_t i=0; i<numJoints; i++){
    qNullspace(i) = q_nullspace(i);
    qNullspaceWeights(i) = q_nullspace_weights(i);
  }

  fkPosSolver = new KDL::ChainFkSolverPos_recursive(kdlChain);
  ikVelSolver = new KDL::ChainIkSolverVel_pinv_nso(kdlChain, qNullspace, qNullspaceWeights);
  ikPosSolver = new KDL::ChainIkSolverPos_NR_JL(kdlChain, *fkPosSolver, *ikVelSolver);
}

VectorXd IKNullSpaceController_KDL::computeCommand(const Matrix3d& rot, const Vector3d& pos, const VectorXd& q0)
{
  KDL::Frame pose_desired;
  pose_desired.M = KDL::Rotation(rot(0, 0), rot(0, 1), rot(0, 2),
                                 rot(1, 0), rot(1, 1), rot(1, 2),
                                 rot(2, 0), rot(2, 1), rot(2, 2));
  pose_desired.p = KDL::Vector(pos(0), pos(1), pos(2));

  KDL::JntArray q_init(numJoints);
  for (int i = 0; i < numJoints; i++)
  {
    q_init(i) = q0(i);
  }

  KDL::JntArray q_out(numJoints);
  ikPosSolver->CartToJnt(q_init, pose_desired, q_out);

  VectorXd res;
  res.resize(numJoints);
  for (int i = 0; i < numJoints; i++)
  {
    res(i) = q_out(i);
  }
  return res;
}
} // namespace rc