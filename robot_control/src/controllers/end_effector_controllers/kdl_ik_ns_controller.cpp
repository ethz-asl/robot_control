/*!
 * @file     kdl_ik_ns_controller.cpp
 * @author   Giuseppe Rizzi
 * @date     28.10.2020
 * @version  1.0
 * @brief    description
 */

#include "robot_control/controllers/end_effector_controllers/kdl_ik_ns_controller.h"
#include <Eigen/Geometry>
#include <urdf/model.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#define THRESHOLD_FK 1e-2

using namespace Eigen;

namespace rc
{
void IKNullSpaceController_KDL::initFromUrdfFile(const std::string &urdf_file, 
                                                 const std::string &root_link, 
                                                 const std::string &tip_link)
{
  KDL::Tree kdlTree;
  if (!kdl_parser::treeFromFile(urdf_file, kdlTree))
  {
    throw std::invalid_argument("Failed to load URDF");
  }

  setup(kdlTree, root_link, tip_link);
}

void IKNullSpaceController_KDL::initFromXmlString(const std::string &xml_string, 
                                                  const std::string &root_link, 
                                                  const std::string &tip_link)
{
  KDL::Tree kdlTree;
  if (!kdl_parser::treeFromString(xml_string, kdlTree))
  {
    throw std::invalid_argument("Failed to load URDF");
  }

  setup(kdlTree, root_link, tip_link);
}

void IKNullSpaceController_KDL::setJointLimitsFromUrdf(const std::string &xml_string,
    const std::vector<std::string>& joint_names) {
  std::cout << "Parsing urdf" << std::endl;
  urdf::Model urdf;
  urdf.initString(xml_string);

  std::cout << "Extracting joint limits from urdf" << std::endl;
  joint_limits_interface::JointLimits limits;
  Eigen::VectorXd upper(numJoints);
  Eigen::VectorXd lower(numJoints);
  for (size_t i=0; i<numJoints; i++){
    urdf::JointConstSharedPtr urdf_joint = urdf.getJoint(joint_names[i]);
    const bool urdf_limits_ok = getJointLimits(urdf_joint, limits);
    if (! urdf_limits_ok){
      std::cout << "Failed to parse limits for joint " << joint_names[i] << ", setting to max.";
      upper(i) = std::numeric_limits<double>::max();
      lower(i) = std::numeric_limits<double>::min();
    }
    else{
      upper(i) = limits.max_position;
      lower(i) = limits.min_position;
    }
  }

  std::cout << "Setting up joint limits:" << std::endl <<
                                             "upper: " << upper.transpose() << std::endl <<
                                             "lower: " << lower.transpose() << std::endl;
  setJointLimits(lower, upper);
}

void IKNullSpaceController_KDL::setNullspaceConfiguration(const VectorXd& q_nullspace){
  if (q_nullspace.size() != numJoints){
    std::cout << "Failed to set the nullspace configuration. Wrong size: " << std::endl;
    return;
  }

  for (size_t i=0; i<numJoints; i++){
    qNullspace(i) = q_nullspace(i);
  }
  ikVelSolver->setOptPos(qNullspace);
  ikPosSolver = new KDL::ChainIkSolverPos_NR_JL(kdlChain, *fkPosSolver, *ikVelSolver, maxIter, eps);
}

void IKNullSpaceController_KDL::setNullspaceWeights(const VectorXd& q_nullspace_weights){
  if (q_nullspace_weights.size() != numJoints){
    std::cout << "Failed to set the nullspace weights. Wrong size." << std::endl;
    return;
  }

  for (size_t i=0; i<numJoints; i++){
    qNullspaceWeights(i) = q_nullspace_weights(i);
  }
  ikVelSolver->setWeights(qNullspaceWeights);
  ikPosSolver = new KDL::ChainIkSolverPos_NR_JL(kdlChain, *fkPosSolver, *ikVelSolver, maxIter, eps);
}

void IKNullSpaceController_KDL::setup(const KDL::Tree &kdlTree, 
                                      const std::string &root_link, 
                                      const std::string &tip_link)
{
  if (!kdlTree.getChain(root_link, tip_link, kdlChain))
  {
    throw std::invalid_argument("Failed to extract chain");
  }


  numJoints = getNumJoints();
  qNullspace = KDL::JntArray(numJoints);
  qNullspaceWeights = KDL::JntArray(numJoints);

  fkPosSolver = new KDL::ChainFkSolverPos_recursive(kdlChain);

  double eps_ikvel = 0.001; //0.00001;
  int maxiter_ikvel = 150; //150;
  ikVelSolver = new KDL::ChainIkSolverVel_pinv_nso(kdlChain, eps_ikvel, maxiter_ikvel);
  ikPosSolver = new KDL::ChainIkSolverPos_NR_JL(kdlChain, *fkPosSolver, *ikVelSolver, maxIter, eps);

  // solver without joint limits
  ikVelSolver2 = new KDL::ChainIkSolverVel_pinv(kdlChain);
  ikPosSolver2 = new KDL::ChainIkSolverPos_NR(kdlChain, *fkPosSolver, *ikVelSolver);

}

void IKNullSpaceController_KDL::setJointLimits(const VectorXd& lower, const VectorXd& upper){
  KDL::JntArray q_min(numJoints);
  KDL::JntArray q_max(numJoints);
  for (size_t i=0; i<numJoints; i++){
    q_min(i) = lower(i);
    q_max(i) = upper(i);
  }
  int ret = ikPosSolver->setJointLimits(q_min, q_max);
  if (verbose) { std::cout << ikPosSolver->strError(ret) << std::endl;}
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
  int return_code = ikPosSolver->CartToJnt(q_init, pose_desired, q_out);
  if (verbose) {
    std::cout << "IK return code: " << ikPosSolver->strError(return_code) << std::endl;
  }

  if (return_code == ikPosSolver->E_MAX_ITERATIONS_EXCEEDED ||
      return_code == ikPosSolver->E_FKSOLVERPOS_FAILED ||
      return_code == ikPosSolver->E_IKSOLVERVEL_FAILED) {
    return q0;
  }

  VectorXd res;
  res.resize(numJoints);
  for (int i = 0; i < numJoints; i++)
  {
    res(i) = q_out(i);
  }
  return res;
}

VectorXd IKNullSpaceController_KDL::computeCommand2(const Matrix3d& rot, const Vector3d& pos, const VectorXd& q0)
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
  int return_code = ikPosSolver2->CartToJnt(q_init, pose_desired, q_out);
  if (verbose) {
    std::cout << "IK return code: " << ikPosSolver2->strError(return_code) << std::endl;
  }

  if (return_code == ikPosSolver2->E_MAX_ITERATIONS_EXCEEDED ||
      return_code == ikPosSolver2->E_FKSOLVERPOS_FAILED) {
    return q0;
  }

  VectorXd res;
  res.resize(numJoints);
  for (int i = 0; i < numJoints; i++)
  {
    res(i) = q_out(i);
  }
  return res;
}

void IKNullSpaceController_KDL::forwardKinematics(const VectorXd &q, Matrix3d &rot, Vector3d &pos) {
  KDL::Frame pose_out;
  KDL::JntArray q_in(numJoints);
  for (size_t i=0; i< numJoints; i++) q_in(i) = q(i);
  fkPosSolver->JntToCart(q_in, pose_out);

  pos(0) = pose_out.p.x();
  pos(1) = pose_out.p.y();
  pos(2) = pose_out.p.z();
  double x, y, z, w;
  pose_out.M.GetQuaternion(x, y, z, w);
  rot = Eigen::Matrix3d(Eigen::Quaterniond(w, x, y, z));
}

} // namespace rc