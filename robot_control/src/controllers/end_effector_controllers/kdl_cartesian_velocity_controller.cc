#include "robot_control/controllers/end_effector_controllers/kdl_cartesian_velocity_controller.h"

namespace rc
{
    CartesianVelocityController_KDL::CartesianVelocityController_KDL(const std::string &urdf_file, const std::string &root_link, const std::string &tip_link)
    {
        KDL::Tree kdlTree;
        if (!kdl_parser::treeFromFile(urdf_file, kdlTree))
        {
            throw std::invalid_argument("Failed to load URDF");
        }

        if (!kdlTree.getChain(root_link, tip_link, kdlChain))
        {
            throw std::invalid_argument("Failed to extract chain");
        }

        numJoints = getNumJoints();

        ikSolver = new KDL::ChainIkSolverVel_pinv(kdlChain);
    }

    Eigen::VectorXd CartesianVelocityController_KDL::computeCommand(Eigen::Matrix<double, 3, 1> &velocity_translation, Eigen::Matrix<double, 3, 1> &velocity_rotation)
    {
        Eigen::VectorXd res;
        res.resize(numJoints);
        return res;
    }
} // namespace rc
