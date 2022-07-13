#include "robot_control/controllers/end_effector_controllers/kdl_cartesian_velocity_controller.h"

using namespace Eigen;

namespace rc
{
    void CartesianVelocityController_KDL::initFromUrdfFile(const std::string &urdf_file, const std::string &root_link, const std::string &tip_link)
    {
        KDL::Tree kdlTree;
        if (!kdl_parser::treeFromFile(urdf_file, kdlTree))
        {
            throw std::invalid_argument("Failed to load URDF");
        }

        setup(kdlTree, root_link, tip_link);
    }

    void CartesianVelocityController_KDL::initFromXmlString(const std::string &xml_string, const std::string &root_link, const std::string &tip_link)
    {
        KDL::Tree kdlTree;
        if (!kdl_parser::treeFromString(xml_string, kdlTree))
        {
            throw std::invalid_argument("Failed to load URDF");
        }

        setup(kdlTree, root_link, tip_link);
    }

    void CartesianVelocityController_KDL::setup(const KDL::Tree &kdlTree, const std::string &root_link, const std::string &tip_link)
    {
        if (!kdlTree.getChain(root_link, tip_link, kdlChain))
        {
            throw std::invalid_argument("Failed to extract chain");
        }

        numJoints = getNumJoints();

        ikSolver = new KDL::ChainIkSolverVel_pinv(kdlChain);
    }

    VectorXd CartesianVelocityController_KDL::computeCommand(Matrix<double, 3, 1> &velocity_translation, Matrix<double, 3, 1> &velocity_rotation, Matrix<double, Dynamic, 1> &q_in)
    {
        KDL::JntArray q_in_arr(numJoints);
        KDL::Twist v_desired;
        for (int i = 0; i < numJoints; i++)
        {
            q_in_arr(i) = q_in[i];
        }
        for (int i = 0; i < 3; i++)
        {
            v_desired[i] = velocity_translation[i];
            v_desired[i + 3] = velocity_rotation[i];
        }

        KDL::JntArray qdot_out(numJoints);
        ikSolver->CartToJnt(q_in_arr, v_desired, qdot_out);

        VectorXd res;
        res.resize(numJoints);
        for (int i = 0; i < numJoints; i++)
        {
            res[i] = qdot_out(i);
        }
        return res;
    }
} // namespace rc
