#pragma once

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <Eigen/Core>

namespace rc
{
    class CartesianVelocityController_KDL
    {
    public:
        CartesianVelocityController_KDL(const std::string &urdf_file, const std::string &first_link, const std::string &last_link);
        Eigen::VectorXd computeCommand(Eigen::Matrix<double, 3, 1> &velocity_translation, Eigen::Matrix<double, 3, 1> &velocity_rotation);
        unsigned int getNumJoints() { return kdlChain.getNrOfJoints(); }

    private:
        KDL::Chain kdlChain;
        unsigned int numJoints;
        KDL::ChainIkSolverVel_pinv *ikSolver;
    };
} // namespace rc
