#pragma once

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <Eigen/Core>

using namespace Eigen;

namespace rc
{
    class CartesianVelocityController_KDL
    {
    public:
        CartesianVelocityController_KDL(const std::string &urdf_file, const std::string &first_link, const std::string &last_link);
        VectorXd computeCommand(Matrix<double, 3, 1> &velocity_translation, Matrix<double, 3, 1> &velocity_rotation, Matrix<double, Dynamic, 1> &q_in);
        unsigned int getNumJoints() { return kdlChain.getNrOfJoints(); }
        ~CartesianVelocityController_KDL() { delete ikSolver; }

    private:
        KDL::Chain kdlChain;
        unsigned int numJoints;
        KDL::ChainIkSolverVel_pinv *ikSolver;
    };
} // namespace rc
