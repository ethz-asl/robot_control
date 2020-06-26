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
        void initFromUrdfFile(const std::string &urdf_file, const std::string &first_link, const std::string &last_link);
        void initFromXmlString(const std::string &xml_string, const std::string &root_link, const std::string &tip_link);
        VectorXd computeCommand(Matrix<double, 3, 1> &velocity_translation, Matrix<double, 3, 1> &velocity_rotation, Matrix<double, Dynamic, 1> &q_in);
        unsigned int getNumJoints() { return kdlChain.getNrOfJoints(); }
        ~CartesianVelocityController_KDL() { delete ikSolver; }

    private:
        void setup(const KDL::Tree &kdlTree, const std::string &root_link, const std::string &tip_link);

        KDL::Chain kdlChain;
        unsigned int numJoints;
        KDL::ChainIkSolverVel_pinv *ikSolver;
    };
} // namespace rc
