#include <gtest/gtest.h>
#include <math.h>
#include <ros/package.h>
#include "robot_control/controllers/end_effector_controllers/kdl_cartesian_velocity_controller.h"

using namespace Eigen;

// Declare a test
TEST(CartesianTestSuite, firstCase)
{
    std::cout << "-------------------------------------------------------------------------------------------" << std::endl;
    std::string pkg_path = ros::package::getPath("moma_description");
    // TODO need to parse the xacro before using it here.
    rc::CartesianVelocityController_KDL ctrl(pkg_path + "/urdf/panda.urdf.xacro", "panda_link0", "panda_default_EE");
    Vector3d velocity_translation;
    velocity_translation << 0.5, 0.0, 0.0;
    Vector3d velocity_rotation;
    velocity_rotation << 0.0, 0.0, 0.0;
    VectorXd q_in;
    q_in.resize(ctrl.getNumJoints());
    q_in << 0.0, -M_PI / 4.0, 0.0, -3.0 * M_PI / 4.0, 0.0, M_PI / 2.0, M_PI / 4.0;
    auto cmd = ctrl.computeCommand(velocity_translation, velocity_rotation, q_in);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
