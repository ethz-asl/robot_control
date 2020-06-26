#include <gtest/gtest.h>
#include <math.h>
#include <ros/ros.h>
#include "robot_control/controllers/end_effector_controllers/kdl_cartesian_velocity_controller.h"

using namespace Eigen;

// Fixture
class CartesianVelocityTest : public ::testing::Test
{
protected:
    std::string robot_description;
    CartesianVelocityTest()
    {
        ros::NodeHandle nh;
        nh.param("robot_description", robot_description, std::string());
    }
};

TEST_F(CartesianVelocityTest, validCommand)
{
    rc::CartesianVelocityController_KDL ctrl;
    ctrl.initFromXmlString(robot_description, "panda_link0", "panda_hand");
    Vector3d velocity_translation;
    velocity_translation << 0.5, 0.0, 0.0;
    Vector3d velocity_rotation;
    velocity_rotation << 0.0, 0.0, 0.0;
    VectorXd q_in;
    q_in.resize(ctrl.getNumJoints());
    q_in << 0.0, -M_PI / 4.0, 0.0, -3.0 * M_PI / 4.0, 0.0, M_PI / 2.0, M_PI / 4.0;
    auto cmd = ctrl.computeCommand(velocity_translation, velocity_rotation, q_in);
    ASSERT_GT(cmd.norm(), 0.0);
}

TEST_F(CartesianVelocityTest, zeroCommand)
{
    rc::CartesianVelocityController_KDL ctrl;
    ctrl.initFromXmlString(robot_description, "panda_link0", "panda_hand");
    Vector3d velocity_translation;
    velocity_translation << 0.0, 0.0, 0.0;
    Vector3d velocity_rotation;
    velocity_rotation << 0.0, 0.0, 0.0;
    VectorXd q_in;
    q_in.resize(ctrl.getNumJoints());
    q_in << 0.0, -M_PI / 4.0, 0.0, -3.0 * M_PI / 4.0, 0.0, M_PI / 2.0, M_PI / 4.0;
    auto cmd = ctrl.computeCommand(velocity_translation, velocity_rotation, q_in);
    ASSERT_EQ(cmd.norm(), 0.0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");

    return RUN_ALL_TESTS();
}
