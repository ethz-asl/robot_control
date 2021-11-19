/*!
 * @file     ik_debug.cpp
 * @author   Giuseppe Rizzi
 * @date     30.10.2020
 * @version  1.0
 * @brief    description
 */

#include <math.h>
#include <mutex>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Geometry>
#include "robot_control/controllers/end_effector_controllers/kdl_ik_ns_controller.h"

#define DEFAULT_CONFIGURATION -0.18, 0.30, 0.31, 1.15, -0.27, 1.25, -1.17//0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57

class IKDebug{
 private:
  ros::NodeHandle nh;
  ros::ServiceServer ikService;
  ros::Subscriber poseDesiredSub;
  ros::Publisher jointPublisher;
  sensor_msgs::JointState jointStateSolver;

  rc::IKNullSpaceController_KDL ctrl;
  std::string robot_description;

  std::mutex target_mutex, solution_mutex;
  geometry_msgs::PoseStamped target;
  Eigen::VectorXd solution;
  Eigen::VectorXd q_guess;

 public:
  IKDebug() = delete;
  IKDebug(ros::NodeHandle& node): nh(node), ctrl(rc::IKNullSpaceController_KDL(true)){
    nh.param("robot_description", robot_description, std::string());

    jointPublisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ikService = nh.advertiseService("/set_target", &IKDebug::solve, this);
    poseDesiredSub = nh.subscribe("/target_pose", 10, &IKDebug::target_callback, this);

    VectorXd q_nullspace(7), q_nullspace_weights(7);
    q_nullspace << DEFAULT_CONFIGURATION;
    q_nullspace_weights.setConstant(1.0);
    ctrl.initFromXmlString(robot_description, "base_link", "end_effector_link");
    ctrl.setNullspaceConfiguration(q_nullspace);
    ctrl.setNullspaceWeights(q_nullspace_weights);

    jointStateSolver.name = {"joint_1", "joint_2", "joint_3", "joint_4",
                             "joint_5", "joint_6", "joint_7",};
    jointStateSolver.position.resize(7, 0.0);
    solution.setZero(ctrl.getNumJoints());
    for(size_t i=0; i<ctrl.getNumJoints(); i++){
      jointStateSolver.position[i] = q_nullspace(i);
      solution(i) = q_nullspace(i);
    }

    q_guess.setZero(ctrl.getNumJoints());
    q_guess << DEFAULT_CONFIGURATION;

    ctrl.setJointLimitsFromUrdf(robot_description, jointStateSolver.name);
  }

  void target_callback(const geometry_msgs::PoseStampedConstPtr& msg){
    std::lock_guard<std::mutex> lock(target_mutex);
    target = *msg;
  }

  bool solve( std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
    VectorXd q_out(ctrl.getNumJoints());
    std::cout << "Num of joints is: " << ctrl.getNumJoints() << std::endl;
    std::cout << "Configuration guess is: " << q_guess.transpose() << std::endl;

    size_t i = 0;
    double pos_err = 0;
    double rot_err = 0;
    std::lock_guard<std::mutex> lock(target_mutex);
    Vector3d pos_in{target.pose.position.x, target.pose.position.y, target.pose.position.z};
    Matrix3d rot_in(Quaterniond(target.pose.orientation.w,
                                target.pose.orientation.x,
                                target.pose.orientation.y,
                                target.pose.orientation.z));
    Vector3d pos_out;
    Matrix3d rot_out;

    q_out = ctrl.computeCommand(rot_in, pos_in, q_guess);
    // reuse last solution as new guess
    q_guess = q_out;
    std::cout << "Computing the forward kinematics." << std::endl;
    ctrl.forwardKinematics(q_out, rot_out, pos_out);

    pos_err = (pos_out - pos_in).norm();
    rot_err = AngleAxisd(rot_out.transpose() * rot_in).angle();
    std::cout << "Test pose 1 " << std::endl;
    std::cout << "t_des: " << pos_in.transpose() << std::endl;
    std::cout << "R_des: " << Quaterniond(rot_in).coeffs().transpose() << std::endl;
    std::cout << "q_out: " << q_out.transpose() << std::endl;
    std::cout << "t_err: " << pos_err << std::endl;
    std::cout << "R_err: " << rot_err << std::endl;

    {
      std::lock_guard<std::mutex> solution_lock(solution_mutex);
      solution = q_out;
    }
    return true;
  }

  void publish_solution() {
    for (size_t i=0; i<ctrl.getNumJoints(); i++){
      jointStateSolver.position[i] = solution(i);
    }
    jointStateSolver.header.stamp = ros::Time::now();
    jointPublisher.publish(jointStateSolver);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "debug_ik_node");
  ros::NodeHandle nh;
  IKDebug ik_debug(nh);

  ros::Rate rate(20);
  while (ros::ok()){
    ik_debug.publish_solution();
    rate.sleep();
    ros::spinOnce();
  }

}