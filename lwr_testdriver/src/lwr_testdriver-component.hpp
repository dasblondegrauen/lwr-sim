#ifndef OROCOS_LWR_TESTDRIVER_COMPONENT_HPP
#define OROCOS_LWR_TESTDRIVER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <eigen3/Eigen/Dense>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/robot/JointState.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

class Lwr_testdriver : public RTT::TaskContext{
public:
    Lwr_testdriver(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    bool loadModel(const std::string& model_path);
    Eigen::VectorXd computeTorques(const Eigen::Matrix<double, 6, 1>& axis, const double magnitude = 1.0);

    Eigen::VectorXf target_angles;
    Eigen::Matrix<float, 6, 1> hand_axis;
    Eigen::Matrix<float, 7, 1> tau;
    Eigen::Index counter;
    float positioning_torque;
    float epsilon;
    unsigned short in_position = 0;
    bool push = false;

    RTT::InputPort<rstrt::robot::JointState> joint_state_base_in_port;
    RTT::InputPort<rstrt::robot::JointState> joint_state_upper_arm_in_port;
    RTT::FlowStatus joint_state_base_in_flow;
    RTT::FlowStatus joint_state_upper_arm_in_flow;
    rstrt::robot::JointState joint_state_base_in_data;
    rstrt::robot::JointState joint_state_upper_arm_in_data;

    RTT::OutputPort<rstrt::dynamics::JointTorques> torques_base_out_port;
    RTT::OutputPort<rstrt::dynamics::JointTorques> torques_upper_arm_out_port;
    rstrt::dynamics::JointTorques torques_base_out_data;
    rstrt::dynamics::JointTorques torques_upper_arm_out_data;


    bool model_loaded = true;
    void setForceAxis(double x, double y, double z);
    urdf::Model model;
    KDL::Tree model_tree;
    KDL::Chain lwr;
    KDL::JntArray q;
    int ind_i, ind_j;

    void printShit();

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_pos;
    std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    KDL::Frame hand;
    KDL::Frame inv;
    Eigen::Matrix<double, 6, 6> htb;
    KDL::Jacobian j;
    KDL::Jacobian j_htb;
};
#endif
