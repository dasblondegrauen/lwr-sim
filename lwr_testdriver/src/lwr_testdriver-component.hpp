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
    bool loadModel(const std::string& model_path, const std::string& lower_tip_link="lwr_arm_3_link", const std::string& upper_root_link="lwr_arm_3_link");
    Eigen::VectorXd computeTorquesUpper(const Eigen::Matrix<double, 6, 1>& axis, const double magnitude = 1.0);
    Eigen::VectorXd computeTorquesLower(const Eigen::Matrix<double, 6, 1>& axis, const double magnitude = 1.0);
    bool setMode(const std::string& mode);
    void setForceAxisUpper(float x, float y, float z);
    void setForceAxisLower(float x, float y, float z);
    void setTorqueAxisUpper(float x, float y, float z);
    void setTorqueAxisLower(float x, float y, float z);
    Eigen::VectorXd controlPID(const Eigen::VectorXd& target, const Eigen::VectorXd& current);
    void averageTau(int frames);
    void printShit();

    Eigen::VectorXf target_angles;
    Eigen::VectorXf hand_forces;
    Eigen::VectorXf elbow_forces;
    Eigen::VectorXf tau;
    Eigen::Index joint_counter;
    float positioning_torque;
    float epsilon;
    unsigned short in_position = 0;
    std::string mode;

    RTT::InputPort<rstrt::robot::JointState> joint_state_in_port;
    RTT::FlowStatus joint_state_in_flow;
    rstrt::robot::JointState joint_state_in_data;

    RTT::OutputPort<rstrt::dynamics::JointTorques> torques_out_port;
    rstrt::dynamics::JointTorques torques_out_data;


    bool model_loaded = true;
    urdf::Model model;
    KDL::Tree model_tree;
    KDL::Chain lower, upper;
    KDL::JntArray q_lower, q_upper;
    KDL::Jacobian j_lower, j_upper;
    int ind_i, ind_j;

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_pos_lower, fk_solver_pos_upper;
    std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_lower, jnt_to_jac_solver_upper;
    KDL::Frame tip_lower, tip_upper;
    KDL::Frame inv_lower, inv_upper;
    Eigen::Matrix<double, 6, 6> htb_lower, htb_upper;
    KDL::Jacobian j_htb_lower, j_htb_upper;
    bool elbow_to_base = true;

    Eigen::VectorXd e_current, e_total, e_previous;
    double k_p = 1, k_i = 1, k_d = 1;
    bool enable_pid = false;

    Eigen::VectorXf tau_sum;
    int frames_total = 0, frames_counter = 0;
};
#endif
