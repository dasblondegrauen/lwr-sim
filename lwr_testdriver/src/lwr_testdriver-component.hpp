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
    bool setMode(std::string mode);
    void setForceAxis(float x, float y, float z);
    void printShit();

    Eigen::VectorXf target_angles;
    Eigen::VectorXf hand_axis;
    Eigen::VectorXf tau;
    Eigen::Index counter;
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
    KDL::Chain lwr;
    KDL::JntArray q;
    int ind_i, ind_j;

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_pos;
    std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    KDL::Frame hand;
    KDL::Frame inv;
    Eigen::Matrix<double, 6, 6> htb;
    KDL::Jacobian j;
    KDL::Jacobian j_htb;
};
#endif
