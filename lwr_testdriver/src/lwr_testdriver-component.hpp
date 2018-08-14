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
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>

class Lwr_testdriver : public RTT::TaskContext{
public:
    Lwr_testdriver(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    Eigen::VectorXf positioning_torques;
    Eigen::VectorXf pushing_torques;
    Eigen::VectorXf target_angles;
    std::string model_path;
    float epsilon;
    bool push = false;

    RTT::InputPort<rstrt::robot::JointState> joint_state_in_port;
    RTT::FlowStatus joint_state_in_flow;
    rstrt::robot::JointState joint_state_in_data;

    RTT::OutputPort<rstrt::dynamics::JointTorques> torques_out_port;
    rstrt::dynamics::JointTorques torques_out_data;

    Eigen::Index counter;
    unsigned short in_position = 0;

    urdf::Model model;
    KDL::Tree model_tree;
    KDL::Chain lwr;
};
#endif
