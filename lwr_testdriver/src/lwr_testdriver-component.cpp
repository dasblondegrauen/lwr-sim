#include "lwr_testdriver-component.hpp"
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <eigen3/Eigen/Dense>
#include <kdl/frames_io.hpp>

Lwr_testdriver::Lwr_testdriver(std::string const& name) : TaskContext(name){
    positioning_torque = 1.0f;
    this->addProperty("positioning_torque", positioning_torque).doc("Torque to be generated in each joint for positioning");

    target_angles.setZero(7);
    //target_angles << 1.064f, 2.084f, -1.421f, 1.403f, 0.874f, -2.641f, -1.814f;
    target_angles << 1.57f, 0.35f, -1.57f, -1.57f, 1.57f, 0.35f, 0.0f;
    this->addProperty("target_angles", target_angles).doc("Target joint angles to be reached [rad]");

    epsilon = 0.005f; // TODO: Adjust?
    this->addProperty("epsilon", epsilon).doc("Desired precision [rad]");

    hand_axis.setZero(6);
    hand_axis(2) = 1.0f;
    this->addProperty("hand_axis", hand_axis).doc("Pushing direction in EE frame");

    push = false;
    this->addProperty("push", push).doc("If true, pushing torques are applied as soon as target angles are reached");

    this->addOperation("loadModel", &Lwr_testdriver::loadModel, this).doc("Load kinematic model from specified URDF file");
    this->addProperty("q", q).doc("Joint values");

    tau.setZero(7);
    this->addProperty("tau", tau).doc("Computed joint torques");

    joint_state_in_port.doc("Joint state feedback port");
    joint_state_in_flow = RTT::NoData;
    this->addPort("jointStateIn", joint_state_in_port);

    torques_out_port.doc("Torque output port");
    this->addPort("torquesOut", torques_out_port);

    this->addOperation("setForces", &Lwr_testdriver::setForceAxis, this, RTT::ClientThread);
    this->addOperation("print", &Lwr_testdriver::printShit, this, RTT::ClientThread);

    model_loaded = false;

    RTT::log(RTT::Info) << "Lwr_testdriver constructed" << RTT::endlog();
}


bool Lwr_testdriver::configureHook(){
    torques_out_data.torques.setZero(7);
    torques_out_port.setDataSample(torques_out_data);

    if(!model_loaded) {
        RTT::log(RTT::Error) << "No model loaded" << RTT::endlog();
        return false;
    }

    RTT::log(RTT::Info) << "Lwr_testdriver configured" << RTT::endlog();
    return true;
}

void Lwr_testdriver::setForceAxis(double x, double y, double z){
    hand_axis[0] = x;
    hand_axis[1] = y;
    hand_axis[2] = z;
    hand_axis[3] = 0.0;
    hand_axis[4] = 0.0;
    hand_axis[5] = 0.0;
}


bool Lwr_testdriver::startHook(){
    torques_out_data.torques.setZero(7);

    RTT::log(RTT::Info) << "Lwr_testdriver started" << RTT::endlog();
    return true;
}


void Lwr_testdriver::updateHook(){
    // Read current state
    joint_state_in_flow = joint_state_in_port.read(joint_state_in_data);
    q.data = joint_state_in_data.angles.cast<double>();

    // If pushing enabled (and in position), push!
    // Else drive to position
    if(push /*&& in_position == 7*/) {
        tau = computeTorques(hand_axis.cast<double>()).cast<float>();
    } else {
        in_position = 0;

        // For all seven joints
        for(counter = 0; counter < 7; counter++) {

            if(target_angles[counter] - q(counter) > epsilon) {
                tau[counter] = positioning_torque;
            } else if(target_angles[counter] - q(counter) < -epsilon) {
                tau[counter] = -positioning_torque;
            } else {
                tau[counter] = 0.0f;
                in_position++;
            }
        }
    }

    // Write torques to their respective output ports
    torques_out_data.torques = tau;
    torques_out_port.write(torques_out_data);
}


void Lwr_testdriver::stopHook() {
    torques_out_data.torques.setZero(7);

    RTT::log(RTT::Info) << "Lwr_testdriver executes stopping" << RTT::endlog();
}


void Lwr_testdriver::cleanupHook() {
    RTT::log(RTT::Info) << "Lwr_testdriver cleaning up" << RTT::endlog();
}


bool Lwr_testdriver::loadModel(const std::string& model_path) {
    model_loaded = false;

    if(!model.initFile(model_path)) {
        RTT::log(RTT::Error) << "Could not load model from URDF at " << model_path << RTT::endlog();
        return false;
    }

    if(!kdl_parser::treeFromUrdfModel(model, model_tree)) {
        RTT::log(RTT::Error) << "Could not get tree from model" << RTT::endlog();
        return false;
    }

    if(!model_tree.getChain("lwr_arm_base_link", "lwr_arm_7_link", lwr)) {
        RTT::log(RTT::Error) << "Could not get chain from tree" << RTT::endlog();
        return false;
    }

    q = KDL::JntArray(lwr.getNrOfJoints());
    j = KDL::Jacobian(lwr.getNrOfJoints());
    fk_solver_pos = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(lwr));
    jnt_to_jac_solver = std::unique_ptr<KDL::ChainJntToJacSolver>(new KDL::ChainJntToJacSolver(lwr));

    model_loaded = true;
    return true;
}


Eigen::VectorXd Lwr_testdriver::computeTorques(const Eigen::Matrix<double, 6, 1>& axis, const double magnitude) {
    fk_solver_pos->JntToCart(q, hand, lwr.getNrOfSegments()-1);
    inv = hand.Inverse();

    htb.Zero(6, 6);
    for(ind_j = 0; ind_j < 3; ind_j++) {
        for(ind_i = 0; ind_i < 3; ind_i++) {
            htb(ind_i, ind_j) = static_cast<double>(inv(ind_i, ind_j));
            htb(3 + ind_i, 3 + ind_j) = static_cast<double>(inv(ind_i, ind_j));
        }
    }

    jnt_to_jac_solver->JntToJac(q, j);
    j_htb.data = htb * j.data;

    return j_htb.data.transpose() * axis * magnitude;
}

void Lwr_testdriver::printShit(){
    std::cout<<"---------HTB--------------"<<std::endl;
    std::cout<<htb<<std::endl;

    std::cout<<"---------JAC--------------"<<std::endl;
    std::cout<<j.data<<std::endl;

    std::cout<<"---------INV--------------"<<std::endl;
    std::cout<<inv<<std::endl;

    std::cout<<"---------TAU--------------"<<std::endl;
    std::cout<<torques_out_data<<std::endl;
}


/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Lwr_testdriver)
 * In case you want to link with another library that
 * already contains components.
 *
 * .f you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Lwr_testdriver)
