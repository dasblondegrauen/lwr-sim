#include "lwr_testdriver-component.hpp"
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <eigen3/Eigen/Dense>


Lwr_testdriver::Lwr_testdriver(std::string const& name) : TaskContext(name){
    positioning_torque = 1.0f;
    this->addProperty("positioning_torque", positioning_torque).doc("Torque to be generated in each joint for positioning");

    target_angles.setZero(7);
    //target_angles << 1.064f, 2.084f, -1.421f, 1.403f, 0.874f, -2.641f, -1.814f;
    target_angles << 0.0f, 0.35f, -1.57f, -1.57f, 1.57f, 0.35f, 0.0f;
    this->addProperty("target_angles", target_angles).doc("Target joint angles to be reached [rad]");

    pushing_axis.setZero(6);
    pushing_axis(2) = 1.0;
    this->addProperty("pushing_axis", pushing_axis).doc("Pushing direction in EE frame");

    epsilon = 0.005f; // TODO: Adjust?
    this->addProperty("epsilon", epsilon).doc("Desired precision [rad]");

    push = false;
    this->addProperty("push", push).doc("If true, pushing torques are applied as soon as target angles are reached");

    this->addOperation("loadModel", &Lwr_testdriver::loadModel, this).doc("Load kinematic model from specified URDF file");

    joint_state_upper_arm_in_port.doc("Upper arm joint state feedback port");
    joint_state_upper_arm_in_flow = RTT::NoData;
    this->addPort("jointStateUpperArmIn", joint_state_upper_arm_in_port);

    torques_upper_arm_out_port.doc("Upper arm torque output port");
    this->addPort("torquesUpperArmOut", torques_upper_arm_out_port);

    model_loaded = false;

    RTT::log(RTT::Info) << "Lwr_testdriver constructed" << RTT::endlog();
}


bool Lwr_testdriver::configureHook(){
    torques_upper_arm_out_data.torques.setZero(6);
    torques_upper_arm_out_port.setDataSample(torques_upper_arm_out_data);

    if(!model_loaded) {
        RTT::log(RTT::Error) << "No model loaded" << RTT::endlog();
        return false;
    }

    RTT::log(RTT::Info) << "Lwr_testdriver configured" << RTT::endlog();
    return true;
}


bool Lwr_testdriver::startHook(){
    torques_upper_arm_out_data.torques.setZero(6);

    RTT::log(RTT::Info) << "Lwr_testdriver started" << RTT::endlog();
    return true;
}


void Lwr_testdriver::updateHook(){
    // Read current state
    joint_state_upper_arm_in_flow = joint_state_upper_arm_in_port.read(joint_state_upper_arm_in_data);
    q.data = joint_state_upper_arm_in_data.angles.cast<double>();

    // If in position & pushing enabled, push!
    // Else drive to position
    if(push && in_position == 6) {
        torques_upper_arm_out_data.torques = computeTorques(pushing_axis).tail<6>().cast<float>();
    } else {
        in_position = 0;

        // For six joints in upper_arm
        for(counter = 0; counter < 6; counter++) {

            // Apply torque until target angles are within reach of epsilon
            if(std::abs(target_angles[counter+1] - joint_state_upper_arm_in_data.angles[counter]) > epsilon) {

                // Actually move in correct direction
                if(target_angles[counter+1] - joint_state_upper_arm_in_data.angles[counter] < 0) {
                    torques_upper_arm_out_data.torques[counter] = -1.0f * positioning_torque;
                } else {
                    torques_upper_arm_out_data.torques[counter] = positioning_torque;
                }
            } else {
                torques_upper_arm_out_data.torques[counter] = 0.0f;
                in_position++;
            }
        }
    }

    torques_upper_arm_out_port.write(torques_upper_arm_out_data);
}


void Lwr_testdriver::stopHook() {
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


Eigen::VectorXd Lwr_testdriver::computeTorques(Eigen::Matrix<double, 6, 1>& axis, double magnitude) {
    // TODO Do not declare variables here!
    KDL::Frame ee;
    fk_solver_pos->JntToCart(q, ee);

    Eigen::Matrix<double, 6, 6> htb;
    htb.Zero(6, 6);

    auto inv = ee.Inverse();
    for(ind_j = 0; ind_j < 3; ind_j++) {
        for(ind_i = 0; ind_i < 3; ind_i++) {
            htb(ind_i, ind_j) = static_cast<double>(inv(ind_i, ind_j));
            htb(3 + ind_i, 3 + ind_j) = static_cast<double>(inv(ind_i, ind_j));
            RTT::log(RTT::Info) << ind_i << ", " << ind_j << RTT::endlog();
        }
    }

    jnt_to_jac_solver->JntToJac(q, j);
    j_htb.data = htb * j.data;

    return j_htb.data.transpose() * axis * magnitude;
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
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Lwr_testdriver)
