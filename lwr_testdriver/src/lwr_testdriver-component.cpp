#include "lwr_testdriver-component.hpp"
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <eigen3/Eigen/Dense>
#include <kdl/frames_io.hpp>


Lwr_testdriver::Lwr_testdriver(std::string const& name) : TaskContext(name){
    positioning_torque = 1.0f;
    this->addProperty("positioning_torque", positioning_torque).doc("Torque to be generated in each joint for positioning");

    target_angles.setZero(7);
    target_angles << -30.0f, -25.0f, 80.0f, -80.0f, 90.0f, 45.0f, 0.0f;
    target_angles = target_angles * 3.141f/180.0f;
    this->addProperty("target_angles", target_angles).doc("Target joint angles to be reached [rad]");

    epsilon = 0.005f;
    this->addProperty("epsilon", epsilon).doc("Desired precision [rad]");

    hand_axis.setZero(6);
    this->addProperty("hand_axis", hand_axis).doc("Forces/torques in EE frame");
    this->addOperation("setForces", &Lwr_testdriver::setForceAxis, this).doc("Set forces in EE frame");

    mode = "none";
    this->addOperation("setMode", &Lwr_testdriver::setMode, this).doc("Set position, torque or none mode");

    this->addOperation("loadModel", &Lwr_testdriver::loadModel, this).doc("Load kinematic model from specified URDF file");
    this->addProperty("q", q).doc("Joint values");

    tau.setZero(7);
    this->addProperty("tau", tau).doc("Computed joint torques");

    joint_state_in_port.doc("Joint state feedback port");
    joint_state_in_flow = RTT::NoData;
    this->addPort("jointStateIn", joint_state_in_port);

    torques_out_port.doc("Torque output port");
    this->addPort("torquesOut", torques_out_port);

    this->addOperation("print", &Lwr_testdriver::printShit, this).doc("Print shit for debugging purposes");

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


bool Lwr_testdriver::startHook(){
    torques_out_data.torques.setZero(7);

    RTT::log(RTT::Info) << "Lwr_testdriver started" << RTT::endlog();
    return true;
}


void Lwr_testdriver::updateHook(){
    // Read current state
    joint_state_in_flow = joint_state_in_port.read(joint_state_in_data);

    if(joint_state_in_flow == RTT::NoData) {
        RTT::log(RTT::Error) << "No joint state input" << RTT::endlog();
        return;
    }

    q.data = joint_state_in_data.angles.tail(lwr.getNrOfJoints()).cast<double>();

    // If in torque mode, compute torques
    // Else drive to position/do nothing
    if(mode == "torque") {
        tau.setZero(7);
        tau.tail(lwr.getNrOfJoints()) = computeTorques(hand_axis.cast<double>()).cast<float>();
    } else if(mode == "position"){
        in_position = 0;

        // For all seven joints
        for(counter = 0; counter < 7; counter++) {

            if(target_angles[counter] - joint_state_in_data.angles[counter] > epsilon) {
                tau[counter] = positioning_torque;
            } else if(target_angles[counter] - joint_state_in_data.angles[counter] < -epsilon) {
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


bool Lwr_testdriver::loadModel(const std::string& model_path, const std::string& base_link) {
    model_loaded = false;

    if(!model.initFile(model_path)) {
        RTT::log(RTT::Error) << "Could not load model from URDF at " << model_path << RTT::endlog();
        return false;
    }

    if(!kdl_parser::treeFromUrdfModel(model, model_tree)) {
        RTT::log(RTT::Error) << "Could not get tree from model" << RTT::endlog();
        return false;
    }

    if(!model_tree.getChain(base_link, "lwr_arm_7_link", lwr)) {
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
    fk_solver_pos->JntToCart(q, hand);
    inv = hand.Inverse();

    htb.setZero(6, 6);
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


bool Lwr_testdriver::setMode(const std::string& mode) {
    if(mode == "position" || mode == "torque" || mode == "none") {
        this->mode = mode;
        return true;
    }

    RTT::log(RTT::Error) << "Available modes are position, torque and none" << RTT::endlog();
    return false;
}


void Lwr_testdriver::setForceAxis(float x, float y, float z){
    hand_axis[0] = x;
    hand_axis[1] = y;
    hand_axis[2] = z;
    hand_axis.tail<3>().setZero();
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

    std::cout<<"---------Segments---------"<<std::endl;
    std::cout<<lwr.getNrOfSegments()<<std::endl;
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
