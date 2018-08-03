#include "lwr_testdriver-component.hpp"
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <cmath>

Lwr_testdriver::Lwr_testdriver(std::string const& name) : TaskContext(name){
    maximum_torques.setOnes(6);
    this->addProperty("maximum_torques", maximum_torques).doc("Maximum torques to be generated");

    target_angles.setZero(6);
    target_angles << 0.35f, -1.57f, -1.57f, 1.57f, 0.35f, 0.0f;
    this->addProperty("target_angles", target_angles).doc("Target joint angles to be reached [rad]");

    epsilon = 0.001f;
    this->addProperty("epsilon", epsilon).doc("Desired precision [rad]");

    joint_state_in_port.doc("Joint state feedback port");
    joint_state_in_flow = RTT::NoData;
    this->addPort("jointStateIn", joint_state_in_port);

    torques_out_port.doc("Torque output port");
    this->addPort("torquesOut", torques_out_port);

    RTT::log(RTT::Info) << "Lwr_testdriver constructed" << RTT::endlog();
}

bool Lwr_testdriver::configureHook(){
    torques_out_data.torques.setZero(6);
    torques_out_port.setDataSample(torques_out_data);

    RTT::log(RTT::Info) << "Lwr_testdriver configured" << RTT::endlog();
    return true;
}

bool Lwr_testdriver::startHook(){
    torques_out_data.torques.setZero(6);

    RTT::log(RTT::Info) << "Lwr_testdriver started" << RTT::endlog();
    return true;
}

void Lwr_testdriver::updateHook(){
    // Read current state
    joint_state_in_flow = joint_state_in_port.read(joint_state_in_data);

    // For six joints in upper_arm
    for(counter = 0; counter < 6; counter++) {
        // Apply torque until target angles are within reach of epsilon
        if(std::abs(target_angles[counter] - joint_state_in_data.angles[counter]) > epsilon) {
            torques_out_data.torques[counter] = maximum_torques[counter]; // Simply apply maximum torques - works in simulation ;)

            // Actually move in correct direction
            if(target_angles[counter] - joint_state_in_data.angles[counter] < 0) {
                torques_out_data.torques[counter] *= -1;
            }
          } else {
            torques_out_data.torques[counter] = 0.0f;
          }
    }

    torques_out_port.write(torques_out_data);
}

void Lwr_testdriver::stopHook() {
    RTT::log(RTT::Info) << "Lwr_testdriver executes stopping" << RTT::endlog();
}

void Lwr_testdriver::cleanupHook() {
    RTT::log(RTT::Info) << "Lwr_testdriver cleaning up" << RTT::endlog();
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
