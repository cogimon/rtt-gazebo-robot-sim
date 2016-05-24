#include "rtt-gazebo-coman-sim.hpp"
#include <Eigen/Dense>

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

void robotSim::WorldUpdateBegin() {
    if (!is_configured && !isRunning())
        return;

    // Get state
    for (unsigned j = 0; j < joints_idx_.size(); j++) {
        jnt_pos_.angles(j) = gazebo_joints_[joints_idx_[j]]->GetAngle(0).Radian();
        jnt_vel_.velocities(j) = gazebo_joints_[joints_idx_[j]]->GetVelocity(0);

        gazebo::physics::JointWrench w =
                gazebo_joints_[joints_idx_[j]]->GetForceTorque(0u);
        gazebo::math::Vector3 a = gazebo_joints_[joints_idx_[j]]->GetLocalAxis(
                0u);
        jnt_trq_.torques(j) = a.Dot(w.body1Torque); // perhaps change to GetForceTorque

        // Reset commands from users
        jnt_trq_cmd_.torques.setZero();
        jnt_trq_gazebo_cmd_.torques.setZero();
    }


    jnt_trq_cmd_fs = port_JointTorqueCommand.readNewest(jnt_trq_cmd_);
    jnt_pos_cmd_fs = port_JointPositionCommand.readNewest(jnt_pos_cmd_);
    jnt_imp_cmd_fs = port_JointImpedanceCommand.readNewest(jnt_imp_cmd_);


    // write feedback to Orocos
    if (port_JointPosition.connected())
        port_JointPosition.write(jnt_pos_);

    if (port_JointVelocity.connected())
        port_JointVelocity.write(jnt_vel_);

    if (port_JointTorque.connected())
        port_JointTorque.write(jnt_trq_);
}

void robotSim::WorldUpdateEnd() {
    if (!is_configured && !isRunning())
        return;
}
