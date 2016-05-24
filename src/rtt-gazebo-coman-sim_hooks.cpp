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
        jointPositionFeedback.joint_feedback.angles(j) = gazebo_joints_[joints_idx_[j]]->GetAngle(0).Radian();
        jointVelocityFeedback.joint_feedback.velocities(j) = gazebo_joints_[joints_idx_[j]]->GetVelocity(0);

        gazebo::physics::JointWrench w =
                gazebo_joints_[joints_idx_[j]]->GetForceTorque(0u);
        gazebo::math::Vector3 a = gazebo_joints_[joints_idx_[j]]->GetLocalAxis(
                0u);
        jointTorqueFeedback.joint_feedback.torques(j) = a.Dot(w.body1Torque); // perhaps change to GetForceTorque

        // Reset commands from users
        jointTorqueCtrl.joint_cmd.torques.setZero();
    }


    jointTorqueCtrl.joint_cmd_fs = jointTorqueCtrl.orocos_port.readNewest(jointTorqueCtrl.joint_cmd);
    jointPositionCtrl.joint_cmd_fs = jointPositionCtrl.orocos_port.readNewest(jointPositionCtrl.joint_cmd);
    jointImpedanceCtrl.joint_cmd_fs = jointImpedanceCtrl.orocos_port.readNewest(jointImpedanceCtrl.joint_cmd);


    // write feedback to Orocos
    if (jointPositionFeedback.orocos_port.connected())
        jointPositionFeedback.orocos_port.write(jointPositionFeedback.joint_feedback);

    if (jointVelocityFeedback.orocos_port.connected())
        jointVelocityFeedback.orocos_port.write(jointVelocityFeedback.joint_feedback);

    if (jointTorqueFeedback.orocos_port.connected())
        jointTorqueFeedback.orocos_port.write(jointTorqueFeedback.joint_feedback);
}

void robotSim::WorldUpdateEnd() {
    if (!is_configured && !isRunning())
        return;

    if(currentControlMode == jointCtrlModes::ControlModes::JointPositionCtrl){
        for(unsigned int i = 0; i < joint_names_.size(); ++i)
        {
            gazebo::common::PID pid;
            pid.SetPGain(5000.);
            pid.SetDGain(5.);
            pid.SetIGain(0.);
            gazebo_joint_ctrl->SetPositionPID(joint_scoped_names_[i], pid);
            gazebo_joint_ctrl->SetPositionTarget(joint_scoped_names_[i], 0.0);
        }
    }



}
