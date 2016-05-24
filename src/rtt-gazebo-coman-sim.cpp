#include "rtt-gazebo-coman-sim.hpp"
#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

robotSim::robotSim(const std::string &name):
    TaskContext(name),
    is_configured(false)
{
    this->provides("gazebo")->addOperation("WorldUpdateBegin",
            &robotSim::WorldUpdateBegin, this, RTT::ClientThread);
    this->provides("gazebo")->addOperation("WorldUpdateEnd",
            &robotSim::WorldUpdateEnd, this, RTT::ClientThread);

    this->addOperation("getModel", &robotSim::getModel, this, ClientThread);

    ///TODO: add ports according to the SRDF configuration!
    this->ports()->addPort(jointCtrlModes::positionCtrlPort, port_JointPositionCommand).doc(
            "Input for JointPosition-cmds from Orocos to Gazebo world.");
    this->ports()->addPort(jointCtrlModes::impedanceCtrlPort, port_JointImpedanceCommand).doc(
            "Input for JointImpedance-cmds from Orocos to Gazebo world.");
    this->ports()->addPort(jointCtrlModes::torqueCtrlPort, port_JointTorqueCommand).doc(
            "Input for JointTorque-cmds from Orocos to Gazebo world.");

    this->ports()->addPort(jointFeedback::velocityFeedbackPort, port_JointVelocity).doc(
            "Output for JointVelocity-fbs from Gazebo to Orocos world.");
    this->ports()->addPort(jointFeedback::torqueFeedbackPort, port_JointTorque).doc(
            "Output for JointTorques-fbs from Gazebo to Orocos world.");
    this->ports()->addPort(jointFeedback::positionFeedbackPort, port_JointPosition).doc(
            "Output for JointPosition-fbs from Gazebo to Orocos world.");

    this->addOperation("setControlMode", &robotSim::setControlMode,
                this, RTT::ClientThread);
    ///

    world_begin = gazebo::event::Events::ConnectWorldUpdateBegin(
            boost::bind(&robotSim::WorldUpdateBegin, this));
    world_end = gazebo::event::Events::ConnectWorldUpdateEnd(
            boost::bind(&robotSim::WorldUpdateEnd, this));
}

void robotSim::setControlMode(const std::string& controlMode) {
    if (controlMode == jointCtrlModes::positionCtrlPort) {
        currentControlMode = jointCtrlModes::ControlModes::JointPositionCtrl;
    } else if (controlMode == jointCtrlModes::torqueCtrlPort) {
        currentControlMode = jointCtrlModes::ControlModes::JointTorqueCtrl;
    } else if (controlMode == jointCtrlModes::impedanceCtrlPort) {
        currentControlMode = jointCtrlModes::ControlModes::JointImpedanceCtrl;
    }
}

bool robotSim::getModel(const std::string& gazebo_comp_name,
        const std::string& model_name, double timeout_s) {
    if (model) {
        log(Warning) << "Model [" << model_name << "] already loaded !"
                << endlog();
        return true;
    }
    gazebo::printVersion();
    if (!gazebo::physics::get_world()) {
        log(Error) << "getWorldPtr does not seem to exists" << endlog();
        return false;
    }
    model = gazebo::physics::get_world()->GetModel(model_name);
    if (model) {
        log(Info) << "Model [" << model_name << "] successfully loaded !"
                << endlog();
        return true;
    }
    return bool(model);
}

void robotSim::updateHook() {
}

bool robotSim::configureHook() {
    this->is_configured = gazeboConfigureHook(model);
    return is_configured;
}

bool robotSim::gazeboConfigureHook(gazebo::physics::ModelPtr model) {
    if (model.get() == NULL) {
        RTT::log(RTT::Error) << "No model could be loaded" << RTT::endlog();
        return false;
    }

    // Get the joints
    gazebo_joints_ = model->GetJoints();
    model_links_ = model->GetLinks();

    RTT::log(RTT::Info) << "Model name "<< model->GetName()
            << RTT::endlog();
    RTT::log(RTT::Info) << "Model has " << gazebo_joints_.size() << " joints"
            << RTT::endlog();
    RTT::log(RTT::Info) << "Model has " << model_links_.size() << " links"
            << RTT::endlog();

    //NOTE: Get the joint names and store their indices
    // Because we have base_joint (fixed), j0...j6, ati_joint (fixed)
    int idx = 0;
    joints_idx_.clear();
    for (gazebo::physics::Joint_V::iterator jit = gazebo_joints_.begin();
            jit != gazebo_joints_.end(); ++jit, ++idx) {

        const std::string name = (*jit)->GetName();
        // NOTE: Remove fake fixed joints (revolute with upper==lower==0
        // NOTE: This is not used anymore thanks to <disableFixedJointLumping>
        // Gazebo option (ati_joint is fixed but gazebo can use it )

        if ((*jit)->GetLowerLimit(0u) == (*jit)->GetUpperLimit(0u)) {
            RTT::log(RTT::Warning) << "Not adding (fake) fixed joint [" << name
                    << "] idx:" << idx << RTT::endlog();
            continue;
        }
        joints_idx_.push_back(idx);
        joint_names_.push_back(name);
        RTT::log(RTT::Info) << "Adding joint [" << name << "] idx:" << idx
                << RTT::endlog();
    }

    if (joints_idx_.size() == 0) {
        RTT::log(RTT::Error) << "No Joints could be added, exiting"
                << RTT::endlog();
        return false;
    }

    RTT::log(RTT::Info) << "Gazebo model found " << joints_idx_.size()
            << " joints " << RTT::endlog();

    jnt_pos_cmd_ = rstrt::kinematics::JointAngles(joints_idx_.size());
    jnt_pos_cmd_.angles.setZero();
    jnt_pos_ = rstrt::kinematics::JointAngles(joints_idx_.size());
    jnt_pos_.angles.setZero();

    jnt_trq_gazebo_cmd_ = rstrt::dynamics::JointTorques(joints_idx_.size());
    jnt_trq_gazebo_cmd_.torques.setZero();
    jnt_trq_cmd_ = rstrt::dynamics::JointTorques(joints_idx_.size());
    jnt_trq_cmd_.torques.setZero();
    jnt_trq_ = rstrt::dynamics::JointTorques(joints_idx_.size());
    jnt_trq_.torques.setZero();

    jnt_vel_cmd_ = rstrt::kinematics::JointVelocities(joints_idx_.size());
    jnt_vel_cmd_.velocities.setZero();
    jnt_vel_ = rstrt::kinematics::JointVelocities(joints_idx_.size());
    jnt_vel_.velocities.setZero();

    jnt_imp_cmd_ = rstrt::dynamics::JointImpedance(joints_idx_.size());
    jnt_imp_cmd_.damping.setZero();
    jnt_imp_cmd_.stiffness.setZero();
    jnt_imp_ = rstrt::dynamics::JointImpedance(joints_idx_.size());
    jnt_imp_.damping.setZero();
    jnt_imp_.stiffness.setZero();

    port_JointPosition.setDataSample(jnt_pos_);
    port_JointVelocity.setDataSample(jnt_vel_);
    port_JointTorque.setDataSample(jnt_trq_);


    RTT::log(RTT::Warning) << "Done configuring component" << RTT::endlog();
    return true;
}

ORO_CREATE_COMPONENT(cogimon::robotSim)

