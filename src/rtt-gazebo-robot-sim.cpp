#include "rtt-gazebo-robot-sim.hpp"
#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>

#include "pid_values_tmp.h"

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
    this->ports()->addPort(ControlModes::JointPositionCtrl, jointPositionCtrl.orocos_port).doc(
            "Input for JointPosition-cmds from Orocos to Gazebo world.");
    this->ports()->addPort(ControlModes::JointImpedanceCtrl, jointImpedanceCtrl.orocos_port).doc(
            "Input for JointImpedance-cmds from Orocos to Gazebo world.");
    this->ports()->addPort(ControlModes::JointTorqueCtrl, jointTorqueCtrl.orocos_port).doc(
            "Input for JointTorque-cmds from Orocos to Gazebo world.");

    this->ports()->addPort(FeedbackModesPorts::velocityFeedbackPort, jointVelocityFeedback.orocos_port).doc(
            "Output for JointVelocity-fbs from Gazebo to Orocos world.");
    this->ports()->addPort(FeedbackModesPorts::torqueFeedbackPort, jointTorqueFeedback.orocos_port).doc(
            "Output for JointTorques-fbs from Gazebo to Orocos world.");
    this->ports()->addPort(FeedbackModesPorts::positionFeedbackPort, jointPositionFeedback.orocos_port).doc(
            "Output for JointPosition-fbs from Gazebo to Orocos world.");

    this->addOperation("setControlMode", &robotSim::setControlMode,
                this, RTT::ClientThread);
    ///

    world_begin = gazebo::event::Events::ConnectWorldUpdateBegin(
            boost::bind(&robotSim::WorldUpdateBegin, this));
    world_end = gazebo::event::Events::ConnectWorldUpdateEnd(
            boost::bind(&robotSim::WorldUpdateEnd, this));
}

bool robotSim::setControlMode(const std::string& controlMode) {
    if(controlMode == ControlModes::JointPositionCtrl){
        if(!initGazeboJointController()){
            RTT::log(RTT::Warning) << "Can NOT initialize Gazebo Joint Controller!" << RTT::endlog();
            return false;}
        else
            setInitialPosition();
    }
    else if(controlMode == ControlModes::JointTorqueCtrl || controlMode == ControlModes::JointImpedanceCtrl)
        gazebo_joint_ctrl->Reset();
    else {
        RTT::log(RTT::Warning) << "Control Mode " << controlMode << " does not exist!" << RTT::endlog();
        RTT::log(RTT::Warning) << "Available Control Modes are:" << RTT::endlog();
        RTT::log(RTT::Warning) << "     "<< ControlModes::JointPositionCtrl << RTT::endlog();
        RTT::log(RTT::Warning) << "     "<< ControlModes::JointImpedanceCtrl << RTT::endlog();
        RTT::log(RTT::Warning) << "     "<< ControlModes::JointTorqueCtrl << RTT::endlog();
        return false;}

    currentControlMode = controlMode;
    return true;
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

//Here we already have the model!
bool robotSim::gazeboConfigureHook(gazebo::physics::ModelPtr model) {
    if (model.get() == NULL) {
        RTT::log(RTT::Error) << "No model could be loaded" << RTT::endlog();
        return false;
    }

    gazebo_joint_ctrl.reset(new gazebo::physics::JointController(model));

    // Get the joints
    gazebo_joints_ = model->GetJoints();
    model_links_ = model->GetLinks();

    RTT::log(RTT::Info) << "Model name "<< model->GetName() << RTT::endlog();
    RTT::log(RTT::Info) << "Model has " << gazebo_joints_.size() << " joints" << RTT::endlog();
    RTT::log(RTT::Info) << "Model has " << model_links_.size() << " links" << RTT::endlog();

    if(!setJointNamesAndIndices())
        return false;

    RTT::log(RTT::Info) << "Gazebo model found " << joints_idx_.size() << " joints " << RTT::endlog();

    jointPositionCtrl.joint_cmd = rstrt::kinematics::JointAngles(joints_idx_.size());
    jointPositionCtrl.joint_cmd.angles.setZero();
    jointPositionFeedback.joint_feedback= rstrt::kinematics::JointAngles(joints_idx_.size());
    jointPositionFeedback.joint_feedback.angles.setZero();

    jointTorqueCtrl.joint_cmd = rstrt::dynamics::JointTorques(joints_idx_.size());
    jointTorqueCtrl.joint_cmd.torques.setZero();
    jointTorqueFeedback.joint_feedback = rstrt::dynamics::JointTorques(joints_idx_.size());
    jointTorqueFeedback.joint_feedback.torques.setZero();

    jointVelocityFeedback.joint_feedback = rstrt::kinematics::JointVelocities(joints_idx_.size());
    jointVelocityFeedback.joint_feedback.velocities.setZero();

    jointImpedanceCtrl.joint_cmd = rstrt::dynamics::JointImpedance(joints_idx_.size());
    jointImpedanceCtrl.joint_cmd.damping.setZero();
    jointImpedanceCtrl.joint_cmd.stiffness.setZero();

    jointPositionFeedback.orocos_port.setDataSample(jointPositionFeedback.joint_feedback);
    jointVelocityFeedback.orocos_port.setDataSample(jointVelocityFeedback.joint_feedback);
    jointTorqueFeedback.orocos_port.setDataSample(jointTorqueFeedback.joint_feedback);


    currentControlMode = ControlModes::JointPositionCtrl;
    RTT::log(RTT::Info) << "Initial default Ctrl Mode is " << currentControlMode << RTT::endlog();

    if(!initGazeboJointController()){
        RTT::log(RTT::Error) << "Joint Controller can NOT be initialized, exiting" << RTT::endlog();
        return false;
    }
    setInitialPosition();


    RTT::log(RTT::Warning) << "Done configuring component" << RTT::endlog();
    return true;
}

bool robotSim::initGazeboJointController()
{
    for(unsigned int i = 0; i < joint_names_.size(); ++i)
            gazebo_joint_ctrl->AddJoint(model->GetJoint(joint_names_[i]));

    if(!(joint_names_.size() > 0))
        return false;

    hardcoded_pids PID;
    for(unsigned int i = 0; i < joint_scoped_names_.size(); ++i)
        gazebo_joint_ctrl->SetPositionPID(joint_scoped_names_[i], PID.pids[joint_names_[i]]);

    return true;
}

void robotSim::setInitialPosition()
{
    ///TODO: check if user initial config is set when it is used in the gazebo configure hook

    jointPositionCtrl.orocos_port.clear();
    for(unsigned int i = 0; i < joint_names_.size(); ++i)
        jointPositionCtrl.joint_cmd.angles[i] = model->GetJoint(joint_names_[i])->GetAngle(0).Radian();
    jointPositionCtrl.joint_cmd_fs = RTT::FlowStatus::NewData;
}

bool robotSim::setJointNamesAndIndices()
{
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
        joint_scoped_names_.push_back((*jit)->GetScopedName());
        RTT::log(RTT::Info) << "Adding joint [" << name << "] idx:" << idx << RTT::endlog();
    }

    if (joints_idx_.size() == 0) {
        RTT::log(RTT::Error) << "No Joints could be added, exiting" << RTT::endlog();
        return false;
    }

    return true;
}

ORO_CREATE_COMPONENT(cogimon::robotSim)

