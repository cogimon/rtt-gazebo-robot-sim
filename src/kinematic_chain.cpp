#include <kinematic_chain.h>
#include <pid_values_tmp.h>


using namespace cogimon;
KinematicChain::KinematicChain(const std::string& chain_name, RTT::DataFlowInterface& ports,
                               gazebo::physics::ModelPtr model):
    _kinematic_chain_name(chain_name),
    _ports(ports),
    _model(model),
    _current_control_mode(std::string(ControlModes::JointPositionCtrl))
{

}

bool KinematicChain::initKinematicChain()
{
    setJointNamesAndIndices();
    _number_of_dofs = _map_joint_name_indices.size();

    if(!setController(std::string(ControlModes::JointPositionCtrl)))
        return false;
    if(!setController(std::string(ControlModes::JointImpedanceCtrl)))
        return false;
    if(!setController(std::string(ControlModes::JointTorqueCtrl)))
        return false;

    if(!setFeedBack(std::string(FeedbackModes::positionFeedback)))
        return false;
    if(!setFeedBack(std::string(FeedbackModes::velocityFeedback)))
        return false;
    if(!setFeedBack(std::string(FeedbackModes::torqueFeedback)))
        return false;

    if(!initGazeboJointController()){
        RTT::log(RTT::Error) << "Joint Controller can NOT be initialized, exiting" << RTT::endlog();
        return false;
    }
    setInitialPosition();
    return true;
}

std::vector<std::string> KinematicChain::getJointScopedNames()
{
    std::map<std::string, std::string>::iterator it;
    std::vector<std::string> joint_names;
    for(it = _map_joint_name_scoped_name.begin(); it != _map_joint_name_scoped_name.end(); it++)
        joint_names.push_back(it->second);
    return joint_names;
}

std::vector<std::string> KinematicChain::getJointNames()
{
    std::map<std::string, unsigned int>::iterator it;
    std::vector<std::string> joint_names;
    for(it = _map_joint_name_indices.begin(); it != _map_joint_name_indices.end(); it++)
        joint_names.push_back(it->first);
    return joint_names;
}

std::vector<unsigned int> KinematicChain::getJointIndices()
{
    std::map<std::string, unsigned int>::iterator it;
    std::vector<unsigned int> joint_indices;
    for(it = _map_joint_name_indices.begin(); it != _map_joint_name_indices.end(); it++)
        joint_indices.push_back(it->second);
    return joint_indices;
}

std::vector<std::string> KinematicChain::getControllersAvailable()
{
    return _controllers_name;
}

std::string KinematicChain::getKinematicChainName()
{
    return _kinematic_chain_name;
}

unsigned int KinematicChain::getNumberOfDOFs()
{
    return _number_of_dofs;
}

std::string KinematicChain::getCurrentControlMode()
{
    return _current_control_mode;
}

bool KinematicChain::setFeedBack(const std::string &feedback_type)
{
    if(feedback_type == FeedbackModes::positionFeedback){
        position_feedback.reset(new position_fbk);
        _ports.addPort(_kinematic_chain_name+"/"+FeedbackModes::positionFeedback, position_feedback->orocos_port).doc(
                "Output for JointPosition-fbs from Gazebo to Orocos world.");
        position_feedback->joint_feedback = JointAngles(_number_of_dofs);
        position_feedback->joint_feedback.angles.setZero();

        position_feedback->orocos_port.setDataSample(position_feedback->joint_feedback);
    }
    else if(feedback_type == FeedbackModes::velocityFeedback){
        velocity_feedback.reset(new velocity_fbk);
        _ports.addPort(_kinematic_chain_name+"/"+FeedbackModes::velocityFeedback, velocity_feedback->orocos_port).doc(
                "Output for JointVelocity-fbs from Gazebo to Orocos world.");
        velocity_feedback->joint_feedback = JointVelocities(_number_of_dofs);
        velocity_feedback->joint_feedback.velocities.setZero();

        velocity_feedback->orocos_port.setDataSample(velocity_feedback->joint_feedback);
    }
    else if(feedback_type == FeedbackModes::torqueFeedback){
        torque_feedback.reset(new torque_fbk);
        _ports.addPort(_kinematic_chain_name+"/"+FeedbackModes::torqueFeedback, torque_feedback->orocos_port).doc(
                "Output for JointTorques-fbs from Gazebo to Orocos world.");
        torque_feedback->joint_feedback = JointTorques(_number_of_dofs);
        torque_feedback->joint_feedback.torques.setZero();

        torque_feedback->orocos_port.setDataSample(torque_feedback->joint_feedback);
    }
    else{
        RTT::log(RTT::Error) << "Feedback Mode: " << feedback_type << " is not available!" << RTT::endlog();
        return false;}
    return true;
}

bool KinematicChain::setController(const std::string& controller_type)
{
    if(controller_type == ControlModes::JointPositionCtrl){
        position_controller.reset(new position_ctrl);
        _ports.addPort(_kinematic_chain_name+"/"+ControlModes::JointPositionCtrl, position_controller->orocos_port).doc(
                "Input for JointPosition-cmds from Orocos to Gazebo world.");

        position_controller->joint_cmd = JointAngles(_number_of_dofs);
        position_controller->joint_cmd.angles.setZero();

        _gazebo_position_joint_controller.reset(new gazebo::physics::JointController(_model));
    }
    else if(controller_type == ControlModes::JointImpedanceCtrl){
        impedance_controller.reset(new impedance_ctrl);
        _ports.addPort(_kinematic_chain_name+"/"+ControlModes::JointImpedanceCtrl, impedance_controller->orocos_port).doc(
                "Input for JointImpedance-cmds from Orocos to Gazebo world.");
        impedance_controller->joint_cmd = JointImpedance(_number_of_dofs);
        impedance_controller->joint_cmd.stiffness.setZero();
        impedance_controller->joint_cmd.damping.setZero();
    }
    else if(controller_type == ControlModes::JointTorqueCtrl)
    {
        torque_controller.reset(new torque_ctrl);
        _ports.addPort(_kinematic_chain_name+"/"+ControlModes::JointTorqueCtrl, torque_controller->orocos_port).doc(
                "Input for JointTorque-cmds from Orocos to Gazebo world.");
        torque_controller->joint_cmd = JointTorques(_number_of_dofs);
        torque_controller->joint_cmd.torques.setZero();
    }
    else{
        RTT::log(RTT::Error) << "Control Mode: " << controller_type << " is not available!" << RTT::endlog();
        return false;}
    _controllers_name.push_back(controller_type);
    return true;
}

bool KinematicChain::setJointNamesAndIndices()
{
    //NOTE: Get the joint names and store their indices
    // Because we have base_joint (fixed), j0...j6, ati_joint (fixed)
    int idx = 0;
    for (gazebo::physics::Joint_V::const_iterator jit = _model->GetJoints().begin();
            jit != _model->GetJoints().end(); ++jit, ++idx) {

        const std::string name = (*jit)->GetName();
        // NOTE: Remove fake fixed joints (revolute with upper==lower==0
        // NOTE: This is not used anymore thanks to <disableFixedJointLumping>
        // Gazebo option (ati_joint is fixed but gazebo can use it )

        if ((*jit)->GetLowerLimit(0u) == (*jit)->GetUpperLimit(0u)) {
            RTT::log(RTT::Warning) << "Not adding (fake) fixed joint [" << name
                    << "] idx:" << idx << RTT::endlog();
            continue;
        }
        _map_joint_name_indices.insert(std::pair<std::string, unsigned int>(name, idx));
        _map_joint_name_scoped_name.insert(std::pair<std::string, std::string>(name, (*jit)->GetScopedName()));
        RTT::log(RTT::Info) << "Adding joint [" << name << "] idx:" << idx << RTT::endlog();
    }

    if (_map_joint_name_indices.size() == 0) {
        RTT::log(RTT::Error) << "No Joints could be added, exiting" << RTT::endlog();
        return false;
    }

    return true;
}

bool KinematicChain::initGazeboJointController()
{
    std::vector<std::string> joint_names = getJointNames();
    for(unsigned int i = 0; i < joint_names.size(); ++i)
            _gazebo_position_joint_controller->AddJoint(_model->GetJoint(joint_names[i]));

    hardcoded_pids PID;
    std::vector<std::string> joint_scoped_names = getJointScopedNames();
    for(unsigned int i = 0; i < joint_scoped_names.size(); ++i)
        _gazebo_position_joint_controller->SetPositionPID(joint_scoped_names[i], PID.pids[joint_names[i]]);

    return true;
}

void KinematicChain::setInitialPosition()
{
    ///TODO: check if user initial config is set when it is used in the gazebo configure hook

    position_controller->orocos_port.clear();
    std::vector<std::string> joint_names = getJointNames();
    for(unsigned int i = 0; i < joint_names.size(); ++i)
        position_controller->joint_cmd.angles[i] = _model->GetJoint(joint_names[i])->GetAngle(0).Radian();
    position_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
}

bool KinematicChain::setControlMode(const std::string &controlMode)
{
    if(controlMode != ControlModes::JointPositionCtrl &&
       controlMode != ControlModes::JointTorqueCtrl   &&
       controlMode != ControlModes::JointImpedanceCtrl ){
        RTT::log(RTT::Warning) << "Control Mode " << controlMode << " does not exist!" << RTT::endlog();
        return false;
    }

    if(!(std::find(_controllers_name.begin(), _controllers_name.end(), controlMode) != _controllers_name.end())){
        RTT::log(RTT::Warning) << "Control Mode " << controlMode << " is not available!" << RTT::endlog();
        return false;
    }

    if(controlMode == ControlModes::JointPositionCtrl){
        if(!initGazeboJointController()){
            RTT::log(RTT::Warning) << "Can NOT initialize Gazebo Joint Controller!" << RTT::endlog();
            return false;}
        else
            setInitialPosition();
    }
    else if(controlMode == ControlModes::JointTorqueCtrl || controlMode == ControlModes::JointImpedanceCtrl)
        _gazebo_position_joint_controller->Reset();

    _current_control_mode = controlMode;
    return true;
}

void KinematicChain::sense()
{
    std::vector<std::string> joint_names = getJointNames();
    if(position_feedback){
        for(unsigned int i = 0; i < _number_of_dofs; ++i)
            position_feedback->joint_feedback.angles(i) =
                _model->GetJoints()[_map_joint_name_indices[joint_names[i]]]->GetAngle(0).Radian();

        if (position_feedback->orocos_port.connected())
            position_feedback->orocos_port.write(position_feedback->joint_feedback);
    }
    if(velocity_feedback){
        for(unsigned int i = 0; i < _number_of_dofs; ++i)
            velocity_feedback->joint_feedback.velocities(i) =
                _model->GetJoints()[_map_joint_name_indices[joint_names[i]]]->GetVelocity(0);

        if (velocity_feedback->orocos_port.connected())
            velocity_feedback->orocos_port.write(velocity_feedback->joint_feedback);
    }
    if(torque_feedback){
        for(unsigned int i = 0; i < _number_of_dofs; ++i){
            gazebo::physics::JointWrench w =
                    _model->GetJoints()[_map_joint_name_indices[joint_names[i]]]->GetForceTorque(0u);
            gazebo::math::Vector3 a = _model->GetJoints()[_map_joint_name_indices[joint_names[i]]]->GetLocalAxis(
                    0u);
            torque_feedback->joint_feedback.torques(i) = a.Dot(w.body1Torque);
        }

        if (torque_feedback->orocos_port.connected())
            torque_feedback->orocos_port.write(torque_feedback->joint_feedback);
    }
}

void KinematicChain::getCommand()
{
    if(_current_control_mode == ControlModes::JointTorqueCtrl)
        torque_controller->joint_cmd_fs = torque_controller->orocos_port.readNewest(
                    torque_controller->joint_cmd);
    if(_current_control_mode == ControlModes::JointPositionCtrl || _current_control_mode == ControlModes::JointImpedanceCtrl)
        position_controller->joint_cmd_fs = position_controller->orocos_port.readNewest(
                    position_controller->joint_cmd);
    if(_current_control_mode == ControlModes::JointPositionCtrl)
        impedance_controller->joint_cmd_fs = impedance_controller->orocos_port.readNewest(
                    impedance_controller->joint_cmd);
}

void KinematicChain::move()
{
    if(_current_control_mode == ControlModes::JointPositionCtrl){
        std::vector<std::string> joint_scoped_names = getJointScopedNames();
        for(unsigned int i = 0; i < joint_scoped_names.size(); ++i)
            _gazebo_position_joint_controller->SetPositionTarget(joint_scoped_names[i], position_controller->joint_cmd.angles(i));
        _gazebo_position_joint_controller->Update();
    }
    else if(_current_control_mode == ControlModes::JointTorqueCtrl){
        std::vector<std::string> joint_names = getJointNames();
        for(unsigned int i = 0; i < joint_names.size(); ++i)
            _model->GetJoint(joint_names[i])->SetForce(0, torque_controller->joint_cmd.torques(i));
    }
    else if(_current_control_mode == ControlModes::JointImpedanceCtrl){
        ///TODO
    }
}
