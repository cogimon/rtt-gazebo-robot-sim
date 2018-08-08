#include <kinematic_chain.h>

using namespace cogimon;
KinematicChain::KinematicChain(const std::string& chain_name, const std::vector<std::string> &joint_names, RTT::DataFlowInterface& ports,
                               gazebo::physics::ModelPtr model
#ifdef USE_INTROSPECTION
                               ,cogimon::RTTIntrospectionBase* introBase
#endif
                               ):
    _kinematic_chain_name(chain_name),
    _ports(ports),
    _model(model),
    _current_control_mode(std::string(ControlModes::JointPositionCtrl)),
    _joint_names(joint_names)
#ifdef USE_INTROSPECTION
    ,_introBase(introBase)
#endif
{
    RTT::log(RTT::Info) << "Creating Kinematic Chain " << chain_name << RTT::endlog();

    for(unsigned int i = 0; i < _joint_names.size(); ++i)
        _map_joint_name_scoped_name.insert(std::pair<std::string, std::string>(_joint_names[i], " "));

    RTT::log(RTT::Info) << "Joints: " << RTT::endlog();
    for(unsigned int i = 0; i < _joint_names.size(); ++i)
        RTT::log(RTT::Info) << "    " << _joint_names[i] << RTT::endlog();

    for(unsigned int i = 0; i < _joint_names.size(); ++i)
        _initial_joints_configuration.push_back(0.0);


}

bool KinematicChain::initKinematicChain(const cogimon::gains& gains_)
{
    _gains.reset(new cogimon::gains(gains_));
    std::vector<std::string> controllers = _gains->map_controllers[_kinematic_chain_name];
    if(controllers.empty())
    {
        RTT::log(RTT::Error)<<"No controllers are available!"<<RTT::endlog();
        return false;
    }
    else
    {
        RTT::log(RTT::Info)<<"Available controllers for "<<_kinematic_chain_name<<" are:"<<RTT::endlog();
        for(unsigned int i = 0; i < controllers.size(); ++i)
            RTT::log(RTT::Info)<<"  "<<controllers[i]<<RTT::endlog();
    }

    setJointNamesAndIndices();
    _number_of_dofs = _map_joint_name_scoped_name.size();

    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointPositionCtrl)) != controllers.end()){
        if(!setController(std::string(ControlModes::JointPositionCtrl)))
            return false;
        else
            RTT::log(RTT::Info)<<std::string(ControlModes::JointPositionCtrl)<<" activated!"<<RTT::endlog();
    }
    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointVelocityCtrl)) != controllers.end()){
        if(!setController(std::string(ControlModes::JointVelocityCtrl)))
            return false;
        else
            RTT::log(RTT::Info)<<std::string(ControlModes::JointVelocityCtrl)<<" activated!"<<RTT::endlog();
    }
    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointImpedanceCtrl)) != controllers.end()){
        if(!setController(std::string(ControlModes::JointImpedanceCtrl)))
            return false;
        else
            RTT::log(RTT::Info)<<std::string(ControlModes::JointImpedanceCtrl)<<" activated!"<<RTT::endlog();
    }
    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointTorqueCtrl)) != controllers.end()){
        if(!setController(std::string(ControlModes::JointTorqueCtrl)))
            return false;
        else
            RTT::log(RTT::Info)<<std::string(ControlModes::JointTorqueCtrl)<<" activated!"<<RTT::endlog();}


    setFeedBack(); //We consider, for now, that the full feedback is available
    RTT::log(RTT::Info)<<"Full feedback activated!"<<RTT::endlog();

    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointPositionCtrl)) != controllers.end()){
        if(!initGazeboJointController(std::string(ControlModes::JointPositionCtrl)))
        {
            RTT::log(RTT::Error) << "Joint Controller can NOT be initialized, exiting" << RTT::endlog();
            return false;
        }
        else
        {
            RTT::log(RTT::Info)<<"Gazebo Joint Controlled inited!"<<RTT::endlog();
            setInitialPosition(false);
            RTT::log(RTT::Info)<<"Initial Position set!"<<RTT::endlog();
        }
    }
    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointVelocityCtrl)) != controllers.end()){
        if(!initGazeboJointController(std::string(ControlModes::JointVelocityCtrl)))
        {
            RTT::log(RTT::Error) << "Joint Controller can NOT be initialized, exiting" << RTT::endlog();
            return false;
        }
        else
        {
            RTT::log(RTT::Info)<<"Gazebo Joint Controlled inited!"<<RTT::endlog();
            setInitialPosition(false);
            RTT::log(RTT::Info)<<"Initial Position set!"<<RTT::endlog();
        }
    }
    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointImpedanceCtrl)) != controllers.end()){
        setInitialImpedance();
        RTT::log(RTT::Info)<<"Initial Impedance set!"<<RTT::endlog();}
    return true;
}

bool KinematicChain::resetKinematicChain()
{
    std::vector<std::string> controllers = _gains->map_controllers[_kinematic_chain_name];
    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointPositionCtrl)) == controllers.end()){
        RTT::log(RTT::Error)<<"Reset can be used only if "<<ControlModes::JointPositionCtrl<<" is available!"<<RTT::endlog();
        return false;}

    setControlMode(ControlModes::JointPositionCtrl);
    setInitialPosition(false);
    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointImpedanceCtrl)) != controllers.end())
        setInitialImpedance();
    return true;
}

std::vector<std::string> KinematicChain::getJointScopedNames()
{
    std::vector<std::string> joint_names;
    for(unsigned int i = 0; i < _joint_names.size(); ++i)
        joint_names.push_back(_map_joint_name_scoped_name[_joint_names[i]]);
    return joint_names;
}

std::vector<std::string> KinematicChain::getJointNames()
{
    return _joint_names;
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

void KinematicChain::setFeedBack()
{
		full_feedback.reset(new full_fbk);
		full_feedback->orocos_port.setName(_kinematic_chain_name+"_JointFeedback");
		full_feedback->orocos_port.doc("Output for Joint-fb from Gazebo to Orocos world. Contains joint-position, -velocity and -torque.");
		_ports.addPort(full_feedback->orocos_port);

		full_feedback->joint_feedback = JointState(_number_of_dofs);
		full_feedback->orocos_port.setDataSample(full_feedback->joint_feedback);
}

bool KinematicChain::setController(const std::string& controller_type)
{
    if(controller_type == ControlModes::JointPositionCtrl){
        position_controller.reset(new position_ctrl);
                position_controller->orocos_port.setName(_kinematic_chain_name+"_"+ControlModes::JointPositionCtrl);
                position_controller->orocos_port.doc("Input for JointPosition-cmds from Orocos to Gazebo world.");
                _ports.addPort(position_controller->orocos_port);

                position_controller->joint_cmd = JointAngles(_number_of_dofs);
                position_controller->joint_cmd.angles.setZero();

                _gazebo_position_joint_controller.reset(new gazebo::physics::JointController(_model));
    }
    else if(controller_type == ControlModes::JointVelocityCtrl){
        velocity_controller.reset(new velocity_ctrl);
                velocity_controller->orocos_port.setName(_kinematic_chain_name+"_"+ControlModes::JointVelocityCtrl);
                velocity_controller->orocos_port.doc("Input for JointVelocity-cmds from Orocos to Gazebo world.");
                _ports.addPort(velocity_controller->orocos_port);

                velocity_controller->joint_cmd = JointVelocities(_number_of_dofs);
                velocity_controller->joint_cmd.velocities.setZero();

                _gazebo_velocity_joint_controller.reset(new gazebo::physics::JointController(_model));
    }
    else if(controller_type == ControlModes::JointImpedanceCtrl){
    	impedance_controller.reset(new impedance_ctrl);
		impedance_controller->orocos_port.setName(_kinematic_chain_name+"_"+ControlModes::JointImpedanceCtrl);
		impedance_controller->orocos_port.doc("Input for JointImpedance-cmds from Orocos to Gazebo world.");
		_ports.addPort(impedance_controller->orocos_port);

		impedance_controller->joint_cmd = JointImpedance(_number_of_dofs);
		impedance_controller->joint_cmd.stiffness.setZero();
		impedance_controller->joint_cmd.damping.setZero();
    }
    else if(controller_type == ControlModes::JointTorqueCtrl)
    {
    	torque_controller.reset(new torque_ctrl);
		torque_controller->orocos_port.setName(_kinematic_chain_name+"_"+ControlModes::JointTorqueCtrl);
		torque_controller->orocos_port.doc("Input for JointTorque-cmds from Orocos to Gazebo world.");
		_ports.addPort(torque_controller->orocos_port);

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
    std::map<std::string, std::string>::iterator it1;
    for(it1 = _map_joint_name_scoped_name.begin(); it1 != _map_joint_name_scoped_name.end(); it1++)
    {
        gazebo::physics::JointPtr joint = _model->GetJoint(it1->first);
        if(!joint){
            RTT::log(RTT::Error) << "No Joint " << it1->first << " could be added, exiting" << RTT::endlog();
            return false;}
        _map_joint_name_scoped_name[it1->first] = _model->GetJoint(it1->first)->GetScopedName();

        RTT::log(RTT::Info) << "Joint " << it1->first << " in chain " << _kinematic_chain_name <<
                               " has scoped name " << _map_joint_name_scoped_name[it1->first] << RTT::endlog();
    }
    return true;
}

bool KinematicChain::initGazeboJointController(const std::string &controlMode)
{
    std::vector<std::string> joint_scoped_names = getJointScopedNames();
    if (controlMode == ControlModes::JointPositionCtrl)
    {
        for(unsigned int i = 0; i < _joint_names.size(); ++i)
                _gazebo_position_joint_controller->AddJoint(_model->GetJoint(_joint_names[i]));

        RTT::log(RTT::Info)<<_kinematic_chain_name<<" Position PIDs:"<<RTT::endlog();
        for(unsigned int i = 0; i < joint_scoped_names.size(); ++i){
            cogimon::PIDGain pid;
            _gains->getPID(_kinematic_chain_name, _joint_names[i], pid);
            _gazebo_position_joint_controller->SetPositionPID(joint_scoped_names[i], gazebo::common::PID(pid.P,pid.I,pid.D));
            RTT::log(RTT::Info)<<"  "<<pid.joint_name<<" P: "<<pid.P<<" I: "<<pid.I<<" D: "<<pid.D << " -> " << joint_scoped_names[i]<<RTT::endlog();
        }

        _gazebo_velocity_joint_controller->Reset();

        return true;
    }
    else if (controlMode == ControlModes::JointVelocityCtrl)
    {
        for(unsigned int i = 0; i < _joint_names.size(); ++i)
                _gazebo_velocity_joint_controller->AddJoint(_model->GetJoint(_joint_names[i]));

        RTT::log(RTT::Info)<<_kinematic_chain_name<<" Velocity PIDs:"<<RTT::endlog();
        for(unsigned int i = 0; i < joint_scoped_names.size(); ++i){
            cogimon::VelPIDGain velPid;
            _gains->getVelPID(_kinematic_chain_name, _joint_names[i], velPid);
            _gazebo_velocity_joint_controller->SetVelocityPID(joint_scoped_names[i], gazebo::common::PID(velPid.P,velPid.I,velPid.D));
            RTT::log(RTT::Info)<<"  "<<velPid.joint_name<<" P: "<<velPid.P<<" I: "<<velPid.I<<" D: "<<velPid.D<<RTT::endlog();
        }

        _gazebo_position_joint_controller->Reset();

        return true;
    }

    return false;
}

bool KinematicChain::runtimeVelPidUpdate(const std::string &joint_name, const double &p, const double &i, const double &d)
{
    std::vector<std::string> joint_scoped_names = getJointScopedNames();
    if(!(std::find(joint_scoped_names.begin(), joint_scoped_names.end(), joint_name) != joint_scoped_names.end())){
        RTT::log(RTT::Error)<<joint_name<<" is not available!"<<RTT::endlog();
        return false;}

    _gazebo_velocity_joint_controller->SetVelocityPID(joint_name, gazebo::common::PID(p,i,d));

    return true;
}

void KinematicChain::setInitialPosition(const bool use_actual_model_pose)
{
    position_controller->orocos_port.clear();
    velocity_controller->orocos_port.clear();
    if(use_actual_model_pose)
    {
        for(unsigned int i = 0; i < _joint_names.size(); ++i)
        {
            position_controller->joint_cmd.angles[i] = _model->GetJoint(_joint_names[i])->GetAngle(0).Radian();
            velocity_controller->joint_cmd.velocities[i] = _model->GetJoint(_joint_names[i])->GetVelocity(0);
        }
    }
    else
    {
        for(unsigned int i = 0; i < _joint_names.size(); ++i)
        {
            position_controller->joint_cmd.angles[i] = _initial_joints_configuration[i];
            velocity_controller->joint_cmd.velocities[i] = 0;
        }
        ///TODO: check if user initial config is set when it is used in the gazebo configure hook
    }
    position_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
    velocity_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
}

void KinematicChain::setInitialImpedance()
{
    RTT::log(RTT::Info)<<_kinematic_chain_name<<" impedance:"<<RTT::endlog();
    for(unsigned int i = 0; i < _joint_names.size(); ++i){
        cogimon::ImpedanceGain impedance;
        _gains->getImpedance(_kinematic_chain_name, _joint_names[i], impedance);
        impedance_controller->joint_cmd.stiffness[i] = impedance.stiffness;
        impedance_controller->joint_cmd.damping[i] = impedance.damping;
        RTT::log(RTT::Info)<<"  "<<impedance.joint_name<<" stiffness: "<<impedance.stiffness<<" damping: "<<impedance.damping<<RTT::endlog();
    }
    impedance_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
}

bool KinematicChain::setControlMode(const std::string &controlMode)
{
    if(controlMode != ControlModes::JointPositionCtrl &&
       controlMode != ControlModes::JointVelocityCtrl &&
       controlMode != ControlModes::JointTorqueCtrl   &&
       controlMode != ControlModes::JointImpedanceCtrl ){
        RTT::log(RTT::Warning) << "Control Mode " << controlMode << " does not exist!" << RTT::endlog();
        return false;
    }

    if(!(std::find(_controllers_name.begin(), _controllers_name.end(), controlMode) != _controllers_name.end())){
        RTT::log(RTT::Warning) << "Control Mode " << controlMode << " is not available!" << RTT::endlog();
        return false;
    }

    if(controlMode == ControlModes::JointPositionCtrl || controlMode == ControlModes::JointVelocityCtrl){
        if(!initGazeboJointController(controlMode)){
            RTT::log(RTT::Warning) << "Can NOT initialize Gazebo Joint Controller!" << RTT::endlog();
            return false;}
        else
            setInitialPosition();
    }
    else {
        _gazebo_position_joint_controller->Reset();
        _gazebo_velocity_joint_controller->Reset();
    }

    _current_control_mode = controlMode;
    return true;
}

void KinematicChain::sense()
{
	if (full_feedback) {
		for (unsigned int i = 0; i < _number_of_dofs; ++i)
			full_feedback->joint_feedback.angles(i) = _model->GetJoint(
					_joint_names[i])->GetAngle(0).Radian();

		for (unsigned int i = 0; i < _number_of_dofs; ++i)
			full_feedback->joint_feedback.velocities(i) = _model->GetJoint(
					_joint_names[i])->GetVelocity(0);

		for (unsigned int i = 0; i < _number_of_dofs; ++i) {
//			gazebo::physics::JointWrench w =
//					_model->GetJoint(_joint_names[i])->GetForceTorque(0u);
//			gazebo::math::Vector3 a =
//					_model->GetJoint(_joint_names[i])->GetLocalAxis(0u);
//			full_feedback->joint_feedback.torques(i) = a.Dot(w.body1Torque);
            full_feedback->joint_feedback.torques(i) =
                    _model->GetJoint(_joint_names[i])->GetForce(0u);
		}

		if (full_feedback->orocos_port.connected())
        #ifdef USE_INTROSPECTION
			_introBase->writePort(full_feedback->orocos_port,full_feedback->joint_feedback);
        #else
			full_feedback->orocos_port.write(full_feedback->joint_feedback);
        #endif
	}
}

void KinematicChain::getCommand()
{
    if(_current_control_mode == ControlModes::JointTorqueCtrl)
    #ifdef USE_INTROSPECTION
        torque_controller->joint_cmd_fs = _introBase->readPort(torque_controller->orocos_port,
                    torque_controller->joint_cmd);
    #else
        torque_controller->joint_cmd_fs = torque_controller->orocos_port.readNewest(
                    torque_controller->joint_cmd);
    #endif
    else if(_current_control_mode == ControlModes::JointPositionCtrl)
    #ifdef USE_INTROSPECTION
        position_controller->joint_cmd_fs = _introBase->readPort(position_controller->orocos_port,
                    position_controller->joint_cmd);
    #else
        position_controller->joint_cmd_fs = position_controller->orocos_port.readNewest(
                    position_controller->joint_cmd);
    #endif
    else if(_current_control_mode == ControlModes::JointVelocityCtrl)
    #ifdef USE_INTROSPECTION
        velocity_controller->joint_cmd_fs = _introBase->readPort(velocity_controller->orocos_port,
                    velocity_controller->joint_cmd);
    #else
        velocity_controller->joint_cmd_fs = velocity_controller->orocos_port.readNewest(
                    velocity_controller->joint_cmd);
    #endif
    else if(_current_control_mode == ControlModes::JointImpedanceCtrl){
    #ifdef USE_INTROSPECTION
        position_controller->joint_cmd_fs = _introBase->readPort(position_controller->orocos_port,
                    position_controller->joint_cmd);
        impedance_controller->joint_cmd_fs = _introBase->readPort(impedance_controller->orocos_port,
                    impedance_controller->joint_cmd);
        torque_controller->joint_cmd_fs = _introBase->readPort(torque_controller->orocos_port,
                    torque_controller->joint_cmd);
    #else
        position_controller->joint_cmd_fs = position_controller->orocos_port.readNewest(
                    position_controller->joint_cmd);
        impedance_controller->joint_cmd_fs = impedance_controller->orocos_port.readNewest(
                    impedance_controller->joint_cmd);
        torque_controller->joint_cmd_fs = torque_controller->orocos_port.readNewest(
                    torque_controller->joint_cmd);
    #endif
    }
}

void KinematicChain::move()
{
    if(_current_control_mode == ControlModes::JointPositionCtrl){
        std::vector<std::string> joint_scoped_names = getJointScopedNames();
        for(unsigned int i = 0; i < joint_scoped_names.size(); ++i)
            _gazebo_position_joint_controller->SetPositionTarget(joint_scoped_names[i], position_controller->joint_cmd.angles(i));
        _gazebo_position_joint_controller->Update();
    }
    else if(_current_control_mode == ControlModes::JointVelocityCtrl){
        std::vector<std::string> joint_scoped_names = getJointScopedNames();
        for(unsigned int i = 0; i < joint_scoped_names.size(); ++i)
            _gazebo_velocity_joint_controller->SetVelocityTarget(joint_scoped_names[i], velocity_controller->joint_cmd.velocities(i));
        _gazebo_velocity_joint_controller->Update();
    }
    else if(_current_control_mode == ControlModes::JointTorqueCtrl){
        for(unsigned int i = 0; i < _joint_names.size(); ++i)
            _model->GetJoint(_joint_names[i])->SetForce(0, torque_controller->joint_cmd.torques(i));
    }
    else if(_current_control_mode == ControlModes::JointImpedanceCtrl){
        for(unsigned int i = 0; i < _joint_names.size(); ++i){
            double q = full_feedback->joint_feedback.angles[i];
            double qd = position_controller->joint_cmd.angles[i];
            double Kd = impedance_controller->joint_cmd.stiffness[i];
            double qdot = full_feedback->joint_feedback.velocities[i];
            double Dd = impedance_controller->joint_cmd.damping[i];
            double tauoff = torque_controller->joint_cmd.torques[i];
            double tau = -Kd*(q-qd)-Dd*qdot+tauoff;
            _model->GetJoint(_joint_names[i])->SetForce(0, tau);
        }
    }
}

std::string KinematicChain::printKinematicChainInformation()
{
    std::stringstream joint_names_stream;
    for(unsigned int i = 0; i < _joint_names.size(); ++i)
        joint_names_stream << _joint_names[i] << " ";

    std::vector<std::string> controller_names = getControllersAvailable();
    std::stringstream controller_names_stream;
    for(unsigned int i = 0; i < controller_names.size(); ++i)
        controller_names_stream << controller_names[i] << " ";

    std::stringstream info;
    info << "Kinematic Chain: " << _kinematic_chain_name << std::endl;
    info << "    Number of DOFs: " << _number_of_dofs << std::endl;
    info << "    Joints:  [" << joint_names_stream.str() << "]" << std::endl;
    info << "    Control Modes:  [ " << controller_names_stream.str() << "]" << std::endl;
    info << "    Current Control Mode: " << _current_control_mode << std::endl;

    return info.str();
}

bool KinematicChain::setInitialJointConfiguration(const std::vector<double>& home)
{
    if(home.size() != _initial_joints_configuration.size()){
	RTT::log(RTT::Error)<<"Configuration size mismatch!"<<RTT::endlog();
        return false;
    }
    _initial_joints_configuration = home;
    return true;
}
