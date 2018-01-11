#include <rtt-gazebo-robot-sim.hpp>
#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>
#include <gazebo/sensors/sensors.hh>


using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

robotSim::robotSim(const std::string &name):
#ifdef USE_INTROSPECTION
    cogimon::RTTIntrospectionBase(name),
#else
    TaskContext(name),
#endif
    is_configured(false),
    _models_loaded(false)
{
    this->provides("gazebo")->addOperation("WorldUpdateBegin",
            &robotSim::WorldUpdateBegin, this, RTT::ClientThread);
    this->provides("gazebo")->addOperation("WorldUpdateEnd",
            &robotSim::WorldUpdateEnd, this, RTT::ClientThread);

    this->addOperation("getModel", &robotSim::getModel,
                this, ClientThread);

    this->addOperation("setControlMode", &robotSim::setControlMode,
                this, RTT::ClientThread);

    this->addOperation("getKinematicChains", &robotSim::getKinematiChains,
                this, RTT::ClientThread);

    this->addOperation("getKinematicChainsAndJoints", &robotSim::getKinematiChainsAndJoints,
                this, RTT::ClientThread);

    this->addOperation("printKinematicChainInformation", &robotSim::printKinematicChainInformation,
                this, RTT::ClientThread);

    this->addOperation("getControlMode", &robotSim::getControlMode,
                this, RTT::ClientThread);

    this->addOperation("getAvailableControlMode", &robotSim::getControlAvailableMode,
                this, RTT::ClientThread);

    this->addOperation("loadURDFAndSRDF", &robotSim::loadURDFAndSRDF,
                this, RTT::ClientThread);

    this->addOperation("reset_model_configuration", &robotSim::resetModelConfiguration,
                this, RTT::ClientThread);

    this->addOperation("setInitialPosition", &robotSim::setInitialPosition,
                this, RTT::ClientThread);

    this->addOperation("getForceTorqueSensorsFrames", &robotSim::getForceTorqueSensorsFrames,
                this, RTT::ClientThread);

    this->addOperation("getLinkPoseVelocityGazebo", &robotSim::getLinkPoseVelocityGazebo,
                this, RTT::ClientThread);

    world_begin = gazebo::event::Events::ConnectWorldUpdateBegin(
            boost::bind(&robotSim::WorldUpdateBegin, this));
    world_end = gazebo::event::Events::ConnectWorldUpdateEnd(
            boost::bind(&robotSim::WorldUpdateEnd, this));

#ifdef USE_INTROSPECTION
    cts_worldUpdate = rstrt::monitoring::CallTraceSample("WorldUpdate()", this->getName(), 0.0, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);
#endif
}

bool robotSim::resetModelConfiguration()
{
    RTT::log(RTT::Info)<<"Reset Model Configuration has been called!"<<RTT::endlog();
    bool reset = true;
    std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it;
    for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        reset = reset && it->second->resetKinematicChain();
    return reset;
}

std::string robotSim::printKinematicChainInformation(const std::string& kinematic_chain)
{
    std::vector<std::string> chain_names = getKinematiChains();
    if(!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain) != chain_names.end())){
        log(Warning) << "Kinematic Chain " << kinematic_chain << " is not available!" << endlog();
        return "";}

    return kinematic_chains[kinematic_chain]->printKinematicChainInformation();
}

std::map<std::string, std::vector<std::string> > robotSim::getKinematiChainsAndJoints()
{
    std::map<std::string, std::vector<std::string> > kinematic_chains_and_joints_map;
    std::vector<std::string> chain_names = getKinematiChains();
    for(unsigned int i = 0; i < chain_names.size(); ++i){
        kinematic_chains_and_joints_map.insert(std::pair<std::string, std::vector<std::string>>(
            chain_names[i], kinematic_chains[chain_names[i]]->getJointNames()));
    }
    return kinematic_chains_and_joints_map;
}

std::string robotSim::getControlMode(const std::string& kinematic_chain)
{
    std::vector<std::string> chain_names = getKinematiChains();
        if(!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain) != chain_names.end())){
            log(Warning) << "Kinematic Chain " << kinematic_chain << " is not available!" << endlog();
            return "";}

    return kinematic_chains[kinematic_chain]->getCurrentControlMode();
}

std::vector<std::string> robotSim::getControlAvailableMode(const std::string& kinematic_chain)
{
    std::vector<std::string> control_modes;

    std::vector<std::string> chain_names = getKinematiChains();
    if(!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain) != chain_names.end())){
        log(Warning) << "Kinematic Chain " << kinematic_chain << " is not available!" << endlog();
        control_modes.push_back("");}
    else
        control_modes = kinematic_chains[kinematic_chain]->getControllersAvailable();
    return control_modes;
}

std::vector<std::string> robotSim::getKinematiChains()
{
    std::vector<std::string> chains;
    for(std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it = kinematic_chains.begin();
        it != kinematic_chains.end(); it++)
        chains.push_back(it->second->getKinematicChainName());
    return chains;
}

bool robotSim::setControlMode(const std::string& kinematic_chain, const std::string& controlMode)
{
    std::vector<std::string> chain_names = getKinematiChains();
    if(!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain) != chain_names.end())){
        log(Warning) << "Kinematic Chain " << kinematic_chain << " is not available!" << endlog();
        return false;}

    return kinematic_chains[kinematic_chain]->setControlMode(controlMode);
}

bool robotSim::getModel(const std::string& model_name) {
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

#ifdef USE_INTROSPECTION
void robotSim::updateHookInternal() {
#else
void robotSim::updateHook() {
#endif

}

#ifdef USE_INTROSPECTION
bool robotSim::startHookInternal() {
    return true;
}

void robotSim::stopHookInternal() {

}

void robotSim::cleanupHookInternal() {

}
#endif

#ifdef USE_INTROSPECTION
bool robotSim::configureHookInternal() {
#else
bool robotSim::configureHook() {
#endif
    this->is_configured = gazeboConfigureHook(model);
    return is_configured;
}

//Here we already have the model!
bool robotSim::gazeboConfigureHook(gazebo::physics::ModelPtr model) {
    if (model.get() == NULL) {
        RTT::log(RTT::Error) << "No model could be loaded" << RTT::endlog();
        return false;
    }

    if (!_models_loaded) {
        RTT::log(RTT::Error) << "URDF and SRDF models has not been passed. Call loadURDFAndSRDF(URDF_path, SRDF_path)" << RTT::endlog();
        return false;
    }

    // Get the joints
    gazebo_joints_ = model->GetJoints();
    model_links_ = model->GetLinks();

    RTT::log(RTT::Info) << "Model name "<< model->GetName() << RTT::endlog();
    RTT::log(RTT::Info) << "Model has " << gazebo_joints_.size() << " joints" << RTT::endlog();
    RTT::log(RTT::Info) << "Model has " << model_links_.size() << " links" << RTT::endlog();

    for(unsigned int i = 0; i < _xbotcore_model.get_chain_names().size(); ++i){
        std::string chain_name = _xbotcore_model.get_chain_names()[i];
        std::vector<std::string> enabled_joints_in_chain;
        _xbotcore_model.get_enabled_joints_in_chain(chain_name,enabled_joints_in_chain);

        kinematic_chains.insert(std::pair<std::string, boost::shared_ptr<KinematicChain>>(
            chain_name, boost::shared_ptr<KinematicChain>(
                new KinematicChain(chain_name, enabled_joints_in_chain, *(this->ports()), model
#ifdef USE_INTROSPECTION
                ,this
#endif
                ))));
    }

    RTT::log(RTT::Info) << "Kinematic Chains map created!" << RTT::endlog();

    for(std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it = kinematic_chains.begin();
        it != kinematic_chains.end(); it++){
        if(!(it->second->initKinematicChain(gains.Gains))){
            RTT::log(RTT::Warning) << "Problem Init Kinematic Chain" <<
                it->second->getKinematicChainName() << RTT::endlog();
            return false;
        }
    }
    RTT::log(RTT::Info) << "Kinematic Chains Initialized!" << RTT::endlog();



    RTT::log(RTT::Info) << "Checking Sensors (URDF/SRDF)"<<RTT::endlog();
    std::map<std::string,int> ft_srdf = _xbotcore_model.get_ft_sensors();


    RTT::log(RTT::Info) << "Force Update of SensorManager (before accessing sensors)!" << RTT::endlog();
    gazebo::sensors::SensorManager::Instance()->Update(true);

    RTT::log(RTT::Info) << "Checking Sensors (Gazebo)"<<RTT::endlog();
    gazebo::sensors::Sensor_V sensors = gazebo::sensors::SensorManager::Instance()->
            GetSensors();

    gazebo::sensors::Sensor_V sensors_attached_to_robot;
    for(unsigned int i = 0; i < sensors.size(); ++i){
        if(sensors[i]->ScopedName().find("::"+model->GetName()+"::") != std::string::npos)
            sensors_attached_to_robot.push_back(sensors[i]);
    }

    for(std::map<std::string,int>::iterator i = ft_srdf.begin();
        i != ft_srdf.end(); i++)
    {
        force_torque_sensor ft(i->first, model, _xbotcore_model.get_urdf_model(),
                               sensors_attached_to_robot,
                               *(this->ports())
    #ifdef USE_INTROSPECTION
                               ,this
    #endif
                               );
        if(ft.isInited())
            force_torque_sensors.push_back(ft);
    }





    RTT::log(RTT::Warning) << "Done Configuring Component" << RTT::endlog();
    return true;
}

bool robotSim::loadURDFAndSRDF(const std::string &URDF_path, const std::string &SRDF_path)
{
    if(!_models_loaded)
    {
        std::string _urdf_path = URDF_path;
        std::string _srdf_path = SRDF_path;

        RTT::log(RTT::Info)<<"URDF path: "<<_urdf_path<<RTT::endlog();
        RTT::log(RTT::Info)<<"SRDF path: "<<_srdf_path<<RTT::endlog();

        _models_loaded = _xbotcore_model.init(_urdf_path, _srdf_path);
        _models_loaded = _models_loaded && gains.initFile(_srdf_path);

        for(unsigned int i = 0; i < _xbotcore_model.get_chain_names().size(); ++i){
            RTT::log(RTT::Info)<<"chain #"<<i<<" "<<_xbotcore_model.get_chain_names()[i]<<RTT::endlog();
            std::vector<std::string> enabled_joints_in_chain_i;
            _xbotcore_model.get_enabled_joints_in_chain(_xbotcore_model.get_chain_names()[i], enabled_joints_in_chain_i);
            for(unsigned int j = 0; j < enabled_joints_in_chain_i.size(); ++j)
                RTT::log(RTT::Info)<<"  "<<enabled_joints_in_chain_i[j]<<RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Info)<<"URDF and SRDF have been already loaded!"<<RTT::endlog();

    return _models_loaded;
}

bool robotSim::setInitialPosition(const std::string& kin_chain, const std::vector<double>& init)
{
    std::vector<std::string> kin_chains = getKinematiChains();
    if(!(std::find(kin_chains.begin(), kin_chains.end(), kin_chain) != kin_chains.end())){
        RTT::log(RTT::Error)<<kin_chain<<" is not available!"<<RTT::endlog();
        return false;}

    bool a = kinematic_chains[kin_chain]->setInitialJointConfiguration(init);

    if(a){
        RTT::log(RTT::Info)<<kin_chain<<" home config: [ ";
        for(unsigned int i = 0; i < init.size(); ++i)
            RTT::log(RTT::Info)<<init[i]<<" ";
        RTT::log(RTT::Info)<<"]"<<RTT::endlog();}

    if(a)
        a = a && resetModelConfiguration();

    return a;
}

std::vector<std::string> robotSim::getForceTorqueSensorsFrames()
{
    std::vector<std::string> tmp;
    for(unsigned int i = 0; i < force_torque_sensors.size(); ++i)
        tmp.push_back(force_torque_sensors[i].getFrame());
    return tmp;
}

robotSim::~robotSim() {
    // Disconnect slots
    gazebo::event::Events::DisconnectWorldUpdateBegin(world_begin);
    gazebo::event::Events::DisconnectWorldUpdateEnd(world_end);
}

bool robotSim::getLinkPoseVelocityGazebo(const std::string& link_name, rstrt::geometry::Pose& pose,
                       rstrt::kinematics::Twist& twist)
{
    gazebo::physics::LinkPtr link = model->GetLink(link_name);
    if(link)
    {
        gazebo::math::Pose tmp = link->GetWorldPose();
        rstrt::geometry::Pose tmp_pose(tmp.pos.x, tmp.pos.y, tmp.pos.z, link_name,
                                   tmp.rot.w, tmp.rot.x, tmp.rot.y, tmp.rot.z, link_name);
        pose = tmp_pose;

        gazebo::math::Vector3 vel =  link->GetWorldLinearVel();
        gazebo::math::Vector3 omega = link->GetWorldAngularVel();
        twist.linear[0] = vel.x;
        twist.linear[1] = vel.y;
        twist.linear[2] = vel.z;
        twist.angular[0] = omega.x;
        twist.angular[1] = omega.y;
        twist.angular[2] = omega.z;

        return true;
    }

    RTT::log(RTT::Error)<<"Link "<<link_name<<" does not exists!"<<RTT::endlog();
    return false;
}

ORO_CREATE_COMPONENT_LIBRARY()
//ORO_CREATE_COMPONENT(cogimon::robotSim)
ORO_LIST_COMPONENT_TYPE(cogimon::robotSim)
