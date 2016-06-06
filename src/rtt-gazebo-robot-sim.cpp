#include <rtt-gazebo-robot-sim.hpp>
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

    this->addOperation("getModel", &robotSim::getModel,
                this, ClientThread);

    this->addOperation("setControlMode", &robotSim::setControlMode,
                this, RTT::ClientThread);

    this->addOperation("getKinematicChains", &robotSim::getKinematiChains,
                this, RTT::ClientThread);

    this->addOperation("printKinematicChainInformation", &robotSim::printKinematicChainInformation,
                this, RTT::ClientThread);

    world_begin = gazebo::event::Events::ConnectWorldUpdateBegin(
            boost::bind(&robotSim::WorldUpdateBegin, this));
    world_end = gazebo::event::Events::ConnectWorldUpdateEnd(
            boost::bind(&robotSim::WorldUpdateEnd, this));
}

std::string robotSim::printKinematicChainInformation(const std::string& kinematic_chain)
{
    std::vector<std::string> chain_names = getKinematiChains();
    if(!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain) != chain_names.end())){
        log(Warning) << "Kinematic Chain " << kinematic_chain << " is not available!" << endlog();
        return "";}

    return kinematic_chains[kinematic_chain]->printKinematicChainInformation();
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
        kinematic_chains[kinematic_chain]->getControllersAvailable();
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

    // Get the joints
    gazebo_joints_ = model->GetJoints();
    model_links_ = model->GetLinks();

    RTT::log(RTT::Info) << "Model name "<< model->GetName() << RTT::endlog();
    RTT::log(RTT::Info) << "Model has " << gazebo_joints_.size() << " joints" << RTT::endlog();
    RTT::log(RTT::Info) << "Model has " << model_links_.size() << " links" << RTT::endlog();

    hardcoded_chains chains;
    std::map<std::string, std::vector<std::string>>::iterator it;
    for(it = chains.map_chains_joints.begin(); it != chains.map_chains_joints.end(); it++)
        kinematic_chains.insert(std::pair<std::string, boost::shared_ptr<KinematicChain>>(
            it->first, boost::shared_ptr<KinematicChain>(
                                    new KinematicChain(it->first, it->second, *(this->ports()), model))));
    RTT::log(RTT::Info) << "Kinematic Chains map created!" << RTT::endlog();

    for(std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it = kinematic_chains.begin();
        it != kinematic_chains.end(); it++){
        if(!(it->second->initKinematicChain())){
            RTT::log(RTT::Warning) << "Problem Init Kinematic Chain" <<
                it->second->getKinematicChainName() << RTT::endlog();
            return false;
        }
    }
    RTT::log(RTT::Info) << "Kinematic Chains Initialized!" << RTT::endlog();

    RTT::log(RTT::Warning) << "Done configuring component" << RTT::endlog();
    return true;
}

ORO_CREATE_COMPONENT_LIBRARY()
//ORO_CREATE_COMPONENT(cogimon::robotSim)
ORO_LIST_COMPONENT_TYPE(cogimon::robotSim)

